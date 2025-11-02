// ObtencionDatosSimultaneaEnergiaEspectral.cpp
// Obtención de datos simultánea de MPU6500 (IMU) y MAX30102
// Extrae características estadísticas y espectrales en ventanas deslizantes
// - IMU: acelerómetro y giróscopo a 200 Hz
// - PPG: MAX30102 (IR o RED) a 100 Hz
// - Ventanas de 2 s con 50% de solapamiento
// - Características estadísticas: RMS, varianza, energía, pico a pico, asimetría, curtosis y media (7 por canal)
// - Características espectrales: fracción de energía en bandas específicas (7 en total
//   - PPG: 0.10-0.40 Hz, 0.70-3.00 Hz, 3.00-5.00 Hz
//   - IMU (magnitud): 0.30-3.00 Hz, 3.00-12.00 Hz (acelerómetro y giróscopo)
// - Emisión por Serial Monitor o a Python (62 características por ventana)
// - Plataforma: ESP32 (ESP32 WROOM-32)
// - Librerías: ArduinoFFT, MPU6500, SparkFun MAX3010x5
// - Fecha: octubre 2024
// Librerías
#include <Arduino.h>
#include <Wire.h>
#include <MPU6500.h>
#include "MAX30105.h"
#include <math.h>     
#include <string.h>   
#include "esp_timer.h"
#include <arduinoFFT.h>
#include <limits.h>

// Defines
#define PRINT_TIMESTAMP     0   // 1: agrega timestamp µs al inicio de cada fila emitida
#define BAUDIOS             115200  // velocidad serial
#define FS_IMU_HZ           200    // IMU 200 Hz
#define FS_PPG_HZ           100    // PPG 100 Hz
#define WINDOW_SEC          2.0f   // 2 s
#define OVERLAP             0.5f   // 50%
#define USE_G_IN_MPS2       1      // 1: convertir 'g' a m/s^2 (opcional)
#define SDA_PIN             21     // GPIO21
#define SCL_PIN             22     // GPIO22
#define I2C_ADDR_MPU        0x68   // MPU6500
#define I2C_ADDR_MAX        0x57   // MAX30102
#define FeaturesPerChannel  7      // 7 características por canal
#define FeaturesIMU         6 * FeaturesPerChannel + 6 // 42 + 6 xcorr
#define FeaturesPPG         FeaturesPerChannel         // 7
#define TotalFeatures       FeaturesIMU + FeaturesPPG + FeaturesPerChannel // 55 + 7 espectrales
#define USE_PYTHON          0  // 1: enviar a Python; 0: enviar a Serial Monitor
#define USE_IR              1  // 1: usar IR; 0: usar RED
#define DEBUG               1  // 1: debug info por Serial; 0: nada (No Usar con Python)

// Objetos
MPU6500 mpu;
MAX30105 ppg;
struct FFTWork; 

// CONTEXTO para pasar punteros y metadatos a las rutinas
struct FFTWork {
  float* vReal;
  float* vImag;
  int    N;
  float  fs;
  const float* hann;
};

// Tamaños de ventana y hop
constexpr int IMU_WIN = int(WINDOW_SEC * FS_IMU_HZ);      // 2s -> 400
constexpr int IMU_HOP = int(IMU_WIN * (1.0f - OVERLAP));  // 200
constexpr int PPG_WIN = int(WINDOW_SEC * FS_PPG_HZ);      // 2s -> 200
constexpr int PPG_HOP = int(PPG_WIN * (1.0f - OVERLAP));  // 100

// Buffers y variables globales
static float ax_win[IMU_WIN], ay_win[IMU_WIN], az_win[IMU_WIN];
static float gx_win[IMU_WIN], gy_win[IMU_WIN], gz_win[IMU_WIN];
static float ax_buf[IMU_WIN], ay_buf[IMU_WIN], az_buf[IMU_WIN];
static float gx_buf[IMU_WIN], gy_buf[IMU_WIN], gz_buf[IMU_WIN];
static float ppg_buf[PPG_WIN];
static float ppg_win[PPG_WIN];

volatile int imu_widx = 0;   // inddex escritura IMU
volatile int imu_ready = 0;  // flags desde ISR
volatile unsigned long imu_total = 0; // total muestras IMU
volatile int ppg_widx = 0; // index escritura PPG
volatile int ppg_ready = 0; // flags desde ISR
volatile unsigned long ppg_total = 0; // total muestras PPG

static unsigned long last_total_mpu = 0; // para evitar backlog
static unsigned long last_total_ppg = 0; // para evitar backlog
static unsigned long last_emit_mpu = 0; // para debug
static unsigned long last_emit_ppg = 0; // para debug

// Parámetros FFT
constexpr int N_PPG  = 256;   // 2 s @100 Hz (~200 -> zero-pad a 256)
constexpr int N_IMU  = 512;   // 2 s @200 Hz (~400 -> zero-pad a 512)

// Ventanas y buffers reutilizables
static float hann_ppg[N_PPG], hann_imu[N_IMU];
static float vReal_ppg[N_PPG], vImag_ppg[N_PPG];
static float vReal_imu[N_IMU], vImag_imu[N_IMU];

// “descriptores” para PPG e IMU
static FFTWork W_PPG { vReal_ppg, vImag_ppg, N_PPG, (float)FS_PPG_HZ, hann_ppg };
static FFTWork W_IMU { vReal_imu, vImag_imu, N_IMU, (float)FS_IMU_HZ, hann_imu };

// Timers y sincronización
esp_timer_handle_t imu_timer; // timer para IMU
esp_timer_handle_t ppg_timer; // timer para PPG
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // mutex para variables compartidas

// ====== Helpers de depuración ======
#if DEBUG
  // Macro: ejecuta cada N llamadas con un contador local/estático
  #define DBG_EVERY(cntVar, N)  ( (++(cntVar) >= (N)) ? ((cntVar)=0, true) : false )

  // Temporizador por tiempo (no por ticks)
  static inline bool every_ms(uint32_t ms){
    static uint32_t last=0; uint32_t now=millis();
    if (now - last >= ms){ last = now; return true; }
    return false;
  }

  // Estadísticos Welford de dt (µs)
  struct TimerStats {
    unsigned long n=0; double mean=0, M2=0; unsigned long minv=ULONG_MAX, maxv=0; unsigned long last=0;
    inline void update(unsigned long now){ if(last==0){ last=now; return; } unsigned long dt=now-last; last=now; n++; double x=(double)dt; double d=x-mean; mean+=d/n; M2+=d*(x-mean); if(dt<minv)minv=dt; if(dt>maxv)maxv=dt; }
    inline void dump(const char* tag){ double var=(n>1)?(M2/(n-1)):0.0; Serial.printf("[%s] n=%lu dt_mean=%.1f us std=%.1f min=%lu max=%lu\n", tag, n, mean, sqrt(var), minv, maxv); }
  };
  static TimerStats TS_IMU, TS_PPG;   // globales

  // Monitoreo de backlogs y consumos
  static volatile uint32_t imu_isr_ticks=0, ppg_isr_ticks=0;
  static uint32_t imu_loop_consumed=0, ppg_loop_consumed=0;

  // Watermarks de índices de escritura en ventana
  static int imu_widx_max=0, ppg_widx_max=0;

  // Perfil de emisión (cadencia de maybeEmitOnce)
  static unsigned long last_emit_us=0;
  static inline void mark_emit(){ unsigned long now=esp_timer_get_time(); if(last_emit_us){ unsigned long dt=now-last_emit_us; Serial.printf("Emit dt=%.1f ms\n", dt/1000.0); } last_emit_us=now; }

  // Heap
  extern "C" uint32_t esp_get_free_heap_size();
#endif

// Utilidades estadísticas
//Calculo Raíz Media Cuadrática
inline float rms(const float* x, int n) {
  double s = 0.0;
  for (int i=0;i<n;++i) s += (double)x[i]*x[i]; // suma de cuadrados;
  return sqrtf((float)(s/n) + 1e-12f); // +1e-12f para evitar sqrt(0)
}
//Calculo Media aritmética
inline float mean(const float* x, int n) {
  double s=0.0; for (int i=0;i<n;++i) s += x[i]; // suma
  return (float)(s/n); // media
}
//Calculo Varianza
inline float var(const float* x, int n, float mu=-9999.0f) {
  double s=0.0; float m = (mu==-9999.0f? mean(x,n): mu); // media
  for (int i=0;i<n;++i){ double d = x[i]-m; s += d*d; } // suma de cuadrados de diferencias de la media
  return (float)(s/n);
}
//Calculo Energía
inline float energy(const float* x, int n) {
  double s=0.0; for (int i=0;i<n;++i) s += (double)x[i]*x[i];
  return (float)(s/n); // energía media (= RMS^2)
}
//Calculo Pico a Pico
inline float ptp(const float* x, int n) {
  float mn=x[0], mx=x[0];
  for (int i=1;i<n;++i){ if(x[i]<mn) mn=x[i]; if(x[i]>mx) mx=x[i]; } // min y max
  return mx - mn;
}
//Calculo Asimetría
inline float skewness(const float* x, int n) {
  if (n<3) return 0.0f;
  float m = mean(x,n); // media
  float s2 = 0.0f, s3 = 0.0f; // varianza y tercer momento
  for (int i=0;i<n;++i){ float d=x[i]-m; s2 += d*d; s3 += d*d*d; } 
  s2 /= n; s3 /= n; // momentos
  float sd = sqrtf(s2 + 1e-12f); // desviación típica
  return (s3 / (sd*sd*sd + 1e-12f)); // asimetría
}
//Calculo Curtosis (exceso)
inline float kurtosis_excess(const float* x, int n) {
  if (n<4) return 0.0f;
  float m = mean(x,n); // media
  double s2=0.0, s4=0.0; // varianza y cuarto momento
  for (int i=0;i<n;++i){ double d=x[i]-m; s2 += d*d; s4 += d*d*d*d; } 
  s2/=n; s4/=n; // momentos
  double k = s4 / ((s2*s2)+1e-18); // curtosis
  return (float)(k - 3.0); // exceso
}
//Calculo Correlación Cruzada Normalizada (lag=0)
inline float xcorr0(const float* a, const float* b, int n){
  float ma = mean(a,n), mb = mean(b,n); // medias
  double num=0.0, va=0.0, vb=0.0; // numerador y varianzas
  for(int i=0;i<n;++i){ 
    double da=a[i]-ma, db=b[i]-mb; // diferencias
    num += da*db; va += da*da; vb += db*db; // sumas
  }
  double den = sqrt((va+1e-18)*(vb+1e-18)); // denominador
  return (float)(num/(den+1e-18)); // correlación
}

// Sensores y muestreo
void onIMUTick_task(void* /*arg*/) {
  portENTER_CRITICAL(&timerMux); // proteger variables compartidas
  imu_ready++; // flag para loop
  #if DEBUG
    imu_isr_ticks++; // conteo ISR
  #endif
  portEXIT_CRITICAL(&timerMux); // liberar mutex

  #if DEBUG
    // Estadísticos de periodo (ligeros)
    TS_IMU.update(esp_timer_get_time());

    // Tu debug existente cada 200 ticks (respetado)
    static unsigned long last = 0; static int counter = 0;
    if (DBG_EVERY(counter, 200)) { // ~1 s
      unsigned long now = esp_timer_get_time();
      if (last != 0){
        unsigned long dt = now - last; float freq = 1e6f / (float)dt;
        Serial.print("IMU dt (us): "); Serial.println(dt);
        Serial.print("Frecuencia IMU: "); Serial.print(freq, 2); Serial.println(" Hz");
      }
      last = now;
    }
  #endif
}

void onPPGTick_task(void* /*arg*/){
  portENTER_CRITICAL(&timerMux); // proteger variables compartidas
  ppg_ready++; // flag para loop
  #if DEBUG
    ppg_isr_ticks++; // conteo ISR
  #endif
  portEXIT_CRITICAL(&timerMux); // liberar mutex

  #if DEBUG
    TS_PPG.update(esp_timer_get_time());
    static unsigned long last = 0; static int counter = 0;
    if (DBG_EVERY(counter, 100)) { // ~1 s
      unsigned long now = esp_timer_get_time();
      if (last != 0){
        unsigned long dt = now - last; float freq = 1e6f / (float)dt;
        Serial.print("PPG dt (us): "); Serial.println(dt);
        Serial.print("Frecuencia PPG: "); Serial.print(freq, 2); Serial.println(" Hz");
      }
      last = now;
    }
  #endif
}

// Toma una muestra de la IMU y la almacena en el buffer circular
void imuSample(){
  float ax, ay, az, gx, gy, gz;
  mpu.readAccelG(ax, ay, az); // aceleración en 'g'
  mpu.readGyroDps(gx, gy, gz); // giro en dps

  #if USE_G_IN_MPS2
    const float G = 9.80665f;
    ax*=G; ay*=G; az*=G;
  #endif

  if (imu_widx >= IMU_WIN){
    memmove(ax_buf, ax_buf+IMU_HOP, sizeof(float)*(IMU_WIN-IMU_HOP));
    memmove(ay_buf, ay_buf+IMU_HOP, sizeof(float)*(IMU_WIN-IMU_HOP)); 
    memmove(az_buf, az_buf+IMU_HOP, sizeof(float)*(IMU_WIN-IMU_HOP));
    memmove(gx_buf, gx_buf+IMU_HOP, sizeof(float)*(IMU_WIN-IMU_HOP));
    memmove(gy_buf, gy_buf+IMU_HOP, sizeof(float)*(IMU_WIN-IMU_HOP));
    memmove(gz_buf, gz_buf+IMU_HOP, sizeof(float)*(IMU_WIN-IMU_HOP));
    imu_widx = IMU_WIN-IMU_HOP;
  }
  ax_buf[imu_widx] = ax; ay_buf[imu_widx] = ay; az_buf[imu_widx] = az;
  gx_buf[imu_widx] = gx; gy_buf[imu_widx] = gy; gz_buf[imu_widx] = gz;
  imu_widx++;
  imu_total++;

  #if DEBUG
    if (imu_widx > imu_widx_max) imu_widx_max = imu_widx; // watermark
    static int count = 0;
    if ((++count % 100) == 0) {
      Serial.print("IMU sample: ax="); Serial.print(ax); 
      Serial.print(", ay="); Serial.print(ay);
      Serial.print(", az="); Serial.print(az);
      Serial.print(", gx="); Serial.print(gx);
      Serial.print(", gy="); Serial.print(gy);
      Serial.print(", gz="); Serial.println(gz);
    }
  #endif
}

// Toma muestras del PPG y las almacena en el buffer circular
void ppgSample(){
  // Mueve FIFO interno de la lib (no bloqueante)
  ppg.safeCheck(0);

  // Drenar TODO el FIFO disponible
  while (ppg.available()) {
    uint32_t ir  = ppg.getIR();
    uint32_t red = ppg.getRed();
    ppg.nextSample();

    float sample = USE_IR ? (float)ir : (float)red;

    if (ppg_widx >= PPG_WIN){
      // Desplaza por HOP
      memmove(ppg_buf, ppg_buf + PPG_HOP, sizeof(float)*(PPG_WIN - PPG_HOP));
      ppg_widx = PPG_WIN - PPG_HOP;
    }
    ppg_buf[ppg_widx++] = sample;
    ppg_total++;
  }

  #if DEBUG
    if (ppg_widx > ppg_widx_max) ppg_widx_max = ppg_widx; // watermark
    static int count = 0;
    if ((++count % 100) == 0) {
      Serial.print("PPG sample: ");
      Serial.println(ppg_buf[(ppg_widx>0?ppg_widx-1:0)]);
    }
  #endif
}

// calcula y añade las 7 características de un canal en out[idxBase..idxBase+6]
static inline void add_channel_mpu(const float* x, int n, int idxBase, float* out){
  float mu = mean(x,n);
  out[idxBase+0] = rms(x,n);
  out[idxBase+1] = var(x,n, mu);
  out[idxBase+2] = energy(x,n);
  out[idxBase+3] = ptp(x,n);
  out[idxBase+4] = skewness(x,n);
  out[idxBase+5] = kurtosis_excess(x,n);
  out[idxBase+6] = mu;
}
// Computa las 7 características desde copias locales (para evitar condición de carrera)
void computePPGFeatures_fromCopies(float* out7, const float* x, int n){
  float mu = mean(x,n);
  out7[0] = rms(x,n);
  out7[1] = var(x,n);         
  out7[2] = energy(x,n);
  out7[3] = ptp(x,n);
  out7[4] = skewness(x,n);
  out7[5] = kurtosis_excess(x,n);
  out7[6] = mu;
}

// Computa features desde copias locales (para evitar condición de carrera)
void computeMPUFeatures_fromCopies(float* out48,
  const float* ax,const float* ay,const float* az,
  const float* gx,const float* gy,const float* gz)
{
  int idx = 0;

  add_channel_mpu(ax, IMU_WIN, idx, out48); idx+=7;
  add_channel_mpu(ay, IMU_WIN, idx, out48); idx+=7;
  add_channel_mpu(az, IMU_WIN, idx, out48); idx+=7;
  add_channel_mpu(gx, IMU_WIN, idx, out48); idx+=7;
  add_channel_mpu(gy, IMU_WIN, idx, out48); idx+=7;
  add_channel_mpu(gz, IMU_WIN, idx, out48); idx+=7;

  // xcorr (6)
  out48[idx++] = xcorr0(ax, ay, IMU_WIN);
  out48[idx++] = xcorr0(ax, az, IMU_WIN);
  out48[idx++] = xcorr0(ay, az, IMU_WIN);
  out48[idx++] = xcorr0(gx, gy, IMU_WIN);
  out48[idx++] = xcorr0(gx, gz, IMU_WIN);
  out48[idx++] = xcorr0(gy, gz, IMU_WIN);

  #if DEBUG
    Serial.println("Características IMU calculadas:");
    for (int i=0; i<48; i++) {
      Serial.print(out48[i], 6);
      Serial.print(", ");
    }
    Serial.println();
  #endif
}

// Print features as CSV line (con opción de timestamp)
void printFeaturesCSV(const float* f, int n){
  #if PRINT_TIMESTAMP
    unsigned long us = esp_timer_get_time();
    Serial.print(us); Serial.print(',');
  #endif
  for(int i=0;i<n;++i){
    Serial.print(f[i], 6);
    if (i<n-1) Serial.print(',');
  }
  Serial.println();
}

// Precompute Hann window (para FFT)
inline void precomputeHann(float* w, int N) {
  const float kTwoPi = 6.283185307179586f;   // NO usar 'twoPi' (macro de la lib)
  for (int i=0;i<N;++i) {
    w[i] = 0.5f*(1.0f - cosf(kTwoPi * i / (N - 1)));
  }
}

// Convierte frecuencia (Hz) a bin FFT (0..N/2)
// - fs: frecuencia de muestreo
inline int hz2bin(float f, float fs, int N) {
  int k = int((f * N) / fs + 0.5f);
  if (k < 0) k = 0;
  if (k > N/2) k = N/2;
  return k;
}

// Fracción de energía en banda [f1,f2] con:
// - media calculada SOLO sobre muestras válidas (sin contaminar por el padding)
// - exclusión del bin DC (k=0) en Etot y en la banda
static float bandpower_frac_window(const float* x_win, int valid_n, const FFTWork& W, float f1, float f2) {
  // 1) Media sólo de la parte válida
  float mu = 0.f;
  for (int i = 0; i < valid_n; ++i) mu += x_win[i];
  mu /= (float)valid_n;

  // 2) Cargar buffer FFT: válidos centrados en mu con Hann; resto = 0
  for (int i = 0; i < W.N; ++i) {
    float x = (i < valid_n ? (x_win[i] - mu) : 0.f);
    W.vReal[i] = x * W.hann[i];
    W.vImag[i] = 0.f;
  }

  // 3) FFT → magnitud
  ArduinoFFT<float> fft(W.vReal, W.vImag, (uint16_t)W.N, W.fs, false);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();   // vReal[k] = |X[k]|

  // 4) Potencia y sumas (excluir DC: k=0)
  float Etot = 0.f;
  for (int k = 1; k <= W.N/2; ++k) {
    float p = W.vReal[k] * W.vReal[k];
    W.vReal[k] = p;   // reutiliza vReal como espectro de potencia
    Etot += p;
  }

  // 5) Límites de banda en bins (garantizar k>=1)
  int k1 = hz2bin(f1, W.fs, W.N);
  int k2 = hz2bin(f2, W.fs, W.N);
  if (k2 < k1) { int t = k1; k1 = k2; k2 = t; }
  if (k1 < 1) k1 = 1;              // excluye DC
  if (k2 > W.N/2) k2 = W.N/2;

  float E = 0.f;
  for (int k = k1; k <= k2; ++k) E += W.vReal[k];

  return E / (Etot + 1e-12f);
}

// Calcula las 7 espectrales sobre ppg_win + magnitudes de IMU
void computeSpectralPlus7(const float* ppg_ir, int n_ppg,
                          const float* ax, const float* ay, const float* az, int n_imu,
                          const float* gx, const float* gy, const float* gz,
                          float out7[7]) {
  // 1) PPG -> copiar/zero-pad a 256
  static float tmp_ppg[N_PPG];
  memset(tmp_ppg, 0, sizeof(tmp_ppg));
  int c_ppg = (n_ppg < N_PPG ? n_ppg : N_PPG);
  memcpy(tmp_ppg, ppg_ir, sizeof(float) * c_ppg);

  // Bandas PPG: B0=0.10–0.40, B1=0.70–3.00, B2=3.00–5.00
  out7[0] = bandpower_frac_window(tmp_ppg, c_ppg, W_PPG, 0.10f, 0.40f); // ppg_B0_frac
  out7[1] = bandpower_frac_window(tmp_ppg, c_ppg, W_PPG, 0.70f, 3.00f); // ppg_B1_frac
  out7[2] = bandpower_frac_window(tmp_ppg, c_ppg, W_PPG, 3.00f, 5.00f); // ppg_B2_frac

  // 2) Magnitudes IMU (|a|, |g|) -> zero-pad a 512
  static float acc_mag[N_IMU], gyro_mag[N_IMU];
  int c_imu = (n_imu < N_IMU ? n_imu : N_IMU);
  for (int i=0;i<c_imu;++i) {
    float axv=ax[i], ayv=ay[i], azv=az[i];
    float gxv=gx[i], gyv=gy[i], gzv=gz[i];
    acc_mag[i]  = sqrtf(axv*axv + ayv*ayv + azv*azv);
    gyro_mag[i] = sqrtf(gxv*gxv + gyv*gyv + gzv*gzv);
  }
  for (int i=c_imu;i<N_IMU;++i){ acc_mag[i]=0.f; gyro_mag[i]=0.f; }

  // Bandas IMU: L=0.30–3.00, M=3.00–12.00
  out7[3] = bandpower_frac_window(acc_mag, c_imu, W_IMU, 0.30f, 3.00f);   // acc_L_frac
  out7[4] = bandpower_frac_window(acc_mag, c_imu, W_IMU, 3.00f, 12.00f);  // acc_M_frac
  out7[5] = bandpower_frac_window(gyro_mag, c_imu, W_IMU, 0.30f, 3.00f);   // gyro_L_frac
  out7[6] = bandpower_frac_window(gyro_mag, c_imu, W_IMU, 3.00f, 12.00f);  // gyro_M_frac
}

// Emite UNA ventana si ambos flujos tienen ≥1 hop pendiente
void maybeEmitOnce() {
  if (imu_total < IMU_WIN || ppg_total < PPG_WIN) return; // no hay ventana completa

  unsigned long imu_pending = (imu_total - last_emit_mpu);
  unsigned long ppg_pending = (ppg_total - last_emit_ppg);

  // ¿tenemos al menos un hop completo en ambos?
  if (imu_pending >= (unsigned long)IMU_HOP && ppg_pending >= (unsigned long)PPG_HOP) {
    // Copias locales
    memcpy(ax_win, ax_buf, sizeof ax_win);
    memcpy(ay_win, ay_buf, sizeof ay_win);
    memcpy(az_win, az_buf, sizeof az_win);
    memcpy(gx_win, gx_buf, sizeof gx_win);
    memcpy(gy_win, gy_buf, sizeof gy_win);
    memcpy(gz_win, gz_buf, sizeof gz_win);
    memcpy(ppg_win, ppg_buf, sizeof ppg_win);

    // Primer tiempo
    #if DEBUG
    unsigned long t0 = esp_timer_get_time();
    #endif
    // Features
    float feats_imu[FeaturesIMU];
    float feats_ppg[FeaturesPPG];
    computeMPUFeatures_fromCopies(feats_imu, ax_win, ay_win, az_win, gx_win, gy_win, gz_win);
    computePPGFeatures_fromCopies(feats_ppg, ppg_win, PPG_WIN);

    // Concatena y emite una sola fila (62 features)
    float feats55[FeaturesIMU + FeaturesPPG];
    memcpy(feats55,      feats_imu, sizeof(feats_imu));
    memcpy(feats55 + 48, feats_ppg, sizeof(feats_ppg));
    float spec7[FeaturesPerChannel];
    
    computeSpectralPlus7(ppg_win, PPG_WIN,
                         ax_win, ay_win, az_win, IMU_WIN,
                         gx_win, gy_win, gz_win,
                         spec7);

    float feats62[TotalFeatures];
    memcpy(feats62, feats55, sizeof(feats55));
    memcpy(feats62 + 55, spec7, sizeof(spec7));

    // Segundo tiempo
    #if DEBUG
    unsigned long t1 = esp_timer_get_time();
    #endif

    // Emisión
    printFeaturesCSV(feats62, 62);

    // Tercer tiempo
    #if DEBUG
    unsigned long t2 = esp_timer_get_time();
      Serial.printf("calc=%.1f ms, print=%.1f ms\n",
                (t1-t0)/1000.0, (t2-t1)/1000.0);
      mark_emit();
    #endif

    // Actualiza índices y totales (1 hop exacto por flujo)
    last_emit_mpu += (unsigned long)IMU_HOP;
    last_emit_ppg += (unsigned long)PPG_HOP;
  }
}

// Setup inicial
void setup() {
  Serial.setTxBufferSize(4096);   // <-- mover antes
  Serial.begin(BAUDIOS); 
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN); // inicia I2C
  Wire.setClock(400000); // 400 kHz

   // Debug: Iniciando inicialización
  #if DEBUG
    Serial.println("Iniciando inicialización de sensores...");
  #endif

  // Inicializa MPU6500
  while (!mpu.begin(Wire, I2C_ADDR_MPU)) {
    #if DEBUG
      Serial.println("MPU6500 no detectado. Reintentando en 1 segundo...");
    #endif
    delay(1000);
  }
  #if DEBUG
    Serial.println("MPU6500 inicializado correctamente.");
  #endif

  // Inicializa MAX30102
  while(!ppg.begin(Wire, 400000, I2C_ADDR_MAX)){
    #if DEBUG
      Serial.println("MAX30102 no detectado. Reintentando en 1 segundo...");
    #endif
    delay(1000);
  }
  #if DEBUG
    Serial.println("MAX30102 inicializado correctamente.");
  #endif

  // RANGOS y FILTRO para 200 Hz
  // MPU6500
  mpu.setAccelRange(MPU6500::ACCEL_4G);
  mpu.setGyroRange(MPU6500::GYRO_500DPS);
  mpu.setDlpf(3);               
  mpu.setSampleRateDivider(4);  
  mpu.calibrate(400);

  // MAX30102
  ppg.setup(0x3F, /*avg=*/4, /*ledMode=*/2, /*rate=*/FS_PPG_HZ, /*pw=*/411, /*adc=*/16384);
  ppg.setPulseAmplitudeIR(0x3F);
  ppg.setPulseAmplitudeRed(0x3F);
  ppg.setPulseAmplitudeGreen(0x00);
  ppg.setFIFOAverage(4);
  ppg.enableFIFORollover();
  ppg.clearFIFO();   // arranca FIFO limpio

  // Timers
  esp_timer_create_args_t targs_imu = {};
  targs_imu.callback = &onIMUTick_task;
  targs_imu.arg = nullptr;
  targs_imu.dispatch_method = ESP_TIMER_TASK;
  targs_imu.name = "imu200hz";
  ESP_ERROR_CHECK(esp_timer_create(&targs_imu, &imu_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(imu_timer, 1000000ULL / FS_IMU_HZ)); // 200 Hz

  esp_timer_create_args_t targs_ppg = {};
  targs_ppg.callback = &onPPGTick_task;
  targs_ppg.arg = nullptr;
  targs_ppg.name = "ppg_fs";
  ESP_ERROR_CHECK(esp_timer_create(&targs_ppg, &ppg_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(ppg_timer, 1000000ULL / FS_PPG_HZ));

  // Handshake con Python (opcional)
  #if USE_PYTHON
  bool ready = false;
  while(!ready){
    if(Serial.available()){
      String s = Serial.readStringUntil('\n');
      s.trim();
      if(s=="READY") ready=true;
    }
  }
  Serial.println("READY");
  #endif
  // Precomputar ventanas Hann
  precomputeHann(hann_ppg, N_PPG);
  precomputeHann(hann_imu, N_IMU);
}

// Loop principal
void loop() {
  // Manejo de flags y muestreo
  while (imu_ready > 0){
    portENTER_CRITICAL(&timerMux);
    imu_ready--;
    portEXIT_CRITICAL(&timerMux);
    imuSample();
    #if DEBUG
      imu_loop_consumed++;
    #endif
  }
  while (ppg_ready > 0){
    portENTER_CRITICAL(&timerMux);
    ppg_ready--;
    portEXIT_CRITICAL(&timerMux);
    ppgSample();
    #if DEBUG
      ppg_loop_consumed++;
    #endif
  }

  maybeEmitOnce();

  // ====== Panel de salud 1 Hz (no intrusivo) ======
  #if DEBUG
    if (every_ms(1000)) {
      uint32_t imu_isr = imu_isr_ticks; uint32_t ppg_isr = ppg_isr_ticks;
      static uint32_t last_imu_isr=0, last_ppg_isr=0, last_imu_cons=0, last_ppg_cons=0;
      uint32_t imu_isr_rate = imu_isr - last_imu_isr; uint32_t ppg_isr_rate = ppg_isr - last_ppg_isr;
      uint32_t imu_cons_rate = imu_loop_consumed - last_imu_cons; uint32_t ppg_cons_rate = ppg_loop_consumed - last_ppg_cons;
      last_imu_isr = imu_isr; last_ppg_isr = ppg_isr; last_imu_cons = imu_loop_consumed; last_ppg_cons = ppg_loop_consumed;

      Serial.printf("IMU: ISR=%lu/s CONSUMED=%lu/s backlog=%ld\n", (unsigned long)imu_isr_rate, (unsigned long)imu_cons_rate, (long)imu_isr_rate-(long)imu_cons_rate);
      Serial.printf("PPG: ISR=%lu/s CONSUMED=%lu/s backlog=%ld\n", (unsigned long)ppg_isr_rate, (unsigned long)ppg_cons_rate, (long)ppg_isr_rate-(long)ppg_cons_rate);

      Serial.printf("WM: imu_widx_max=%d/%d  ppg_widx_max=%d/%d\n", imu_widx_max, IMU_WIN, ppg_widx_max, PPG_WIN);
      TS_IMU.dump("IMU");
      TS_PPG.dump("PPG");
      Serial.printf("Free heap: %u bytes\n", (unsigned)esp_get_free_heap_size());
      // reset watermarks cada segundo (opcional)
      imu_widx_max = 0; ppg_widx_max = 0;
    }
  #endif
}
