#include <Arduino.h>
#include <Wire.h>
#include <MPU6500.h>
#include <math.h>     // sqrtf
#include <string.h>   // memcpy, memmove
#include "esp_timer.h"

// ====== PPG ======
#include "MAX30105.h"   // SparkFun MAX3010x (MAX30102/MAX30105)
MAX30105 ppg;

// ====== CONFIG (IMU) ======
#define FS_IMU_HZ   200      // IMU 200 Hz
#define WINDOW_SEC  2.0f     // 2 s
#define OVERLAP     0.5f     // 50%
#define USE_G_IN_MPS2 1
#define SDA_PIN     21
#define SCL_PIN     22
#define I2C_ADDR    0x68     // change to 0x69 if AD0 is HIGH

// ====== CONFIG (PPG) ======
#define FS_PPG_HZ   100      // PPG 100 Hz (ajústalo a tu setup del MAX3010x)
#define USE_IR      1        // 1: usar canal IR para features (puedes cambiar a RED)

// ====== OBJETOS ======
MPU6500 mpu;

// ====== DERIVED SIZES ======
constexpr int IMU_WIN = int(WINDOW_SEC * FS_IMU_HZ);       // 2s -> 400
constexpr int IMU_HOP = int(IMU_WIN * (1.0f - OVERLAP));   // 200

constexpr int PPG_WIN = int(WINDOW_SEC * FS_PPG_HZ);       // 2s -> 200
constexpr int PPG_HOP = int(PPG_WIN * (1.0f - OVERLAP));   // 100

// ====== BUFFERS IMU ======
static float ax_win[IMU_WIN], ay_win[IMU_WIN], az_win[IMU_WIN];
static float gx_win[IMU_WIN], gy_win[IMU_WIN], gz_win[IMU_WIN];
static float ax_buf[IMU_WIN], ay_buf[IMU_WIN], az_buf[IMU_WIN];
static float gx_buf[IMU_WIN], gy_buf[IMU_WIN], gz_buf[IMU_WIN];

volatile int imu_widx = 0;   // write index IMU
volatile int imu_ready = 0;  // flags desde ISR
volatile unsigned long imu_total = 0;

// ====== BUFFERS PPG ======
static float ppg_buf[PPG_WIN];    // guardamos como float (convertido desde int)
static float ppg_win[PPG_WIN];

volatile int ppg_widx = 0;
volatile int ppg_ready = 0;
volatile unsigned long ppg_total = 0;

// ====== TIMERS ======
esp_timer_handle_t imu_timer;
esp_timer_handle_t ppg_timer;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ====== UTILS (stats) ======
inline float rms(const float* x, int n) {
  double s = 0.0;
  for (int i=0;i<n;++i) s += (double)x[i]*x[i];
  return sqrtf((float)(s/n) + 1e-12f);
}
inline float mean(const float* x, int n) {
  double s=0.0; for (int i=0;i<n;++i) s += x[i]; return (float)(s/n);
}
inline float var_pop(const float* x, int n, float mu=-9999.0f) {
  double s=0.0; float m = (mu==-9999.0f? mean(x,n): mu);
  for (int i=0;i<n;++i){ double d = x[i]-m; s += d*d; }
  return (float)(s/n);
}
inline float energy_mean(const float* x, int n) {
  // energía media (= RMS^2). Si prefieres energía acumulada, multiplica por n.
  double s=0.0; for (int i=0;i<n;++i) s += (double)x[i]*x[i];
  return (float)(s/n);
}
inline float ptp(const float* x, int n) {
  float mn=x[0], mx=x[0];
  for (int i=1;i<n;++i){ if(x[i]<mn) mn=x[i]; if(x[i]>mx) mx=x[i]; }
  return mx - mn;
}
inline float skewness(const float* x, int n) {
  if (n<3) return 0.0f;
  float m = mean(x,n);
  double m2 = 0.0, m3 = 0.0;
  for (int i=0;i<n;++i){ double d=x[i]-m; double d2=d*d; m2 += d2; m3 += d2*d; }
  m2 /= n; m3 /= n;
  double sd = sqrt(m2 + 1e-18);
  return (float)( m3 / (sd*sd*sd + 1e-18) );
}
inline float kurtosis_excess(const float* x, int n) {
  if (n<4) return 0.0f;
  float m = mean(x,n);
  double m2=0.0, m4=0.0;
  for (int i=0;i<n;++i){ double d=x[i]-m; double d2=d*d; m2 += d2; m4 += d2*d2; }
  m2/=n; m4/=n;
  double k = m4 / ((m2*m2)+1e-18);
  return (float)(k - 3.0); // exceso
}
inline float xcorr0(const float* a, const float* b, int n){
  float ma = mean(a,n), mb = mean(b,n);
  double num=0.0, va=0.0, vb=0.0;
  for(int i=0;i<n;++i){
    double da=a[i]-ma, db=b[i]-mb;
    num += da*db; va += da*da; vb += db*db;
  }
  double den = sqrt((va+1e-18)*(vb+1e-18));
  return (float)(num/(den+1e-18));
}
inline void demean(float* x, int n){
  float m = mean(x,n);
  for(int i=0;i<n;++i) x[i]-=m;
}

// ====== READ SENSORS (timers ISR flags) ======
void onIMUTick_task(void* /*arg*/) {
  portENTER_CRITICAL(&timerMux);
  imu_ready++;
  portEXIT_CRITICAL(&timerMux);
}
void onPPGTick_task(void* /*arg*/) {
  portENTER_CRITICAL(&timerMux);
  ppg_ready++;
  portEXIT_CRITICAL(&timerMux);
}

// ====== IMU SAMPLE ======
void imuSample(){
  float ax, ay, az, gx, gy, gz;
  mpu.readAccelG(ax, ay, az);
  mpu.readGyroDps(gx, gy, gz);

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
}

// ====== PPG SAMPLE ======
void ppgSample(){
  // Leemos el FIFO del MAX3010x si hay datos
  // Usamos una lectura simple por tick para mantener 100 Hz
  uint32_t ir=0, red=0, green=0;
  bool valid = ppg.safeCheck(0); // procesa FIFO
  (void)valid;

  // Toma la muestra "más nueva" disponible (si no hay, reutiliza la última)
  if (ppg.available()) {
    ir   = ppg.getIR();
    red  = ppg.getRed();
    green= ppg.getGreen();
  } else {
    // Si no hay nueva, salimos temprano para no contaminar la ventana
    return;
  }

  // Convierte a float (puedes escalar para evitar overflow en stats)
  float sample = USE_IR ? (float)ir : (float)red;
  // Escalado opcional: sample *= 1e-6f; // descomenta si números muy grandes

  if (ppg_widx >= PPG_WIN){
    memmove(ppg_buf, ppg_buf+PPG_HOP, sizeof(float)*(PPG_WIN-PPG_HOP));
    ppg_widx = PPG_WIN-PPG_HOP;
  }
  ppg_buf[ppg_widx] = sample;
  ppg_widx++;
  ppg_total++;
}

// ====== FEATURES IMU ======
static inline void add_channel(const float* x, int n, int idxBase, float /*fsIMU*/, float* out){
  float mu = mean(x,n);
  out[idxBase+0] = rms(x,n);
  out[idxBase+1] = var_pop(x,n, mu);
  out[idxBase+2] = energy_mean(x,n);
  out[idxBase+3] = ptp(x,n);
  out[idxBase+4] = skewness(x,n);
  out[idxBase+5] = kurtosis_excess(x,n);
}

// Computa features desde copias locales (para evitar condición de carrera)
void computeFeatures_fromCopies(float* out42,
  const float* ax,const float* ay,const float* az,
  const float* gx,const float* gy,const float* gz)
{
  const float fsIMU = (float)FS_IMU_HZ;
  int idx = 0;

  add_channel(ax, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel(ay, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel(az, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel(gx, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel(gy, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel(gz, IMU_WIN, idx, fsIMU, out42); idx+=6;

  // xcorr (6)
  out42[idx++] = xcorr0(ax, ay, IMU_WIN);
  out42[idx++] = xcorr0(ax, az, IMU_WIN);
  out42[idx++] = xcorr0(ay, az, IMU_WIN);
  out42[idx++] = xcorr0(gx, gy, IMU_WIN);
  out42[idx++] = xcorr0(gx, gz, IMU_WIN);
  out42[idx++] = xcorr0(gy, gz, IMU_WIN);
}

// ====== FEATURES PPG (6) ======
struct PPGFeatures { float rms, var, energy, ptp, skew, kurt; };

void computePPGFeatures(const float* x, int n, PPGFeatures& out, bool remove_mean = true){
  if (n <= 1) { out = {0,0,0,0,0,0}; return; }

  // 1ª pasada: mean, energy, ptp
  float mn = x[0], mx = x[0];
  double sum = 0.0, sumsq = 0.0;
  for (int i=0;i<n;++i) {
    float xi = x[i];
    sum   += xi;
    sumsq += (double)xi * (double)xi;
    if (xi<mn) mn=xi; if (xi>mx) mx=xi;
  }
  float mu = (float)(sum / (double)n);
  out.energy = (float)(sumsq / (double)n);        // energía media (=RMS^2)
  out.rms    = sqrtf(out.energy + 1e-12f);
  out.ptp    = mx - mn;

  // 2ª pasada: m2, m3, m4 (centrados)
  double m2=0.0, m3=0.0, m4=0.0;
  if (remove_mean){
    for (int i=0;i<n;++i){ double d=x[i]-mu; double d2=d*d; m2+=d2; m3+=d2*d; m4+=d2*d2; }
  } else {
    for (int i=0;i<n;++i){ double d=x[i]-mu; double d2=d*d; m2+=d2; m3+=d2*d; m4+=d2*d2; }
  }
  m2/=n; m3/=n; m4/=n;

  if (m2 <= 1e-18) {
    out.var  = 0.0f;
    out.skew = 0.0f;
    out.kurt = 0.0f;   // exceso = 0 (equivalente a 3 si fuese kurtosis no-exceso)
  } else {
    out.var  = (float)m2;
    double sd = sqrt(m2);
    out.skew = (float)( m3 / (sd*sd*sd + 1e-18) );
    double kurt = m4 / (m2*m2 + 1e-18); // kurtosis
    out.kurt = (float)(kurt - 3.0);     // exceso
  }
}

// ====== PRINT CSV ======
void printFeaturesCSV(const float* f, int n){
  for(int i=0;i<n;++i){
    Serial.print(f[i], 4);
    if (i<n-1) Serial.print(',');
  }
  Serial.println();
}
void printPPGHeaderCSV(){
  Serial.println("ppg_rms,ppg_var,ppg_energy,ppg_ptp,ppg_skew,ppg_kurt");
}
void printPPGFeaturesCSV(const PPGFeatures& f){
  Serial.print(f.rms,   6); Serial.print(',');
  Serial.print(f.var,   6); Serial.print(',');
  Serial.print(f.energy,6); Serial.print(',');
  Serial.print(f.ptp,   6); Serial.print(',');
  Serial.print(f.skew,  6); Serial.print(',');
  Serial.println(f.kurt,6);
}

// ====== SETUP ======
void setup() {
  Serial.begin(921600);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // IMU init
  while (!mpu.begin(Wire, I2C_ADDR)) {
    Serial.println("[BasicRead] MPU6500 no detectado. Reintentando en 1 segundo...");
    delay(1000);
  }
  mpu.setAccelRange(MPU6500::ACCEL_4G);
  mpu.setGyroRange(MPU6500::GYRO_500DPS);
  mpu.setDlpf(3);
  mpu.setSampleRateDivider(4); // ~1kHz/(1+4)=200 Hz
  mpu.calibrate(400);

  // PPG init (MAX3010x)
  if (!ppg.begin(Wire, I2C_ADDR)) {
    // muchos módulos MAX3010x usan 0x57; si fallara con 0x68 cambia esto:
    if (!ppg.begin(Wire, 0x57)) {
      Serial.println("[PPG] MAX3010x no detectado (prueba dirección 0x57).");
    }
  }
  // Configuración simple para IR a 100 Hz (ajusta según tu módulo)
  ppg.setup();                    // carga defaults SparkFun
  ppg.setPulseAmplitudeIR(0x2F);  // sube si señal baja, baja si satura
  ppg.setPulseAmplitudeRed(0x2F);
  ppg.setPulseAmplitudeGreen(0x00);
  ppg.setFIFOAverage(4);          // smoothing FIFO
  ppg.setSampleRate(FS_PPG_HZ);   // 100 Hz
  ppg.setPulseWidth(411);         // 18-bit (depende de la lib)
  ppg.setADCRange(16384);         // rango alto

  // Timers
  {
    esp_timer_create_args_t targs = {};
    targs.callback = &onIMUTick_task;
    targs.arg = nullptr;
    targs.dispatch_method = ESP_TIMER_TASK;
    targs.name = "imu200hz";
    ESP_ERROR_CHECK(esp_timer_create(&targs, &imu_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(imu_timer, 1000000ULL / FS_IMU_HZ));
  }
  {
    esp_timer_create_args_t targs = {};
    targs.callback = &onPPGTick_task;
    targs.arg = nullptr;
    targs.dispatch_method = ESP_TIMER_TASK;
    targs.name = "ppg100hz";
    ESP_ERROR_CHECK(esp_timer_create(&targs, &ppg_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(ppg_timer, 1000000ULL / FS_PPG_HZ));
  }

  // Handshake opcional como en tu código
  bool ready = false;
  while(!ready){
    if(Serial.available()){
      String s = Serial.readStringUntil('\n');
      s.trim();
      if(s=="READY") ready=true;
    }
  }
  Serial.println("READY");

  // Header CSV de PPG (solo una vez)
  printPPGHeaderCSV();
}

// ====== LOOP ======
void loop() {
  // ---- IMU ----
  while (imu_ready > 0){
    portENTER_CRITICAL(&timerMux);
    imu_ready--;
    portEXIT_CRITICAL(&timerMux);
    imuSample();
  }

  static unsigned long last_total = 0;
  while (imu_widx >= IMU_WIN && (imu_total - last_total) >= IMU_HOP) {
    last_total += IMU_HOP;                // avanza solo un hop lógico
    memcpy(ax_win, ax_buf, sizeof ax_win);
    memcpy(ay_win, ay_buf, sizeof ay_win);
    memcpy(az_win, az_buf, sizeof az_win);
    memcpy(gx_win, gx_buf, sizeof gx_win);
    memcpy(gy_win, gy_buf, sizeof gy_win);
    memcpy(gz_win, gz_buf, sizeof gz_win);

    float feats[42];
    computeFeatures_fromCopies(feats, ax_win, ay_win, az_win, gx_win, gy_win, gz_win);
    // Si quisieras imprimir también las IMU features, descomenta:
    // printFeaturesCSV(feats, 42);
  }

  // ---- PPG ----
  while (ppg_ready > 0){
    portENTER_CRITICAL(&timerMux);
    ppg_ready--;
    portEXIT_CRITICAL(&timerMux);
    ppgSample();
  }

  static unsigned long last_ppg_total = 0;
  while (ppg_widx >= PPG_WIN && (ppg_total - last_ppg_total) >= PPG_HOP) {
    last_ppg_total += PPG_HOP;

    memcpy(ppg_win, ppg_buf, sizeof ppg_win);
    // Opcional: quitar DC ANTES de features → mejora skew/kurt ante drift
    // demean(ppg_win, PPG_WIN);

    PPGFeatures pf;
    computePPGFeatures(ppg_win, PPG_WIN, pf, /*remove_mean=*/true);
    printPPGFeaturesCSV(pf);
  }
}
