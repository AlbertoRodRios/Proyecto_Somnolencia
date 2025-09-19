#include <Arduino.h>
#include <Wire.h>

// ====== CONFIG ======
#define FS_IMU_HZ       200      // IMU 200 Hz
#define FS_PPG_HZ       50       // PPG 50 Hz
#define WINDOW_SEC      2.0f     // 2 s
#define OVERLAP         0.5f     // 50%
#define USE_G_IN_MPS2   1        // 1: convertir 'g' a m/s^2 (opcional)
#define DISABLE_SPECTRAL 0       // 1: desactiva energía espectral (pone 0)
#define PPG_DEMEAN_AC    1       // 1: restar DC al PPG para features (AC)

// ====== SENSORS ======
#include "MPU6050.h"      // i2cdevlib MPU6050
MPU6050 mpu;
#include <SparkFun_MAX3010x.h>
MAX30105 ppg;

// ====== FFT (para espectral) ======
#if !DISABLE_SPECTRAL
#include <arduinoFFT.h>
#include <algorithm>
#endif

#ifndef MAX_FFT_N
#define MAX_FFT_N 512   // máx tamaño FFT permitido (2s@200Hz=400 → se rellena a 512)
#endif

// ====== DERIVED SIZES ======
const int IMU_WIN = int(WINDOW_SEC * FS_IMU_HZ);  // 2s -> 400
const int IMU_HOP = int(IMU_WIN * (1.0f - OVERLAP)); // 200
const int PPG_WIN = int(WINDOW_SEC * FS_PPG_HZ);  // 2s -> 100
const int PPG_HOP = int(PPG_WIN * (1.0f - OVERLAP)); // 50

// ====== BUFFERS ======
static float ax_buf[IMU_WIN], ay_buf[IMU_WIN], az_buf[IMU_WIN];
static float gx_buf[IMU_WIN], gy_buf[IMU_WIN], gz_buf[IMU_WIN];
static float ppg_buf[PPG_WIN];

volatile int imu_widx = 0;   // write index IMU
volatile int ppg_widx = 0;   // write index PPG
volatile int imu_ready = 0;  // flags desde ISR
volatile int ppg_ready = 0;

// ====== TIMERS ======
hw_timer_t* timerIMU = nullptr;
hw_timer_t* timerPPG = nullptr;
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
inline float var(const float* x, int n, float mu=-9999.0f) {
  double s=0.0; float m = (mu==-9999.0f? mean(x,n): mu);
  for (int i=0;i<n;++i){ double d = x[i]-m; s += d*d; }
  return (float)(s/n);
}
inline float energy(const float* x, int n) {
  double s=0.0; for (int i=0;i<n;++i) s += (double)x[i]*x[i];
  return (float)(s/n); // energía media
}
inline float ptp(const float* x, int n) {
  float mn=x[0], mx=x[0];
  for (int i=1;i<n;++i){ if(x[i]<mn) mn=x[i]; if(x[i]>mx) mx=x[i]; }
  return mx - mn;
}
inline float skewness(const float* x, int n) {
  if (n<3) return 0.0f;
  float m = mean(x,n);
  float s2 = 0.0f, s3 = 0.0f;
  for (int i=0;i<n;++i){ float d=x[i]-m; s2 += d*d; s3 += d*d*d; }
  s2 /= n; s3 /= n;
  float sd = sqrtf(s2 + 1e-12f);
  return (s3 / (sd*sd*sd + 1e-12f));
}
inline float kurtosis_excess(const float* x, int n) {
  if (n<4) return 0.0f;
  float m = mean(x,n);
  double s2=0.0, s4=0.0;
  for (int i=0;i<n;++i){ double d=x[i]-m; s2 += d*d; s4 += d*d*d*d; }
  s2/=n; s4/=n;
  double k = s4 / ((s2*s2)+1e-18);
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

#if !DISABLE_SPECTRAL
// energía espectral integrada hasta fmax usando arduinoFFT (periodograma con Hann)
float spectralE_upTo(const float* x, int n, float fs, float fmax) {
  if (n <= 1 || fs <= 0.0f || fmax <= 0.0f) return 0.0f;

  // Escoger N como potencia de 2 >= n, pero ≤ MAX_FFT_N
  int N = 1;
  while (N < n && (N << 1) <= MAX_FFT_N) N <<= 1;
  if (N < n) N = MAX_FFT_N;

  static double vReal[MAX_FFT_N];
  static double vImag[MAX_FFT_N];

  // Copiar datos y rellenar con ceros si hace falta
  int copyLen = std::min(n, N);
  for (int i = 0; i < copyLen; i++) {
    vReal[i] = (double)x[i];
    vImag[i] = 0.0;
  }
  for (int i = copyLen; i < N; i++) {
    vReal[i] = 0.0;
    vImag[i] = 0.0;
  }

  // Nota: según versión de la lib, puede ser 'arduinoFFT' o 'ArduinoFFT<double>'
  ArduinoFFT fft(vReal, vImag, N, fs);
  // ArduinoFFT<double> fft(vReal, vImag, N, fs); // ← usar esta si tu lib lo requiere

  fft.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
  fft.compute(FFT_FORWARD);
  fft.complexToMagnitude();

  // Integrar energía espectral hasta fmax
  double df = fs / (double)N;
  int kMax = std::min((int)(fmax / df), N/2);
  if (kMax < 1) return 0.0f;

  double E = 0.0;
  for (int k = 1; k <= kMax; k++) {  // saltamos DC
    double mag = vReal[k];
    double Pk = (mag * mag) / ((double)N * (double)N);  // densidad
    bool isNyquist = ((N % 2) == 0) && (k == N/2);
    if (!isNyquist) Pk *= 2.0;  // espectro de un solo lado
    E += Pk * df;
  }
  return (float)E;
}
#else
inline float spectralE_upTo(const float* x, int n, float fs, float fmax){
  (void)x; (void)n; (void)fs; (void)fmax;
  return 0.0f;
}
#endif

// ====== READ SENSORS ======
void IRAM_ATTR onIMUTick(){
  portENTER_CRITICAL_ISR(&timerMux);
  imu_ready++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR onPPGTick(){
  portENTER_CRITICAL_ISR(&timerMux);
  ppg_ready++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void imuSample(){
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  mpu.getMotion6(&axRaw,&ayRaw,&azRaw,&gxRaw,&gyRaw,&gzRaw);

  // Conversión simple: MPU6050 accel LSB->g (16384 LSB/g), gyro LSB->°/s (131 LSB/(°/s))
  float ax = (float)axRaw / 16384.0f;
  float ay = (float)ayRaw / 16384.0f;
  float az = (float)azRaw / 16384.0f;
  float gx = (float)gxRaw / 131.0f;   // °/s
  float gy = (float)gyRaw / 131.0f;
  float gz = (float)gzRaw / 131.0f;

#if USE_G_IN_MPS2
  const float G = 9.80665f;
  ax*=G; ay*=G; az*=G;
#endif
  // (Opcional: pasar gyro a rad/s si lo mezclas con accel)
  // const float DEG2RAD = 0.01745329252f;
  // gx*=DEG2RAD; gy*=DEG2RAD; gz*=DEG2RAD;

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
}

void ppgSample(){
  // MAX3010x: toma el canal de IR como PPG básico
  if (ppg.available()){
    long ir = ppg.getIR();
    ppg.nextSample();
    float v = (float)ir;

    if (ppg_widx >= PPG_WIN){
      memmove(ppg_buf, ppg_buf+PPG_HOP, sizeof(float)*(PPG_WIN-PPG_HOP));
      ppg_widx = PPG_WIN-PPG_HOP;
    }
    ppg_buf[ppg_widx] = v;
    ppg_widx++;
  }
}

// ====== FEATURES ======
static inline void add_channel(const float* x, int n, int idxBase, float fsIMU, float* out){
  float mu = mean(x,n);
  out[idxBase+0] = rms(x,n);
  out[idxBase+1] = var(x,n, mu);
  out[idxBase+2] = energy(x,n);
  out[idxBase+3] = ptp(x,n);
  out[idxBase+4] = skewness(x,n);
  out[idxBase+5] = kurtosis_excess(x,n);
  out[idxBase+6] = spectralE_upTo(x,n,fsIMU,10.0f);
}

// Computa features desde copias locales (para evitar condición de carrera)
void computeFeatures_fromCopies(float* out54,
  const float* ax,const float* ay,const float* az,
  const float* gx,const float* gy,const float* gz,
  const float* ppgLocal)
{
  const float fsIMU = (float)FS_IMU_HZ;
  int idx = 0;

  add_channel(ax, IMU_WIN, idx, fsIMU, out54); idx+=7;
  add_channel(ay, IMU_WIN, idx, fsIMU, out54); idx+=7;
  add_channel(az, IMU_WIN, idx, fsIMU, out54); idx+=7;
  add_channel(gx, IMU_WIN, idx, fsIMU, out54); idx+=7;
  add_channel(gy, IMU_WIN, idx, fsIMU, out54); idx+=7;
  add_channel(gz, IMU_WIN, idx, fsIMU, out54); idx+=7;

  // xcorr (6)
  out54[idx++] = xcorr0(ax, ay, IMU_WIN);
  out54[idx++] = xcorr0(ax, az, IMU_WIN);
  out54[idx++] = xcorr0(ay, az, IMU_WIN);
  out54[idx++] = xcorr0(gx, gy, IMU_WIN);
  out54[idx++] = xcorr0(gx, gz, IMU_WIN);
  out54[idx++] = xcorr0(gy, gz, IMU_WIN);

  // PPG (6) — opcionalmente sin DC (AC puro)
  float ppgTmp[PPG_WIN];
  memcpy(ppgTmp, ppgLocal, sizeof(ppgTmp));
#if PPG_DEMEAN_AC
  demean(ppgTmp, PPG_WIN);
  const float* p = ppgTmp;
#else
  const float* p = ppgLocal;
#endif
  float mu = mean(p, PPG_WIN);
  out54[idx++] = rms(p, PPG_WIN);
  out54[idx++] = var(p, PPG_WIN, mu);
  out54[idx++] = energy(p, PPG_WIN);
  out54[idx++] = ptp(p, PPG_WIN);
  out54[idx++] = skewness(p, PPG_WIN);
  out54[idx++] = kurtosis_excess(p, PPG_WIN);
}

// ====== PRINT CSV ======
void printFeaturesCSV(const float* f, int n){
  static bool printedHeader=false;
  if(!printedHeader){
    Serial.println(
      "ax_rms,ax_var,ax_energy,ax_ptp,ax_skew,ax_kurt,ax_specE_10Hz,"
      "ay_rms,ay_var,ay_energy,ay_ptp,ay_skew,ay_kurt,ay_specE_10Hz,"
      "az_rms,az_var,az_energy,az_ptp,az_skew,az_kurt,az_specE_10Hz,"
      "gx_rms,gx_var,gx_energy,gx_ptp,gx_skew,gx_kurt,gx_specE_10Hz,"
      "gy_rms,gy_var,gy_energy,gy_ptp,gy_skew,gy_kurt,gy_specE_10Hz,"
      "gz_rms,gz_var,gz_energy,gz_ptp,gz_skew,gz_kurt,gz_specE_10Hz,"
      "xcorr_ax_ay,xcorr_ax_az,xcorr_ay_az,xcorr_gx_gy,xcorr_gx_gz,xcorr_gy_gz,"
      "ppg_rms,ppg_var,ppg_energy,ppg_ptp,ppg_skew,ppg_kurt"
    );
    printedHeader = true;
  }
  for(int i=0;i<n;++i){
    Serial.print(f[i], 6);
    if (i<n-1) Serial.print(',');
  }
  Serial.println();
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin();
  Wire.setClock(400000);

  // IMU
  mpu.initialize();
  if (!mpu.testConnection()){
    Serial.println("MPU6050 no conectado!");
  } else {
    // RANGOS y FILTRO para 200 Hz
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);   // ±4g (ajusta a tu caso)
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500 °/s
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);              // ~42 Hz (bueno para 200 Hz)
  }

  // PPG (SparkFun MAX30105)
  if (!ppg.begin(Wire, I2C_SPEED_FAST)){
    Serial.println("MAX3010x no conectado!");
  } else {
    // ledBrightness=0x1F, sampleAverage=4, ledMode=2 (IR only),
    // sampleRate=50 (tu define), pulseWidth=2, adcRange=3 (16384 nA)
    ppg.setup(0x1F, 4, 2, FS_PPG_HZ, 2, 3);
    ppg.setPulseAmplitudeRed(0x00);   // 0 si usas IR-only
    ppg.setPulseAmplitudeIR(0x2F);    // Ajusta según montaje/ruido
    ppg.setPulseAmplitudeGreen(0x00); // 0 si no se usa
  }

  // Timers (disparan flags a frecuencias deseadas)
  timerIMU = timerBegin(0, 80, true); // 80MHz/80 = 1MHz tick
  timerAttachInterrupt(timerIMU, &onIMUTick, true);
  timerAlarmWrite(timerIMU, 1000000UL / FS_IMU_HZ, true);
  timerAlarmEnable(timerIMU);

  timerPPG = timerBegin(1, 80, true);
  timerAttachInterrupt(timerPPG, &onPPGTick, true);
  timerAlarmWrite(timerPPG, 1000000UL / FS_PPG_HZ, true);
  timerAlarmEnable(timerPPG);

  Serial.println("Listo. Recolectando...");
}

// ====== LOOP ======
void loop() {
  // muestreo fino por flags (evita delay() para timing estable)
  if (imu_ready > 0){
    portENTER_CRITICAL(&timerMux);
    imu_ready--;
    portEXIT_CRITICAL(&timerMux);
    imuSample();
  }
  if (ppg_ready > 0){
    portENTER_CRITICAL(&timerMux);
    ppg_ready--;
    portEXIT_CRITICAL(&timerMux);
    ppgSample();
  }

  // cuando ambas ventanas están llenas y hay hop nuevo en IMU, calculamos features
  static int last_imu_widx = 0;
  if (imu_widx >= IMU_WIN && ppg_widx >= PPG_WIN){
    if (imu_widx - last_imu_widx >= IMU_HOP){
      last_imu_widx = imu_widx;

      // Copias locales para evitar condiciones de carrera
      float ax[IMU_WIN], ay[IMU_WIN], az[IMU_WIN];
      float gx[IMU_WIN], gy[IMU_WIN], gz[IMU_WIN];
      float ppgLocal[PPG_WIN];

      noInterrupts();
      memcpy(ax, ax_buf, sizeof(ax));
      memcpy(ay, ay_buf, sizeof(ay));
      memcpy(az, az_buf, sizeof(az));
      memcpy(gx, gx_buf, sizeof(gx));
      memcpy(gy, gy_buf, sizeof(gy));
      memcpy(gz, gz_buf, sizeof(gz));
      memcpy(ppgLocal, ppg_buf, sizeof(ppgLocal));
      interrupts();

      float feats[54];
      computeFeatures_fromCopies(feats, ax,ay,az, gx,gy,gz, ppgLocal);
      printFeaturesCSV(feats, 54);
    }
  }
}
