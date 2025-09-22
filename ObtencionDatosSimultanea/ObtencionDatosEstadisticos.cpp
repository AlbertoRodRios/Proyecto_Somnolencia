#include <Arduino.h>
#include <Wire.h>
#include <MPU6500.h>
#include "MAX30105.h"
#include <math.h>     // sqrtf
#include <string.h>   // memcpy, memmove
#include "esp_timer.h"

// ====== CONFIG ======
#define FS_IMU_HZ    200      // IMU 200 Hz
#define WINDOW_SEC   2.0f     // 2 s
#define OVERLAP      0.5f     // 50%
#define USE_G_IN_MPS2 1       // 1: convertir 'g' a m/s^2 (opcional)
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR_MPU 0x68 
#define I2C_ADDR_MAX 0x57
#define FS_PPG_HZ 100         // Hz
#define USE_IR 1 

// Objetos
MPU6500 mpu;
MAX30105 ppg;

// ====== DERIVED SIZES ======
constexpr int IMU_WIN = int(WINDOW_SEC * FS_IMU_HZ);      // 2s -> 400
constexpr int IMU_HOP = int(IMU_WIN * (1.0f - OVERLAP));  // 200
constexpr int PPG_WIN = int(WINDOW_SEC * FS_PPG_HZ);      // 200
constexpr int PPG_HOP = int(PPG_WIN * (1.0f - OVERLAP));  // 100

// ====== BUFFERS ======
static float ax_win[IMU_WIN], ay_win[IMU_WIN], az_win[IMU_WIN];
static float gx_win[IMU_WIN], gy_win[IMU_WIN], gz_win[IMU_WIN];
static float ax_buf[IMU_WIN], ay_buf[IMU_WIN], az_buf[IMU_WIN];
static float gx_buf[IMU_WIN], gy_buf[IMU_WIN], gz_buf[IMU_WIN];
static float ppg_buf[PPG_WIN];
static float ppg_win[PPG_WIN];

volatile int imu_widx = 0;   // write index IMU
volatile int imu_ready = 0;  // flags desde ISR
volatile unsigned long imu_total = 0;
volatile int ppg_widx = 0;
volatile int ppg_ready = 0;
volatile unsigned long ppg_total = 0;

static unsigned long last_total_mpu = 0;
static unsigned long last_total_ppg = 0;

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
// Varianza poblacional (la que ya usabas)
inline float var(const float* x, int n, float mu=-9999.0f) {
  double s=0.0; float m = (mu==-9999.0f? mean(x,n): mu);
  for (int i=0;i<n;++i){ double d = x[i]-m; s += d*d; }
  return (float)(s/n);
}
inline float energy(const float* x, int n) {
  double s=0.0; for (int i=0;i<n;++i) s += (double)x[i]*x[i];
  return (float)(s/n); // energía media (= RMS^2)
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

// ====== READ SENSORS ======
void onIMUTick_task(void* /*arg*/) {
  portENTER_CRITICAL(&timerMux);
  imu_ready++;
  portEXIT_CRITICAL(&timerMux);
}
void onPPGTick_task(void* /*arg*/){
  portENTER_CRITICAL(&timerMux);
  ppg_ready++;
  portEXIT_CRITICAL(&timerMux);
}

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
}

// ====== FEATURES ======
static inline void add_channel_mpu(const float* x, int n, int idxBase, float /*fsIMU*/, float* out){
  float mu = mean(x,n);
  out[idxBase+0] = rms(x,n);
  out[idxBase+1] = var(x,n, mu);
  out[idxBase+2] = energy(x,n);
  out[idxBase+3] = ptp(x,n);
  out[idxBase+4] = skewness(x,n);
  out[idxBase+5] = kurtosis_excess(x,n);
}

void computePPGFeatures_fromCopies(float* out6, const float* x, int n){
  float mu = mean(x,n);
  (void)mu; // no imprescindible, pero útil si quisieras var(x,n,mu)
  out6[0] = rms(x,n);
  out6[1] = var(x,n);         
  out6[2] = energy(x,n);
  out6[3] = ptp(x,n);
  out6[4] = skewness(x,n);
  out6[5] = kurtosis_excess(x,n);
}

// Computa features desde copias locales (para evitar condición de carrera)
void computeMPUFeatures_fromCopies(float* out42,
  const float* ax,const float* ay,const float* az,
  const float* gx,const float* gy,const float* gz)
{
  const float fsIMU = (float)FS_IMU_HZ;
  int idx = 0;

  add_channel_mpu(ax, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel_mpu(ay, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel_mpu(az, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel_mpu(gx, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel_mpu(gy, IMU_WIN, idx, fsIMU, out42); idx+=6;
  add_channel_mpu(gz, IMU_WIN, idx, fsIMU, out42); idx+=6;

  // xcorr (6)
  out42[idx++] = xcorr0(ax, ay, IMU_WIN);
  out42[idx++] = xcorr0(ax, az, IMU_WIN);
  out42[idx++] = xcorr0(ay, az, IMU_WIN);
  out42[idx++] = xcorr0(gx, gy, IMU_WIN);
  out42[idx++] = xcorr0(gx, gz, IMU_WIN);
  out42[idx++] = xcorr0(gy, gz, IMU_WIN);
}

// ====== PRINT CSV ======
void printFeaturesCSV(const float* f, int n){
  for(int i=0;i<n;++i){
    Serial.print(f[i], 4);
    if (i<n-1) Serial.print(',');
  }
  Serial.println();
}

void maybeEmitOnce() {
  // ¿cuántos hops pendientes tiene cada flujo?
  unsigned long imu_hops = (imu_total > last_total_mpu) 
                           ? (imu_total - last_total_mpu) / (unsigned long)IMU_HOP : 0UL;
  unsigned long ppg_hops = (ppg_total > last_total_ppg) 
                           ? (ppg_total - last_total_ppg) / (unsigned long)PPG_HOP : 0UL;

  // Emite SOLO si ambos tienen ≥1 hop pendiente
  if (imu_hops >= 1UL && ppg_hops >= 1UL) {
    // Avanza ambos contadores para descartar backlog completo (nos quedamos con la ventana más reciente)
    unsigned long min_hops = (imu_hops < ppg_hops) ? imu_hops : ppg_hops;
    last_total_mpu += min_hops * (unsigned long)IMU_HOP;
    last_total_ppg += min_hops * (unsigned long)PPG_HOP;

    // Copias locales de la última ventana COMPLETA
    memcpy(ax_win, ax_buf, sizeof ax_win);
    memcpy(ay_win, ay_buf, sizeof ay_win);
    memcpy(az_win, az_buf, sizeof az_win);
    memcpy(gx_win, gx_buf, sizeof gx_win);
    memcpy(gy_win, gy_buf, sizeof gy_win);
    memcpy(gz_win, gz_buf, sizeof gz_win);
    memcpy(ppg_win, ppg_buf, sizeof ppg_win);

    // Features
    float feats_imu[42];
    float feats_ppg[6];
    computeMPUFeatures_fromCopies(feats_imu, ax_win, ay_win, az_win, gx_win, gy_win, gz_win);
    computePPGFeatures_fromCopies(feats_ppg, ppg_win, PPG_WIN);

    // Concatena y emite una sola fila (48)
    float feats48[48];
    memcpy(feats48,      feats_imu, sizeof(feats_imu));
    memcpy(feats48 + 42, feats_ppg, sizeof(feats_ppg));
    printFeaturesCSV(feats48, 48);
  }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // CORRECCIÓN: usar I2C_ADDR_MPU
  while (!mpu.begin(Wire, I2C_ADDR_MPU)) {
    Serial.println("MPU6500 no detectado. Reintentando en 1 segundo...");
    delay(1000);
  }

  while(!ppg.begin(Wire, 400000, I2C_ADDR_MAX)){
    Serial.println("MAX30102 no detectado. Reintentando en 1 segundo...");
    delay(1000);
  }

  // RANGOS y FILTRO para 200 Hz
  mpu.setAccelRange(MPU6500::ACCEL_4G);
  mpu.setGyroRange(MPU6500::GYRO_500DPS);
  mpu.setDlpf(3);               
  mpu.setSampleRateDivider(4);  
  mpu.calibrate(400);

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

  // Handshake con PC para empezar
  bool ready = false;
  while(!ready){
    if(Serial.available()){
      String s = Serial.readStringUntil('\n');
      s.trim();
      if(s=="READY") ready=true;
    }
  }
  Serial.println("READY"); 
}

// ====== LOOP ======
void loop() {
  // --- drena flags y toma muestras ---
  while (imu_ready > 0){
    portENTER_CRITICAL(&timerMux);
    imu_ready--;
    portEXIT_CRITICAL(&timerMux);
    imuSample();
  }
  while (ppg_ready > 0){
    portENTER_CRITICAL(&timerMux);
    ppg_ready--;
    portEXIT_CRITICAL(&timerMux);
    ppgSample();
  }

    maybeEmitOnce();
}
