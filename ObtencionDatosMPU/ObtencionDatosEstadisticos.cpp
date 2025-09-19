#include <Arduino.h>
#include <Wire.h>
#include <MPU6500.h>
#include <math.h>     // sqrtf
#include <string.h>   // memcpy, memmove
#include "esp_timer.h"

// ====== CONFIG ======
#define FS_IMU_HZ 200      // IMU 200 Hz
#define WINDOW_SEC 2.0f     // 2 s
#define OVERLAP 0.5f     // 50%
#define USE_G_IN_MPS2 1        // 1: convertir 'g' a m/s^2 (opcional)
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x68 // change to 0x69 if AD0 is HIGH

// Objeto mpu
MPU6500 mpu;

// ====== DERIVED SIZES ======
constexpr int IMU_WIN = int(WINDOW_SEC * FS_IMU_HZ);      // 2s -> 400
constexpr int IMU_HOP = int(IMU_WIN * (1.0f - OVERLAP));  // 200

// ====== BUFFERS ======
static float ax_win[IMU_WIN], ay_win[IMU_WIN], az_win[IMU_WIN];
static float gx_win[IMU_WIN], gy_win[IMU_WIN], gz_win[IMU_WIN];
static float ax_buf[IMU_WIN], ay_buf[IMU_WIN], az_buf[IMU_WIN];
static float gx_buf[IMU_WIN], gy_buf[IMU_WIN], gz_buf[IMU_WIN];

volatile int imu_widx = 0;   // write index IMU
volatile int imu_ready = 0;  // flags desde ISR
volatile unsigned long imu_total = 0;

// ====== TIMERS ======
esp_timer_handle_t imu_timer;
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

inline void demean(float* x, int n){
    float m = mean(x,n);
    for(int i=0;i<n;++i) x[i]-=m;
}

// ====== READ SENSORS ======
void onIMUTick_task(void* /*arg*/) {
    portENTER_CRITICAL(&timerMux);
    imu_ready++;
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

// ====== FEATURES ======
static inline void add_channel(const float* x, int n, int idxBase, float /*fsIMU*/, float* out){
    float mu = mean(x,n);
    out[idxBase+0] = rms(x,n);
    out[idxBase+1] = var(x,n, mu);
    out[idxBase+2] = energy(x,n);
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

// ====== PRINT CSV ======
void printFeaturesCSV(const float* f, int n){
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
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    while (!mpu.begin(Wire, I2C_ADDR)) {
        Serial.println("[BasicRead] MPU6500 no detectado. Reintentando en 1 segundo...");
        delay(1000);
    }
  
    // RANGOS y FILTRO para 200 Hz
    // RANGOS y FILTRO para 200 Hz
    mpu.setAccelRange(MPU6500::ACCEL_4G);
    mpu.setGyroRange(MPU6500::GYRO_500DPS);
    mpu.setDlpf(3);
    mpu.setSampleRateDivider(4);
    mpu.calibrate(400);

    esp_timer_create_args_t targs = {};        // <-- C++ seguro
    targs.callback = &onIMUTick_task;
    targs.arg = nullptr;
    targs.dispatch_method = ESP_TIMER_TASK;
    targs.name = "imu200hz";
    ESP_ERROR_CHECK(esp_timer_create(&targs, &imu_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(imu_timer, 1000000ULL / FS_IMU_HZ)); // 200 Hz

    Serial.println("READY"); 
}

// ====== LOOP ======
void loop() {
    if (imu_ready > 0){
        portENTER_CRITICAL(&timerMux);
        imu_ready--;
        portEXIT_CRITICAL(&timerMux);
        imuSample();
    }

    static unsigned long last_total = 0;
    if (imu_widx >= IMU_WIN) {
        if (imu_total - last_total >= IMU_HOP) {   // <-- usa el contador monótono
        last_total = imu_total;
        // copias y features (igual que lo tienes)
        memcpy(ax_win, ax_buf, sizeof(ax_win));
        memcpy(ay_win, ay_buf, sizeof(ay_win));
        memcpy(az_win, az_buf, sizeof(az_win));
        memcpy(gx_win, gx_buf, sizeof(gx_win));
        memcpy(gy_win, gy_buf, sizeof(gy_win));
        memcpy(gz_win, gz_buf, sizeof(gz_win));

        float feats[42];
        computeFeatures_fromCopies(feats, ax_win, ay_win, az_win, gx_win, gy_win, gz_win);
        printFeaturesCSV(feats, 42);
        }
    } 
}
