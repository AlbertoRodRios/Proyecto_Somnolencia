#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "MAX30105.h"
// ====== PPG (SparkFun MAX3010x) ======
MAX30105 ppg;

// ====== CONFIG ======
#define FS_PPG_HZ     100       // Hz
#define WINDOW_SEC    2.0f      // s
#define OVERLAP       0.5f      // 50%
#define SDA_PIN       21
#define SCL_PIN       22
#define USE_IR        1         // 1=IR, 0=RED
   

// ====== DERIVED SIZES ======
constexpr int PPG_WIN = int(WINDOW_SEC * FS_PPG_HZ);        // 200
constexpr int PPG_HOP = int(PPG_WIN * (1.0f - OVERLAP));    // 100

// ====== BUFFERS ======
static float ppg_buf[PPG_WIN];
static float ppg_win[PPG_WIN];
volatile int ppg_widx = 0;
volatile int ppg_ready = 0;
volatile unsigned long ppg_total = 0;

// ====== TIMER ======
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
// ====== ISR FLAG ======
void onPPGTick_task(void* /*arg*/){
  portENTER_CRITICAL(&timerMux);
  ppg_ready++;
  portEXIT_CRITICAL(&timerMux);
}

// ====== READ SENSOR ======
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
void computePPGFeatures(float* out6, const float* x, int n){
  float mu = mean(x,n);
  out6[0] = rms(x,n);
  out6[1] = varpop(x,n, mu);
  out6[2] = energy(x,n);
  out6[3] = ptp(x,n);
  out6[4] = skewness(x,n);
  out6[5] = kurtosis_excess(x,n);
}

// ====== PRINT CSV ======
void printFeaturesCSV(const float* f, int n){
    for(int i=0;i<n;++i){
        Serial.print(f[i], 4);
        if (i<n-1) Serial.print(',');
    }
    Serial.println();
}

// ====== SETUP ======
void setup(){
  Serial.begin(115200);
  delay(300);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  while(!ppg.begin(Wire, 400000, 0x57)){
    Serial.println("[PPG] ERROR: no inicia MAX3010x; reintentando en 1 segundo...");
    delay(1000);
  }

  // setup(powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange)
  // powerLevel 0x2F–0x3F suelen ir bien; ajusta si satura o queda plano
  ppg.setup(0x3F, /*avg=*/4, /*ledMode=*/2, /*rate=*/FS_PPG_HZ, /*pw=*/411, /*adc=*/16384);
  ppg.setPulseAmplitudeIR(0x3F);
  ppg.setPulseAmplitudeRed(0x3F);
  ppg.setPulseAmplitudeGreen(0x00);
  ppg.setFIFOAverage(4);
  ppg.enableFIFORollover();
  ppg.clearFIFO();   // arranca FIFO limpio

  // Timer a FS_PPG_HZ
  esp_timer_create_args_t targs = {};
  targs.callback = &onPPGTick_task;
  targs.arg = nullptr;
  targs.name = "ppg_fs";
  ESP_ERROR_CHECK(esp_timer_create(&targs, &ppg_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(ppg_timer, 1000000ULL / FS_PPG_HZ));
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
void loop(){
  // Consume ticks del timer y drena FIFO
  while (ppg_ready > 0){
    portENTER_CRITICAL(&timerMux);
    ppg_ready--;
    portEXIT_CRITICAL(&timerMux);
    ppgSample();
  }

  // Ventana deslizante disparada por contador monótono (robusto ante atrasos)
  static unsigned long last_total = 0;
  if (ppg_widx >= PPG_WIN && (ppg_total - last_total) >= (unsigned long)PPG_HOP) {
    unsigned long hops = (ppg_total - last_total) / (unsigned long)PPG_HOP;
    last_total += hops * (unsigned long)PPG_HOP;

    memcpy(ppg_win, ppg_buf, sizeof ppg_win);
    float feats[6];
    computePPGFeatures(feats, ppg_win, PPG_WIN);
    printFeats(feats, 6);
  }
}
