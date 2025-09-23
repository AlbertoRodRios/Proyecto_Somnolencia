// Freenove ESP32-WROVER CAM (OV3660) - CAPTURA BAJO DEMANDA (anti-canary)
// Recomendado usar ESP32 core 2.0.14 para probar estabilidad

#include <Arduino.h>
#include "esp_camera.h"
#include "esp32-hal-psram.h"
#include "esp_log.h"

#define BAUDRATE 460800  // Empieza bajo; luego podrás subir

// ---- PINOUT WROVER_KIT (Freenove) ----
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     21
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       19
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM        5
#define Y2_GPIO_NUM        4
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static inline uint16_t checksum16(const uint8_t* d, size_t n){
  uint32_t s=0; for(size_t i=0;i<n;++i) s+=d[i]; return (uint16_t)(s & 0xFFFF);
}

bool cam_init(){
  // Silencia logs del driver para ahorrar stack
  esp_log_level_set("*", ESP_LOG_ERROR);

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;  c.pin_pclk = PCLK_GPIO_NUM;  c.pin_vsync = VSYNC_GPIO_NUM;  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;  c.pin_reset = RESET_GPIO_NUM;

  // Ultra conservador:
  c.xclk_freq_hz = 8000000;            // 8 MHz
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_VGA;    // 160x120
  c.jpeg_quality = 12;                 // menos bytes
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_DRAM;  // evita PSRAM para fb
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s) { s->set_brightness(s,0); s->set_contrast(s,0); s->set_saturation(s,0); }
  return true;
}
void cam_deinit(){ esp_camera_deinit(); delay(20); }

void send_jpeg(const uint8_t* buf, size_t len){
  const uint32_t MAGIC=0xAA55AA55; const uint16_t sum=checksum16(buf,len);
  Serial.write((uint8_t*)&MAGIC,4);
  Serial.write((uint8_t*)&len,4);
  const size_t CH=512;
  for(size_t off=0; off<len; ){
    size_t n = (len-off>CH)?CH:(len-off);
    Serial.write(buf+off, n);
    off += n;
  }
  Serial.write((uint8_t*)&sum,2);
  Serial.flush();
}

void setup(){
  Serial.begin(BAUDRATE);
  delay(400);
  Serial.println("[READY]");
}

void loop(){
  if (Serial.available()){
    char ch=(char)Serial.read();
    if (ch=='c' || ch=='C'){
      if (!cam_init()){ Serial.println("[ERR] init"); return; }
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb && fb->format==PIXFORMAT_JPEG){
        send_jpeg(fb->buf, fb->len);
        esp_camera_fb_return(fb);
      } else if (fb) esp_camera_fb_return(fb);
      cam_deinit();
    }
  }
}
