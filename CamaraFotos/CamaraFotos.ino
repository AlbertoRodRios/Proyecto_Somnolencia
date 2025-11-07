#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#define CAMERA_MODEL_WROVER_KIT
#include "camera_pins.h"

void setup() {
  Serial.begin(115200);
  
  // Configurar cámara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_GRAYSCALE;
  if(psramFound()){
    config.frame_size = FRAMESIZE_240X240;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_240X240;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Inicializar cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error inicializando cámara: 0x%x", err);
    return;
  }
  
  Serial.println("Cámara inicializada correctamente");

  initSDCard();
}

void initSDCard() {
  Serial.println("Inicializando tarjeta SD...");
  
  // Inicializar SD_MMC en modo 1-bit (requerido para ESP32-CAM)
  if(!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Error al montar SD_MMC");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No se detectó tarjeta SD");
    return;
  }
  
  Serial.print("Tipo de tarjeta SD: ");
  if(cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if(cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("Desconocido");
  }
  
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("Tamaño de SD: %lluMB\n", cardSize);
  
  // Crear directorio de imágenes si no existe
  createDir(SD_MMC, "/images");
}

// 📁 CREAR DIRECTORIO
void createDir(fs::FS &fs, const char * path) {
  Serial.printf("Creando directorio: %s\n", path);
  if(fs.mkdir(path)) {
    Serial.println("Directorio creado");
  } else {
    Serial.println("Error creando directorio (puede que ya exista)");
  }
}

// 💾 GUARDAR IMAGEN EN SD
bool saveImageToSD(camera_fb_t * fb) {
  String path = "/images/image_" + String(millis()) + ".raw";
  
  // Serial.printf("Guardando imagen: %s\n", path.c_str());
  
  // File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  // if(!file) {
  //   Serial.println("Error abriendo archivo para escritura");
  //   return false;
  // }
  
  // // Escribir datos RAW de escala de grises
  // size_t written = file.write(fb->buf, fb->len);
  // file.close();
  
  // if(written != fb->len) {
  //   Serial.printf("Error: Escritos %zu de %zu bytes\n", written, fb->len);
  //   return false;
  // }
  
  // Serial.printf("✅ Imagen grayscale guardada: %s\n", path.c_str());
  
  // ✅ este si sirve
  saveAsPGM(fb);
  
  return true;
}

bool saveAsPGM(camera_fb_t * fb) {
  String path = "/images/image_" + String(millis()) + ".pgm";
  
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if(!file) {
    Serial.println("Error abriendo archivo PGM");
    return false;
  }
  
  // Cabecera PGM
  file.print("P5\n");
  file.print(fb->width);
  file.print(" ");
  file.print(fb->height);
  file.print("\n255\n");  // Máximo valor = 255 (8-bit)
  
  // Datos de la imagen
  file.write(fb->buf, fb->len);
  file.close();
  
  Serial.printf("✅ Imagen PGM guardada: %s\n", path.c_str());
  return true;
}

// 📊 VER ESPACIO DISPONIBLE EN SD
void checkSDCardSpace() {
  static unsigned long lastCheck = 0;
  
  // Verificar espacio cada 10 fotos aproximadamente
  if(millis() - lastCheck > 10000) {
    lastCheck = millis();
    
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    uint64_t usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
    uint64_t freeSpace = cardSize - usedSpace;
    
    Serial.printf("SD Card - Usado: %lluMB, Libre: %lluMB, Total: %lluMB\n", 
                  usedSpace, freeSpace, cardSize);
    
    // Advertencia si queda poco espacio
    if(freeSpace < 10) {
      Serial.println("⚠️ POCA CAPACIDAD EN SD!");
    }
  }
}

void loop() {
  // Capturar foto
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Error capturando foto");
    return;
  }
  
  Serial.printf("Foto capturada - Tamaño: %zu bytes\n", fb->len);
  // aqui se procesa la img 

   if(saveImageToSD(fb)) {
    Serial.println("✅ Imagen guardada exitosamente");
  } else {
    Serial.println("❌ Error guardando imagen");
  }
  
  checkSDCardSpace();

  esp_camera_fb_return(fb);
  fb = NULL;
  
  delay(1000); 
}