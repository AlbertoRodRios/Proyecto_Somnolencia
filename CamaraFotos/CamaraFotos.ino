#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#define CAMERA_MODEL_WROVER_KIT
#include "camera_pins.h"

int EYES_Y = 60;

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

camera_fb_t* cropLeftEye(camera_fb_t* original_fb) {
  // Tamaño fijo para el recorte del ojo
  int eye_height = 100;  // Alto del recorte del ojo}
  int eye_width = 240;

  // Centrar el recorte en el ojo izquierdo
  int start_x = 0;
  int start_y = EYES_Y;

  start_y = min(start_y, (int)(original_fb->height - eye_height));

  Serial.printf("Recortando ojo izquierdo: (%d, %d) tamaño %dx%d\n", 
                start_x, start_y, eye_width, eye_height);

  // Crear nuevo buffer para la imagen recortada
  size_t crop_size = eye_width * eye_height;
  uint8_t* crop_buf = (uint8_t*)ps_malloc(crop_size);
  
  if (crop_buf == NULL) {
    Serial.println("Error asignando memoria para recorte");
    return NULL;
  }

  // Copiar región recortada
  for (int y = 0; y < eye_height; y++) {
    for (int x = 0; x < eye_width; x++) {
      int orig_index = (start_y + y) * original_fb->width + (start_x + x);
      int crop_index = y * eye_width + x;
      crop_buf[crop_index] = original_fb->buf[orig_index];
    }
  }

  // Crear nueva estructura camera_fb_t para la imagen recortada
  camera_fb_t* cropped_fb = (camera_fb_t*)ps_malloc(sizeof(camera_fb_t));
  if (cropped_fb == NULL) {
    free(crop_buf);
    Serial.println("Error asignando memoria para cropped_fb");
    return NULL;
  }

  cropped_fb->width = eye_width;
  cropped_fb->height = eye_height;
  cropped_fb->buf = crop_buf;
  cropped_fb->len = crop_size;
  cropped_fb->format = original_fb->format;
  
  return cropped_fb;
}

// 💾 GUARDAR IMAGEN EN SD
bool saveImageToSD(camera_fb_t * fb) {
  // Primero recortar la imagen
  camera_fb_t* cropped_fb = cropLeftEye(fb);
  
  if (cropped_fb == NULL) {
    Serial.println("No se pudo recortar, guardando imagen completa");
    return saveAsPGM(fb); // Guardar original si no se puede recortar
  }

  String path = "/images/cropped_" + String(millis()) + ".pgm";
  
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if(!file) {
    Serial.println("Error abriendo archivo PGM");
    free(cropped_fb->buf);
    free(cropped_fb);
    return false;
  }
  
  // Cabecera PGM para imagen recortada
  file.print("P5\n");
  file.print(cropped_fb->width);
  file.print(" ");
  file.print(cropped_fb->height);
  file.print("\n255\n");
  
  // Datos de la imagen recortada
  file.write(cropped_fb->buf, cropped_fb->len);
  file.close();
  
  Serial.printf("✅ Imagen RECORTADA guardada: %s (%dx%d)\n", 
                path.c_str(), cropped_fb->width, cropped_fb->height);

  // Liberar memoria de la imagen recortada
  free(cropped_fb->buf);
  free(cropped_fb);
  
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
  
}