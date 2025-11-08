#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>

// --- Pines del micrófono ---
#define I2S_WS 25
#define I2S_SD 22
#define I2S_SCK 26

#define I2S_PORT I2S_NUM_0
#define bufferLen 122  // Tamaño del buffer, no pasar de ~125 para evitar errores

int16_t sBuffer[bufferLen];
uint8_t macAddress[6] = {0x3C, 0x8A, 0x1F, 0x5D, 0x7F, 0x1C}; // MAC del receptor

// --- Prototipos ---
void onSendCb(const wifi_tx_info_t *info, esp_now_send_status_t status);
void i2s_install();
void i2s_setpin();

// --- Configuración del I2S ---
void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 10,
    .dma_buf_len = bufferLen,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

// --- Callback de envío (nueva firma) ---
void onSendCb(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    //Serial.println("Transmisión exitosa");
  } else {
    Serial.println("Fallo transmisión");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando emisor...");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al iniciar ESP-NOW");
    return;
  }

  // Información del receptor (peer)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al agregar peer");
    return;
  }

  // Registrar callback de envío
  esp_now_register_send_cb(onSendCb);

  // Inicializar I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
}

void loop() {
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, sizeof(sBuffer), &bytesIn, portMAX_DELAY);

  if (result == ESP_OK && bytesIn > 0) {
    // Enviar el audio por ESP-NOW
    esp_now_send(macAddress, (uint8_t *)&sBuffer, bytesIn);
  }
}
