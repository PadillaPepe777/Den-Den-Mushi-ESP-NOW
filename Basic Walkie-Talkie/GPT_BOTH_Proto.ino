#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>

// ---------- CONFIGURACIÓN DE HARDWARE ----------
// Micrófono INMP441
#define I2S_MIC_PORT I2S_NUM_0
#define MIC_BCK 26 //SCK
#define MIC_WS  25
#define MIC_SD  22

// Bocina PCM5102A
#define I2S_SPK_PORT I2S_NUM_1
#define SPK_BCK 12
#define SPK_WS  27 //LCK
#define SPK_DIN 14

// Botón PTT
#define PTT_BUTTON 13

// Comunicación ESP-NOW
uint8_t peerAddress[6] = {0x3C, 0x8A, 0x1F, 0x5D, 0x7F, 0x1C}; // MAC del otro ESP
//2.- {0x3C, 0x8A, 0x1F, 0x5D, 0x7F, 0x1C}
//1.- {0xB4, 0xE6, 0x2D, 0xD5, 0x6C, 0x85}

// ---------- VARIABLES ----------
#define BUFFER_LEN 120
int16_t audioBuffer[BUFFER_LEN];
size_t bytesCount;
bool isTransmitting = false;

// ---------- CONFIGURACIONES I2S ----------
void i2s_setup_mic() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_MIC_PORT, &cfg, 0, NULL);

  i2s_pin_config_t pin_cfg = {
    .bck_io_num = MIC_BCK,
    .ws_io_num = MIC_WS,
    .data_out_num = -1,
    .data_in_num = MIC_SD
  };
  i2s_set_pin(I2S_MIC_PORT, &pin_cfg);
  i2s_set_clk(I2S_MIC_PORT, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void i2s_setup_speaker() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_SPK_PORT, &cfg, 0, NULL);

  i2s_pin_config_t pin_cfg = {
    .bck_io_num = SPK_BCK,
    .ws_io_num = SPK_WS,
    .data_out_num = SPK_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_set_pin(I2S_SPK_PORT, &pin_cfg);
  i2s_set_clk(I2S_SPK_PORT, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

// ---------- CALLBACK DE RECEPCIÓN ----------
float iir_state = 0.0f;
const float iir_alpha = 0.08f;

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (isTransmitting) return; // Ignora si estás transmitiendo

  size_t toCopy = min((size_t)len, sizeof(audioBuffer));
  memcpy(audioBuffer, data, toCopy);
  size_t samples = toCopy / sizeof(int16_t);

  // Filtro IIR para suavizar ruido
  for (size_t i = 0; i < samples; i++) {
    float s = (float)audioBuffer[i];
    iir_state = iir_alpha * s + (1.0f - iir_alpha) * iir_state;
    audioBuffer[i] = (int16_t)iir_state;
  }

  size_t written;
  i2s_write(I2S_SPK_PORT, audioBuffer, samples * sizeof(int16_t), &written, portMAX_DELAY);
}

// ---------- CALLBACK DE ENVÍO ----------
void onSendCb(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("⚠️ Fallo en transmisión");
  }
}

// ---------- CONFIGURACIÓN ----------
void setup() {
  Serial.begin(115200);
  pinMode(PTT_BUTTON, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    while (true);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onSendCb);

  // Inicialmente en modo escucha
  i2s_setup_speaker();
  Serial.println("Listo: modo escucha");
}

// ---------- LOOP PRINCIPAL ----------
void loop() {
  bool pressed = (digitalRead(PTT_BUTTON) == LOW);

  if (pressed && !isTransmitting) {
    Serial.println("🎙️ Transmitiendo...");
    isTransmitting = true;
    i2s_driver_uninstall(I2S_SPK_PORT);
    i2s_setup_mic();
  }

  if (!pressed && isTransmitting) {
    Serial.println("🔈 Escuchando...");
    isTransmitting = false;
    i2s_driver_uninstall(I2S_MIC_PORT);
    i2s_setup_speaker();
  }

  if (isTransmitting) {
    size_t bytesIn = 0;
    esp_err_t result = i2s_read(I2S_MIC_PORT, &audioBuffer, sizeof(audioBuffer), &bytesIn, portMAX_DELAY);
    if (result == ESP_OK && bytesIn > 0) {
      esp_now_send(peerAddress, (uint8_t*)audioBuffer, bytesIn);
    }
  }
}
