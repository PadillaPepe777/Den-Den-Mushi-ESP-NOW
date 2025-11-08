/* Receiver: recibe bloques int16 por ESP-NOW y los reproduce por I2S.
   Aplica filtro IIR simple para suavizar y escribe bloques enteros al I2S. */
#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>

#define I2S_PORT I2S_NUM_0
#define PIN_BCK  26
#define PIN_WS   25
#define PIN_DOUT 22

#define BUFFER_LEN 122
int16_t playBuffer[BUFFER_LEN];
size_t bytesWritten;

void i2s_setup_tx() {
  i2s_config_t cfg = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 10,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = true
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_pin_config_t pinp = { .bck_io_num = PIN_BCK, .ws_io_num = PIN_WS, .data_out_num = PIN_DOUT, .data_in_num = I2S_PIN_NO_CHANGE };
  i2s_set_pin(I2S_PORT, &pinp);
  i2s_set_clk(I2S_PORT, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

// filtro IIR simple entre bloques para suavizar jitter
float iir_state = 0.0f;
const float iir_alpha = 0.08f; // 0.05-0.25 probar

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  // copia segura: hasta BUFFER_LEN samples (bytes)
  size_t toCopy = min((size_t)len, BUFFER_LEN * sizeof(int16_t));
  memcpy(playBuffer, data, toCopy);
  size_t samples = toCopy / sizeof(int16_t);

  // aplicar filtro simple: suavizado por IIR por muestra
  for (size_t i = 0; i < samples; ++i) {
    float s = (float)playBuffer[i];
    iir_state = iir_alpha * s + (1.0f - iir_alpha) * iir_state;
    int16_t out = (int16_t) iir_state;
    playBuffer[i] = out;
  }

  // Escribir todo el bloque de una vez: reduce overhead y jitter
  size_t bytesToWrite = samples * sizeof(int16_t);
  size_t written = 0;
  i2s_write(I2S_PORT, (const void*)playBuffer, bytesToWrite, &written, portMAX_DELAY);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init fail");
    while(1);
  }
  esp_now_register_recv_cb(onDataRecv);
  i2s_setup_tx();
}

void loop() {
  // vacío: reproducción por callback
}
