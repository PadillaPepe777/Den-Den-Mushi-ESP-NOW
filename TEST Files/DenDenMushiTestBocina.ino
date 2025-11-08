#include <Arduino.h>
#include <driver/i2s.h>
#include "sound.h"

#define I2S_PORT I2S_NUM_0
#define PIN_BCK  26
#define PIN_WS   25
#define PIN_DOUT 22

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 48000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // MONO
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_BCK,
    .ws_io_num = PIN_WS,
    .data_out_num = PIN_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, 48000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void setup() {
  Serial.begin(115200);
  setupI2S();

  size_t bytesWritten;
for (size_t i = 44; i < sound_wav_len; i += 2) {
  int16_t sample = (int16_t)(sound_wav[i] | (sound_wav[i + 1] << 8));
  sample = (int16_t)(sample * 0.05f);  // 🔉 volumen al 5%
  i2s_write(I2S_PORT, &sample, sizeof(sample), &bytesWritten, portMAX_DELAY);
}

  Serial.println("Reproducción finalizada.");
}

void loop() {}
