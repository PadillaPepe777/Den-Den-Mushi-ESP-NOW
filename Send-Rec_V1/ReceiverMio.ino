#include <WiFi.h>
#include <esp_now.h>

#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_PORT I2S_NUM_0
#define PIN_BCK  26
#define PIN_WS   25
#define PIN_DOUT 22

#define bufferLen 120 //antes 64

int16_t sBuffer[bufferLen];
size_t bytesWritten;

void esp_now_recv_cb(const esp_now_recv_info_t*esp_now_info, const uint8_t *data, int data_len);

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 48000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // MONO
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = true,
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

void esp_now_recv_cb(const esp_now_recv_info_t*esp_now_info, const uint8_t *data, int data_len) {
  memcpy(&sBuffer, data, sizeof(sBuffer));
  for (size_t i = 0; i < ((sizeof(sBuffer))/2); i ++) {
    int16_t sample = (sBuffer[i]);
    sample = (int16_t)(sample * 0.1f);  // 🔉 volumen al 5% se pondria *0.05f
    i2s_write(I2S_PORT, &sample, sizeof(sample), &bytesWritten, portMAX_DELAY);
    }
    //Serial.println("Todo bien aqui");
}

void setup() {
  Serial.begin(115200);
  Serial.println(" ");
  delay(1000);

  WiFi.mode(WIFI_STA); //mODO ESTACION STAtion, COMO SI SE FUERA conectar a un wifi como cliente

  if(esp_now_init() != ESP_OK)
  {
    Serial.println("Error al cargar dispositivo par");
    return; //para saltarse el void setup porque falla algo critico, tambien sirve un while(true) para que mejor no haga nada
  }

  esp_now_register_recv_cb(esp_now_recv_cb);



  setupI2S();


}

void loop() {


}
