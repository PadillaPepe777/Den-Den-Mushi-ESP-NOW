/*
    ESP32 I2S Ejemplo de uso del micrófono
    Muestrea sonido con el micrófono y lo despliega en el graficador serial
    Requiere un Micrófono INMP441
*/
//L/R HIGH es Right, LOW es Left
#include <WiFi.h>
#include <esp_now.h>
// Incluir el driver I2S
#include <driver/i2s.h>
// Conexiones al INMP441
#define I2S_WS 25
#define I2S_SD 22
#define I2S_SCK 26
// Uso del procesador 0 I2S
#define I2S_PORT I2S_NUM_0
// Define la longitud del buffer de entrada
#define bufferLen 120 //antes 64
int16_t sBuffer[bufferLen];

uint8_t macAddress[6]={0x3C,0x8A,0x1F,0x5D,0x7F,0x1C}; //direccion mac


void esp_now_send_cb(const uint8_t *macAddress, esp_now_send_status_t status); //funcion para conocer el estatus de envio de la señal

esp_now_peer_info_t peerInformation; //Variable para conocer i formacion del dispositivo par

void i2s_install() {
  // Configuración del I2S
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 48000,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = true
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}
void i2s_setpin() {
  // Configuración de los pines I2S
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}
void setup() {
  // Configuración del monitor serial
  Serial.begin(115200);
  Serial.println(" ");
  delay(1000);

  WiFi.mode(WIFI_STA); //mODO ESTACION STAtion, COMO SI SE FUERA conectar a un wifi como cliente

  if(esp_now_init() != ESP_OK)
  {
    Serial.println("Error al cargar dispositivo par");
    return; //para saltarse el void setup porque falla algo critico, tambien sirve un while(true) para que mejor no haga nada
  }

  memcpy(peerInformation.peer_addr, macAddress, 6); // Dirección MAC del receptor
  peerInformation.channel = 0;                      // Canal WiFi (0 usa el actual)
  peerInformation.encrypt = false;                  // Si no estás usando cifrado, se deja en false


  if(esp_now_add_peer(&peerInformation) !=ESP_OK)
  {
    Serial.println("Error al cargar dispositivo par");
    return; //para saltarse el void setup porque falla algo critico, tambien sirve un while(true) para que mejor no haga nada
  }

  esp_now_register_send_cb(esp_now_send_cb); //Registrar funcon para informar estado de la comunicacion

  // Inicialización del I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
}
void loop() {
  // Impresión de valores falsos para "bloquear el rango" en el graficador serial
  // Cambiar el valor de rangelimit para modificar la resolución de la gráfica
  /*int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");
  */
  // Obtención de los datos I2S y moverlos al buffer de datos
size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
  if (result == ESP_OK) {
    // Lectura del buffer de datos I2S
    int16_t muestras = bytesIn / 8;
    if (muestras > 0) {
      float promedio = 0;
      for (int16_t i = 0; i < muestras; ++i) {
        promedio += (sBuffer[i]);
      }
      // Promedio de los datos leídos
      promedio /= muestras;
      // Gráfica de los datos obtenidos
      //Serial.println(promedio);
    }
  }
  /*if (result == ESP_OK) {
    // Lectura del buffer de datos I2S
    int16_t muestras = bytesIn / 2;
    if (muestras > 0) {
      float promedio = 0;
      for (int16_t i = 0; i < muestras; ++i) {
        promedio += (sBuffer[i]);
      }
      // Promedio de los datos leídos
      promedio /= muestras;
      // Gráfica de los datos obtenidos
      Serial.println(promedio);
    }
  }*/

    if (result == ESP_OK)
    esp_now_send(macAddress, (uint8_t*)&sBuffer, bufferLen * sizeof(int16_t));


  }

  void esp_now_send_cb(const uint8_t *macAddress, esp_now_send_status_t status)
  {
    if(status==ESP_NOW_SEND_SUCCESS)
    {
      //Serial.println("Transmision exitosa");
    }
    else
    {
      Serial.println("Fallo transmision");
    }
  }
