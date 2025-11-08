#include <driver/i2s.h>
#include <SPI.h>
#include <SD.h>

// --- Pines del micrófono ---
#define I2S_WS 25
#define I2S_SD 22
#define I2S_SCK 26
#define I2S_PORT I2S_NUM_0
#define bufferLen 512
int16_t sBuffer[bufferLen];

// --- Pines de la microSD ---
#define SD_CS 5

// --- Duración de la grabación (segundos) ---
#define RECORD_TIME 10

// --- Frecuencia de muestreo ---
#define SAMPLE_RATE 44100

File audioFile;

// ---------------- I2S setup ----------------
void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
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

// ---------------- WAV Header ----------------
void writeWavHeader(File file, uint32_t sampleRate, uint16_t bitsPerSample, uint32_t numSamples) {
  uint32_t dataSize = numSamples * (bitsPerSample / 8);
  uint32_t chunkSize = 36 + dataSize;

  file.write((const uint8_t*)"RIFF", 4);
  file.write((uint8_t*)&chunkSize, 4);
  file.write((const uint8_t*)"WAVE", 4);
  file.write((const uint8_t*)"fmt ", 4);

  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1; // PCM
  uint16_t numChannels = 1;
  file.write((uint8_t*)&subchunk1Size, 4);
  file.write((uint8_t*)&audioFormat, 2);
  file.write((uint8_t*)&numChannels, 2);
  file.write((uint8_t*)&sampleRate, 4);
  uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
  file.write((uint8_t*)&byteRate, 4);
  uint16_t blockAlign = numChannels * (bitsPerSample / 8);
  file.write((uint8_t*)&blockAlign, 2);
  file.write((uint8_t*)&bitsPerSample, 2);

  file.write((const uint8_t*)"data", 4);
  file.write((uint8_t*)&dataSize, 4);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando...");

  // --- Inicializa SD ---
  if (!SD.begin(SD_CS)) {
    Serial.println("Fallo al montar la SD");
    while (true);
  }

  // --- Inicializa I2S ---
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  // --- Crear archivo WAV ---
  audioFile = SD.open("/record.wav", FILE_WRITE);
  if (!audioFile) {
    Serial.println("No se pudo crear el archivo en la SD");
    while (true);
  }

  Serial.println("Grabando 10 segundos de audio...");

  // Reservar espacio aproximado (10s * 44100 muestras * 2 bytes)
  writeWavHeader(audioFile, SAMPLE_RATE, 16, SAMPLE_RATE * RECORD_TIME);

  unsigned long startTime = millis();
  size_t bytesIn = 0;

  while (millis() - startTime < (RECORD_TIME * 1000)) {
    i2s_read(I2S_PORT, (void*)sBuffer, sizeof(sBuffer), &bytesIn, portMAX_DELAY);
    if (bytesIn > 0) {
      audioFile.write((byte*)sBuffer, bytesIn);
    }
  }

  audioFile.close();
  Serial.println("Grabación completada. Archivo guardado como /record.wav");
}

void loop() {}
