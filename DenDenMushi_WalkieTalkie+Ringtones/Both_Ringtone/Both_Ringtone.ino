/* Walkie-talkie + ringtones (Versión A2)
   - Usa el mismo código en ambos ESP; cambia solo peerAddress
   - BUFFER_LEN = 120 (NO cambiar)
   - WAVs: unsigned char arrays, 16-bit PCM, 16 kHz, mono
   - Un solo botón (CALL_BUTTON) para: CALL / ANSWER / HANGUP
   - PTT_BUTTON para hablar (modo walkie)
   - Reproduce ringtone en loop en receptor, reproduce answer tone once when accepted
*/

#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>

#include "denden_esperando.h"   // const unsigned char denden_esperando_wav[], const unsigned int denden_esperando_wav_len
#include "denden_sound.h"       // const unsigned char denden_wav[],           const unsigned int denden_wav_len

// ---------- HARDWARE ----------
#define I2S_MIC_PORT I2S_NUM_0
#define MIC_BCK 26
#define MIC_WS  25
#define MIC_SD  22

#define I2S_SPK_PORT I2S_NUM_1
#define SPK_BCK 12
#define SPK_WS  27
#define SPK_DIN 14

#define PTT_BUTTON    13  // Hablar (PTT)
#define CALL_BUTTON   21  // Call / Answer / Hangup (un solo boton)

// Cambia por la MAC del peer (en cada ESP pones la MAC del otro)
uint8_t peerAddress[6] = {0x3C, 0x8A, 0x1F, 0x5D, 0x7F, 0x1C};
//2.- {0x3C, 0x8A, 0x1F, 0x5D, 0x7F, 0x1C}
//1.- {0xB4, 0xE6, 0x2D, 0xD5, 0x6C, 0x85}

// ---------- BUFFERS Y FLAGS ----------
#define BUFFER_LEN 120
int16_t audioBuffer[BUFFER_LEN];
bool isTransmitting = false;

// ---------- ESTADOS ----------
enum CallState_t { IDLE = 0, CALLING = 1, RINGING = 2, IN_CALL = 3 };
volatile CallState_t CallState = IDLE;

// ---------- RING playback state ----------
const unsigned int WAV_HEADER_OFFSET = 44;
const size_t CHUNK_BYTES = BUFFER_LEN * sizeof(int16_t); // 240 bytes

unsigned int ringOffset = WAV_HEADER_OFFSET;   // offset into waiting wav
bool ringPlayingLocal = false;
bool ringPlayingRemote = false;
bool acceptPlayed = false; // guard to play answer tone only once

// ---------- I2S set-up (USAR exactamente tus valores) ----------
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

// Blocking playback (use only for short answer tone if desired)
void playWavBlocking(const unsigned char* wav, unsigned int wav_len, float volume) {
    if (!wav || wav_len <= WAV_HEADER_OFFSET) return;

    unsigned int offset = WAV_HEADER_OFFSET;

    while (offset + 2 <= wav_len) {
        // Procesar en muestras de 16 bits
        int16_t sample = (int16_t)(wav[offset] | (wav[offset + 1] << 8));

        // Escalar volumen
        sample = (int16_t)(sample * volume);

        size_t written;
        i2s_write(I2S_SPK_PORT, &sample, sizeof(sample), &written, portMAX_DELAY);

        offset += 2; // ← AVANZA SIEMPRE
    }

    Serial.println("🔊 Reproducción finalizada.");
}

bool playWavStreamChunk(const unsigned char* wav, unsigned int wav_len, float volume, unsigned int &offset) {
    if (!wav || wav_len <= WAV_HEADER_OFFSET) return false;

    // Procesa solo un BUFFER_LEN de muestras (no bloqueante)
    for (size_t i = 0; i < BUFFER_LEN && offset + 1 < wav_len; i++) {
        int16_t sample = (int16_t)(wav[offset] | (wav[offset + 1] << 8));
        sample = (int16_t)(sample * volume);

        audioBuffer[i] = sample;
        offset += 2; // avanzar siempre
    }

    // Si llegamos al final, reiniciar y avisar que terminó
    if (offset + 1 >= wav_len) {
        offset = WAV_HEADER_OFFSET;
        return true;  // terminó
    }

    size_t written;
    i2s_write(I2S_SPK_PORT, audioBuffer, BUFFER_LEN * sizeof(int16_t), &written, portMAX_DELAY);

    return false;  // aún no termina
}

// ---------- ESP-NOW callbacks ----------
float iir_state = 0.0f;
const float iir_alpha = 0.08f;

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!data || len <= 0) return;

  // control messages (1 byte)
  if (len == 1) {
    uint8_t cmd = data[0];
    if (cmd == 0xA0) { // CALL_START
      if (CallState == IDLE) {
        CallState = RINGING;
        ringOffset = WAV_HEADER_OFFSET;
        ringPlayingRemote = true;
        ringPlayingLocal = false;
        acceptPlayed = false;
        Serial.println("📞 CALL_START recibido -> RINGING");
        // ensure speaker is active
      }
      return;
    }
    if (cmd == 0xA1) { // CALL_ACCEPT
      Serial.println("✔ CALL_ACCEPT recibido -> IN_CALL");
      CallState = IN_CALL;
      ringPlayingLocal = false;
      ringPlayingRemote = false;
      if (!acceptPlayed) {
        acceptPlayed = true;
        // play answer tone once (slightly louder)
        playWavBlocking(denden_wav, denden_wav_len, 0.18f);
        
      }
      return;
    }
    if (cmd == 0xA2) { // CALL_END
      Serial.println("❌ CALL_END recibido -> IDLE");
      CallState = IDLE;
      ringPlayingLocal = false;
      ringPlayingRemote = false;
      acceptPlayed = false;
      // play hangup tone once
      playWavBlocking(denden_wav, denden_wav_len, 0.18f);
      return;
    }
  }

  // audio payload (in call, not transmitting)
  if (CallState == IN_CALL && !isTransmitting && len > 1) {
    size_t toCopy = min((size_t)len, sizeof(audioBuffer));
    if (toCopy % 2 != 0) toCopy--;
    memcpy(audioBuffer, data, toCopy);
    size_t samples = toCopy / sizeof(int16_t);

    // IIR smoothing (same as original)
    for (size_t i = 0; i < samples; ++i) {
      float s = (float)audioBuffer[i];
      iir_state = iir_alpha * s + (1.0f - iir_alpha) * iir_state;
      audioBuffer[i] = (int16_t)iir_state;
    }

    size_t written = 0;
    i2s_write(I2S_SPK_PORT, audioBuffer, samples * sizeof(int16_t), &written, portMAX_DELAY);
  }
}

void onSendCb(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("⚠️ Fallo en transmisión");
  }
}

// ---------- helpers ----------
void sendControl(uint8_t cmd) {
  esp_err_t r = esp_now_send(peerAddress, &cmd, 1);
  if (r != ESP_OK) Serial.printf("esp_now_send ctrl returned %d\n", r);
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(PTT_BUTTON, INPUT_PULLUP);
  pinMode(CALL_BUTTON, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_err_t r = esp_now_add_peer(&peerInfo);
  if (r != ESP_OK) {
    Serial.println("Nota: esp_now_add_peer returned non-OK (peer maybe exists)");
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onSendCb);

  // start with speaker ready (idle)
  i2s_setup_speaker();
  Serial.println("Listo: modo escucha (IDLE)");
}

// ---------- loop ----------
void loop() {
  static bool lastCallBtn = HIGH;
  static bool lastPttBtn = HIGH;

  bool callBtn = digitalRead(CALL_BUTTON);
  bool pttBtn  = digitalRead(PTT_BUTTON);

  // handle CALL / ANSWER / HANGUP (edge detection)
  if (callBtn == LOW && lastCallBtn == HIGH) {
    // pressed event
    if (CallState == IDLE) {
      // initiate call
      Serial.println("📞 Enviando CALL_START");
      sendControl(0xA0);
      CallState = CALLING;
      ringOffset = WAV_HEADER_OFFSET;
      ringPlayingLocal = true;
      acceptPlayed = false;
      // ensure speaker active
      delay(100);

    } else if (CallState == RINGING) {
      // accept incoming call
      Serial.println("✔ Contestando -> envio CALL_ACCEPT");
      sendControl(0xA1);
      CallState = IN_CALL;
      ringPlayingRemote = false;
      // play answer tone once locally
      playWavBlocking(denden_wav, denden_wav_len, 0.18f);
      // switch to mic ready? Remain speaker until PTT
      i2s_driver_uninstall(I2S_SPK_PORT);
      i2s_setup_speaker(); // keep speaker active until PTT pressed; user wanted same flow
    } else if (CallState == IN_CALL || CallState == CALLING) {
      // hang up
      Serial.println("❌ Enviando CALL_END");
      sendControl(0xA2);
      CallState = IDLE;
      ringPlayingLocal = false;
      ringPlayingRemote = false;
      acceptPlayed = false;
      // play hangup tone once
      playWavBlocking(denden_wav, denden_wav_len, 0.18f);
    }
    delay(250); // debounce
  }

  // PTT pressed -> switch to mic (only when IN_CALL)
  if (pttBtn == LOW && lastPttBtn == HIGH && CallState == IN_CALL) {
    Serial.println("🎙️ PTT pressed -> transmit mode");
    isTransmitting = true;
    // uninstall speaker, setup mic (as you requested)
    i2s_driver_uninstall(I2S_SPK_PORT);
    i2s_setup_mic();
    i2s_zero_dma_buffer(I2S_MIC_PORT);
  }

  // PTT released -> stop transmit, switch speaker
  if (pttBtn == HIGH && lastPttBtn == LOW && isTransmitting) {
    Serial.println("🔈 PTT released -> listen mode");
    isTransmitting = false;
    i2s_driver_uninstall(I2S_MIC_PORT);
    i2s_setup_speaker();
  }

  // play ringtone loop if requested (non-blocking chunked)
if ((ringPlayingLocal || ringPlayingRemote) && !isTransmitting) {
    playWavStreamChunk(denden_esperando_wav, denden_esperando_wav_len, 0.05f, ringOffset);
}

  // transmit audio when PTT (and IN_CALL)
  if (isTransmitting && CallState == IN_CALL) {
    size_t bytesIn = 0;
    esp_err_t res = i2s_read(I2S_MIC_PORT, &audioBuffer, sizeof(audioBuffer), &bytesIn, portMAX_DELAY);
    if (res == ESP_OK && bytesIn > 0) {
      if (bytesIn % 2 != 0) bytesIn--;
      // bytesIn should be <= CHUNK_BYTES (240), safe for ESP-NOW
      esp_now_send(peerAddress, (uint8_t*)audioBuffer, bytesIn);
    }
  }

  lastCallBtn = callBtn;
  lastPttBtn = pttBtn;

  delay(2); // tiny delay to allow background tasks
}
