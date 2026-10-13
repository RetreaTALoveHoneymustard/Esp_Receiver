#include <Arduino.h>
#include "esp_adc/adc_continuous.h"

// ===================== Configuration =====================
#define TARGET_RATE      40000     // output sample rate (Hz)
#define HW_SAMPLE_RATE   40000     // hardware rate (Hz). Must be >= driver minimum.
                                   // If you set it to a multiple of TARGET_RATE,
                                   // the extra samples are decimated in software.
#define DECIMATION       (HW_SAMPLE_RATE / TARGET_RATE)

#define BLOCK_SIZE       1024      // samples per frame (at TARGET_RATE)
#define SERIAL_BAUD      1000000   // 40kHz*2B ~ 80KB/s -> ~82% of 1Mbaud. Use 2000000 for headroom.

// ADC1 channel 6: GPIO34 on classic ESP32, GPIO7 on ESP32-S3
#define ADC_CHANNEL_USED ADC_CHANNEL_6
#define ADC_ATTEN_USED   ADC_ATTEN_DB_11

// Classic ESP32 uses output format TYPE1, newer chips (S2/S3/...) use TYPE2.
#if CONFIG_IDF_TARGET_ESP32
  #define ADC_OUTPUT_FORMAT ADC_DIGI_OUTPUT_FORMAT_TYPE1
  #define SAMPLE_DATA(p)    ((p)->type1.data)
#else
  #define ADC_OUTPUT_FORMAT ADC_DIGI_OUTPUT_FORMAT_TYPE2
  #define SAMPLE_DATA(p)    ((p)->type2.data)
#endif

#define NUM_BUFFERS      2
#define ADC_CONV_BYTES   256                   // driver conversion frame (multiple of 4)
#define RAW_CHUNK_BYTES  512                   // bytes pulled from the driver per read
#define RING_BYTES       (BLOCK_SIZE * DECIMATION * SOC_ADC_DIGI_RESULT_BYTES * 8) // ~8 frames of cushion

const uint16_t FRAME_HEADER = 0xABCD;

#pragma pack(push, 1)
struct DataFrame {
  uint16_t header;
  uint32_t sequence_id;
  uint16_t samples[BLOCK_SIZE];
  uint16_t crc16;
};
#pragma pack(pop)

// ===================== Globals =====================
static DataFrame frames[NUM_BUFFERS];       // the two buffers
static QueueHandle_t free_q = NULL;         // buffers ready to be filled
static QueueHandle_t full_q = NULL;         // buffers ready to be sent

static adc_continuous_handle_t adc_handle = NULL;

static volatile uint32_t adc_overflow_count = 0;  // driver ring buffer overflows
static volatile uint32_t dropped_frames     = 0;  // frames dropped because TX was behind

// ===================== CRC =====================
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ===================== ADC =====================
static bool IRAM_ATTR on_pool_overflow(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata,
                                       void *user_data) {
  adc_overflow_count++;
  return false;
}

static void adc_init() {
  adc_continuous_handle_cfg_t handle_cfg = {};
  handle_cfg.max_store_buf_size = RING_BYTES;
  handle_cfg.conv_frame_size    = ADC_CONV_BYTES;
  ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

  adc_digi_pattern_config_t pattern = {};
  pattern.atten     = ADC_ATTEN_USED;
  pattern.channel   = ADC_CHANNEL_USED;
  pattern.unit      = ADC_UNIT_1;
  pattern.bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t dig_cfg = {};
  dig_cfg.pattern_num    = 1;
  dig_cfg.adc_pattern    = &pattern;
  dig_cfg.sample_freq_hz = HW_SAMPLE_RATE;
  dig_cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  dig_cfg.format         = ADC_OUTPUT_FORMAT;
  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_pool_ovf = on_pool_overflow;
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

  ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

// ===================== Reader task =====================
// Pulls samples from the ADC driver, fills the current buffer, and hands full
// buffers to the sender. It never blocks on the UART.
static void adc_task(void *arg) {
  static uint8_t raw[RAW_CHUNK_BYTES];
  uint32_t packet_counter = 0;
  uint32_t raw_count = 0;       // running count of raw samples (persists across frames)
  int idx = 0;                  // position in current frame

  DataFrame *cur = NULL;
  xQueueReceive(free_q, &cur, portMAX_DELAY);

  for (;;) {
    uint32_t got = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, raw, sizeof(raw), &got, 100);
    if (ret != ESP_OK) continue;   // timeout or transient error

    for (uint32_t off = 0; off + SOC_ADC_DIGI_RESULT_BYTES <= got; off += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw[off];

      if ((raw_count++ % DECIMATION) != 0) continue;
      cur->samples[idx++] = SAMPLE_DATA(p);

      if (idx < BLOCK_SIZE) continue;

      // ---- Frame complete ----
      idx = 0;
      cur->sequence_id = packet_counter++;   // increments even if dropped, so host sees the gap
      cur->crc16 = crc16_ccitt((uint8_t *)cur, sizeof(DataFrame) - sizeof(cur->crc16));

      DataFrame *next = NULL;
      if (xQueueReceive(free_q, &next, 0) == pdTRUE) {
        // Got a spare buffer: send the full one, keep filling the spare.
        xQueueSend(full_q, &cur, 0);
        cur = next;
      } else {
        // Sender is behind and no spare buffer exists: drop this frame and reuse it.
        dropped_frames++;
      }
    }
  }
}

// ===================== Sender task =====================
static void tx_task(void *arg) {
  for (;;) {
    DataFrame *f = NULL;
    xQueueReceive(full_q, &f, portMAX_DELAY);
    Serial.write((uint8_t *)f, sizeof(DataFrame));   // may block; reader keeps sampling
    xQueueSend(free_q, &f, portMAX_DELAY);           // give the buffer back
  }
}

// ===================== Arduino entry points =====================
void setup() {
  Serial.setTxBufferSize(4096);   // must be called before begin()
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  free_q = xQueueCreate(NUM_BUFFERS, sizeof(DataFrame *));
  full_q = xQueueCreate(NUM_BUFFERS, sizeof(DataFrame *));

  for (int i = 0; i < NUM_BUFFERS; i++) {
    frames[i].header = FRAME_HEADER;
    DataFrame *f = &frames[i];
    xQueueSend(free_q, &f, 0);
  }

  adc_init();

  // Reader gets the higher priority so sampling is never starved.
  xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tx_task,  "tx_task",  4096, NULL, 3, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // all work happens in the tasks
}
