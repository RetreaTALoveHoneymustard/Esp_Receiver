#include <Arduino.h>
#include "esp_adc/adc_continuous.h"

// ===================== Configuration =====================
// The ESP32's continuous/DMA ADC mode has a HARDWARE MINIMUM sample rate of
// 20,000 Hz (it's built on the I2S peripheral's clock divider, which can't
// go slower than that). Requesting 10000 directly makes
// adc_continuous_config() fail with ESP_ERR_INVALID_ARG.
//
// Fix: sample at the hardware minimum (still fully hardware/DMA-timed, still
// gapless) and keep every 2nd sample in software. Because the underlying
// 20kHz stream is genuinely evenly spaced, keeping every other sample
// (0, 2, 4, 6...) gives an exactly evenly-spaced 10kHz result -- no jitter
// reintroduced, since which samples get kept is fixed, not timing-dependent.
#define TARGET_RATE      10000
#define HW_SAMPLE_RATE   20000                              // ESP32 continuous-ADC floor
#define DECIMATION       (HW_SAMPLE_RATE / TARGET_RATE)      // = 2

#define BLOCK_SIZE       256      // output samples per frame (at TARGET_RATE)
#define ADC_CHANNEL      ADC_CHANNEL_6   // GPIO34 on classic ESP32 = ADC1_CH6
#define ADC_ATTEN        ADC_ATTEN_DB_11

const uint16_t FRAME_HEADER = 0xABCD;

#pragma pack(push, 1)
struct DataFrame {
  uint16_t header;
  uint32_t sequence_id;
  uint16_t samples[BLOCK_SIZE];
  uint16_t crc16;
};
#pragma pack(pop)

DataFrame frame;
uint32_t packet_counter = 0;

adc_continuous_handle_t adc_handle = NULL;

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

void adc_init() {
  // Raw hardware frame size, before decimation.
  const size_t raw_frame_size = BLOCK_SIZE * DECIMATION * SOC_ADC_DIGI_RESULT_BYTES;

  adc_continuous_handle_cfg_t handle_cfg = {};
  handle_cfg.max_store_buf_size = raw_frame_size * 4;  // ring buffer headroom
  handle_cfg.conv_frame_size    = raw_frame_size;
  ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

  adc_digi_pattern_config_t pattern = {};
  pattern.atten     = ADC_ATTEN;
  pattern.channel   = ADC_CHANNEL;
  pattern.unit      = ADC_UNIT_1;
  pattern.bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t dig_cfg = {};
  dig_cfg.pattern_num    = 1;
  dig_cfg.adc_pattern    = &pattern;
  dig_cfg.sample_freq_hz = HW_SAMPLE_RATE;   // hardware floor, NOT the target rate
  dig_cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  dig_cfg.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1; // classic ESP32 format

  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
  ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void setup() {
  Serial.begin(2000000);
  while (!Serial);
  frame.header = FRAME_HEADER;
  adc_init();
}

void loop() {
  static uint8_t raw_buf[BLOCK_SIZE * DECIMATION * SOC_ADC_DIGI_RESULT_BYTES];
  int idx = 0;         // index into frame.samples (decimated, TARGET_RATE)
  int raw_count = 0;   // running count of raw HW_SAMPLE_RATE samples seen

  while (idx < BLOCK_SIZE) {
    uint32_t bytes_read = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, raw_buf, sizeof(raw_buf), &bytes_read, 1000);
    if (ret != ESP_OK) continue;

    adc_digi_output_data_t *p = (adc_digi_output_data_t *)raw_buf;
    int n = bytes_read / SOC_ADC_DIGI_RESULT_BYTES;
    for (int i = 0; i < n && idx < BLOCK_SIZE; i++) {
      if ((raw_count % DECIMATION) == 0) {
        frame.samples[idx++] = p[i].type1.data;
      }
      raw_count++;
    }
  }

  frame.sequence_id = packet_counter++;
  frame.crc16 = crc16_ccitt((uint8_t*)&frame, sizeof(DataFrame) - sizeof(frame.crc16));
  Serial.write((uint8_t*)&frame, sizeof(DataFrame));
}
