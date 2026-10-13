#include <Arduino.h>
#include "driver/i2s_std.h"

// ===================== Configuration =====================
#define SAMPLE_RATE   16000
#define BLOCK_SIZE    1024

// I2S pins (change to match your wiring)
#define I2S_BCLK      26   // INMP441 SCK
#define I2S_WS        25   // INMP441 WS
#define I2S_DIN       33   // INMP441 SD

const uint16_t FRAME_HEADER = 0xABCD;

#pragma pack(push, 1)
struct DataFrame {
  uint16_t header;
  uint32_t sequence_id;
  int16_t  samples[BLOCK_SIZE];   // signed 16-bit audio
  uint16_t crc16;
};
#pragma pack(pop)

DataFrame frame;
uint32_t packet_counter = 0;
i2s_chan_handle_t rx_handle = NULL;

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

void i2s_init() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    // INMP441 sends 24-bit data inside a 32-bit slot
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK,
      .ws   = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_DIN,
      .invert_flags = { false, false, false },
    },
  };
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;   // L/R pin tied to GND

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

void setup() {
  Serial.begin(921600);          // 2000000 also fine if your USB-UART supports it
  while (!Serial);
  frame.header = FRAME_HEADER;
  i2s_init();
}

void loop() {
  static int32_t raw[BLOCK_SIZE];
  size_t bytes_read = 0;
  size_t total = 0;

  // Read exactly BLOCK_SIZE 32-bit samples
  while (total < sizeof(raw)) {
    i2s_channel_read(rx_handle, (uint8_t*)raw + total, sizeof(raw) - total,
                     &bytes_read, portMAX_DELAY);
    total += bytes_read;
  }

  // Convert 32-bit slot (24-bit data, left-aligned) to 16-bit
  for (int i = 0; i < BLOCK_SIZE; i++) {
    frame.samples[i] = (int16_t)(raw[i] >> 16);
  }

  frame.sequence_id = packet_counter++;
  frame.crc16 = crc16_ccitt((uint8_t*)&frame, sizeof(DataFrame) - sizeof(frame.crc16));
  Serial.write((uint8_t*)&frame, sizeof(DataFrame));
}