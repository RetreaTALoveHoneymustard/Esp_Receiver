#include <Arduino.h>

// ===================== Configuration =====================
#define SAMPLE_RATE     10000  // 10 kSa/s (100 us period)
#define BLOCK_SIZE      256
#define ADC_PIN         34       // Pin D34 (ADC1 channel, input-only pin)

const uint16_t FRAME_HEADER = 0xABCD;

// ===================== Frame format =====================
// header(2) + sequence_id(4) + samples(BLOCK_SIZE*2) + crc16(2)
#pragma pack(push, 1)
struct DataFrame {
  uint16_t header;        // 0xABCD
  uint32_t sequence_id;   // Frame counter
  uint16_t samples[BLOCK_SIZE];
  uint16_t crc16;         // CRC over header+sequence_id+samples
};
#pragma pack(pop)

DataFrame frame;
uint32_t packet_counter = 0;
const uint32_t SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE;

// ===================== CRC16-CCITT (0xFFFF init, poly 0x1021) =====================
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

// ===================== Setup =====================
void setup() {
  Serial.begin(2000000); // 2 Mbps
  while (!Serial);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db); // covers ~0-3.3V input range

  frame.header = FRAME_HEADER;
}

// ===================== Main loop =====================
void loop() {
  frame.sequence_id = packet_counter++;

  uint32_t next_sample_time = micros();
  bool overrun = false;

  for (int i = 0; i < BLOCK_SIZE; i++) {
    // Overflow-safe wait: works correctly across the micros() wraparound
    while ((int32_t)(micros() - next_sample_time) < 0) {
      // busy-wait
    }

    // Detect if we've fallen behind schedule (useful for debugging jitter)
    if ((int32_t)(micros() - next_sample_time) > (int32_t)SAMPLE_INTERVAL_US) {
      overrun = true;
    }

    next_sample_time += SAMPLE_INTERVAL_US;
    frame.samples[i] = analogRead(ADC_PIN);
  }

  // Compute CRC over everything except the CRC field itself
  frame.crc16 = crc16_ccitt((uint8_t*)&frame, sizeof(DataFrame) - sizeof(frame.crc16));

  Serial.write((uint8_t*)&frame, sizeof(DataFrame));

  // Optional: flag timing overruns on a spare pin or via a status byte scheme
  // if you need to know sampling was not gapless (analogRead() likely too slow
  // for a strict 100us budget - see note below).
  (void)overrun;
}
