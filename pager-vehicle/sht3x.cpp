#include "sht3x.h"
#include <Wire.h>

// Sensirion SHT3x datasheet §4.3–4.13. 명령은 16비트 big-endian, 응답 6B = T(2)+CRC, RH(2)+CRC.
static uint8_t s_addr = 0;

static bool cmd(uint16_t c) {
  Wire.beginTransmission(s_addr);
  Wire.write((uint8_t)(c >> 8));
  Wire.write((uint8_t)c);
  return Wire.endTransmission() == 0;
}
static uint8_t crc8(const uint8_t* d, int n) {       // poly 0x31, init 0xFF
  uint8_t crc = 0xFF;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}
static bool probe(uint8_t addr) {
  s_addr = addr;
  if (!cmd(0x30A2)) return false;                    // soft reset
  delay(2);
  if (!cmd(0xF32D)) return false;                    // read status register
  if (Wire.requestFrom((int)addr, 3) != 3) return false;
  uint8_t b[3]; for (auto& x : b) x = Wire.read();
  return crc8(b, 2) == b[2];
}

bool sht3x_begin() {
  if (probe(0x44) || probe(0x45)) { cmd(0x3041); return true; }   // clear status
  s_addr = 0;
  return false;
}

Bme280Reading sht3x_read() {
  Bme280Reading r = {};
  r.press_hpa = NAN;
  r.has_humidity = true;
  if (!s_addr) return r;
  if (!cmd(0x2400)) return r;                        // single shot, high repeatability, no clock stretch
  delay(16);                                         // 최대 15.5 ms
  if (Wire.requestFrom((int)s_addr, 6) != 6) return r;
  uint8_t d[6]; for (auto& x : d) x = Wire.read();
  if (crc8(d, 2) != d[2] || crc8(d + 3, 2) != d[5]) return r;
  uint16_t t = (d[0] << 8) | d[1], h = (d[3] << 8) | d[4];
  r.temp_c  = -45.0f + 175.0f * (float)t / 65535.0f;
  r.hum_pct = 100.0f * (float)h / 65535.0f;
  if (r.hum_pct < 0) r.hum_pct = 0; if (r.hum_pct > 100) r.hum_pct = 100;
  r.ok = true;
  return r;
}
