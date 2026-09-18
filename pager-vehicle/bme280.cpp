#include "bme280.h"
#include <Wire.h>

// Register map / compensation: Bosch BME280 datasheet rev 1.6, §4.2.3 + §5.
#define REG_CALIB_TP   0x88   // 0x88..0xA1 : dig_T1..dig_P9, (0xA0 unused), dig_H1
#define REG_CHIP_ID    0xD0
#define REG_RESET      0xE0
#define REG_CALIB_H    0xE1   // 0xE1..0xE7 : dig_H2..dig_H6
#define REG_CTRL_HUM   0xF2
#define REG_STATUS     0xF3
#define REG_CTRL_MEAS  0xF4
#define REG_CONFIG     0xF5
#define REG_DATA       0xF7   // press[3] temp[3] hum[2]

static uint8_t    s_addr = 0;
static Bme280Chip s_chip = Bme280Chip::None;

static uint16_t dig_T1; static int16_t dig_T2, dig_T3;
static uint16_t dig_P1; static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3; static int16_t dig_H2, dig_H4, dig_H5; static int8_t dig_H6;

static bool rd(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)s_addr, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
static bool wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static inline uint16_t u16le(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline int16_t  s16le(const uint8_t* p) { return (int16_t)u16le(p); }

static Bme280Chip probe(uint8_t addr) {
  s_addr = addr;
  uint8_t id = 0;
  if (!rd(REG_CHIP_ID, &id, 1)) return Bme280Chip::None;
  if (id == 0x60) return Bme280Chip::BME280;
  if (id == 0x58 || id == 0x56 || id == 0x57) return Bme280Chip::BMP280;
  return Bme280Chip::None;
}

Bme280Chip bme280_chip() { return s_chip; }

Bme280Chip bme280_begin(uint8_t addr) {
  s_chip = probe(addr);
  if (s_chip == Bme280Chip::None) s_chip = probe(addr == 0x76 ? 0x77 : 0x76);
  if (s_chip == Bme280Chip::None) return s_chip;

  wr(REG_RESET, 0xB6);
  delay(5);                               // NVM → 레지스터 복사 대기 (start-up 2 ms)

  uint8_t c[26];
  if (!rd(REG_CALIB_TP, c, 26)) { s_chip = Bme280Chip::None; return s_chip; }
  dig_T1 = u16le(c + 0);  dig_T2 = s16le(c + 2);  dig_T3 = s16le(c + 4);
  dig_P1 = u16le(c + 6);  dig_P2 = s16le(c + 8);  dig_P3 = s16le(c + 10);
  dig_P4 = s16le(c + 12); dig_P5 = s16le(c + 14); dig_P6 = s16le(c + 16);
  dig_P7 = s16le(c + 18); dig_P8 = s16le(c + 20); dig_P9 = s16le(c + 22);
  dig_H1 = c[25];

  if (s_chip == Bme280Chip::BME280) {
    uint8_t h[7];
    if (!rd(REG_CALIB_H, h, 7)) { s_chip = Bme280Chip::None; return s_chip; }
    dig_H2 = s16le(h + 0);
    dig_H3 = h[2];
    dig_H4 = (int16_t)(((int16_t)(int8_t)h[3] * 16) | (h[4] & 0x0F));   // 12-bit signed
    dig_H5 = (int16_t)(((int16_t)(int8_t)h[5] * 16) | (h[4] >> 4));
    dig_H6 = (int8_t)h[6];
  }
  wr(REG_CONFIG, 0x00);                   // IIR off, SPI3w off — forced 단발엔 필터 불필요
  return s_chip;
}

Bme280Reading bme280_read() {
  Bme280Reading r = {};
  if (s_chip == Bme280Chip::None) return r;
  r.has_humidity = (s_chip == Bme280Chip::BME280);

  // ctrl_hum은 ctrl_meas를 쓴 뒤에야 반영된다 → 순서 고정. 전부 oversampling ×1, forced.
  if (r.has_humidity && !wr(REG_CTRL_HUM, 0x01)) return r;
  if (!wr(REG_CTRL_MEAS, (1 << 5) | (1 << 2) | 0x01)) return r;

  uint8_t st = 0;
  for (int i = 0; i < 10; i++) {          // ×1/×1/×1 최대 9.3 ms
    delay(5);
    if (!rd(REG_STATUS, &st, 1)) return r;
    if (!(st & 0x08)) break;
  }
  if (st & 0x08) return r;

  uint8_t d[8];
  if (!rd(REG_DATA, d, r.has_humidity ? 8 : 6)) return r;
  int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
  if (adc_T == 0x80000) return r;         // 측정 skip 값

  // --- temperature (0.01 °C) ---
  int32_t v1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t v2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                ((int32_t)dig_T3)) >> 14;
  int32_t t_fine = v1 + v2;
  r.temp_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

  // --- pressure (Pa × 256, 64-bit) ---
  int64_t p1 = ((int64_t)t_fine) - 128000;
  int64_t p2 = p1 * p1 * (int64_t)dig_P6;
  p2 = p2 + ((p1 * (int64_t)dig_P5) << 17);
  p2 = p2 + (((int64_t)dig_P4) << 35);
  p1 = ((p1 * p1 * (int64_t)dig_P3) >> 8) + ((p1 * (int64_t)dig_P2) << 12);
  p1 = (((((int64_t)1) << 47) + p1)) * ((int64_t)dig_P1) >> 33;
  if (p1 != 0) {
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p2) * 3125) / p1;
    p1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + p1 + p2) >> 8) + (((int64_t)dig_P7) << 4);
    r.press_hpa = (float)p / 25600.0f;
  }

  // --- humidity (%RH × 1024) ---
  if (r.has_humidity) {
    int32_t adc_H = ((int32_t)d[6] << 8) | d[7];
    int32_t h = t_fine - ((int32_t)76800);
    h = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * h)) + ((int32_t)16384)) >> 15) *
         (((((((h * ((int32_t)dig_H6)) >> 10) * (((h * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    h = h - (((((h >> 15) * (h >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4);
    if (h < 0) h = 0;
    if (h > 419430400) h = 419430400;
    r.hum_pct = (float)(h >> 12) / 1024.0f;
  }

  r.ok = true;
  return r;
}
