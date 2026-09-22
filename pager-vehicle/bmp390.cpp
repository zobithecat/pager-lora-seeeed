#include "bmp390.h"
#include <Wire.h>

// Register map / compensation: Bosch BMP390 datasheet rev 1.7, §4.3 + §8.4–8.6 (float 버전).
// BMP388도 같은 맵/보정식 (chip id만 0x50).
#define REG_CHIP_ID   0x00   // 0x60 = BMP390, 0x50 = BMP388
#define REG_STATUS    0x03   // bit6 drdy_temp, bit5 drdy_press, bit4 cmd_rdy
#define REG_DATA      0x04   // press xlsb/lsb/msb, temp xlsb/lsb/msb
#define REG_PWR_CTRL  0x1B   // bit0 press_en, bit1 temp_en, bits5:4 mode (01 = forced)
#define REG_OSR       0x1C   // bits2:0 osr_p, bits5:3 osr_t
#define REG_CONFIG    0x1F   // IIR
#define REG_CALIB     0x31   // 21 bytes
#define REG_CMD       0x7E   // 0xB6 = soft reset

// 전송 계층: SPI 비트뱅(mode 0)이 기본. CSB/SDO가 GPIO에 있으면 I2C 모드 잠김 문제가 없는
// SPI가 훨씬 확실하다 (2026-09-22: I2C로는 끝내 침묵하던 CJMCU-390(BMP388)이 SPI엔 바로 응답).
// 4선이 없으면 I2C(0x76/0x77)로 폴백.
static bool    s_spi = false;
static int     s_cs = -1, s_sck = -1, s_mosi = -1, s_miso = -1;
static uint8_t s_addr = 0;
static const char* s_name = nullptr;
static double t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;

static uint8_t spi_xfer(uint8_t out) {
  uint8_t in = 0;
  for (int b = 7; b >= 0; b--) {
    digitalWrite(s_mosi, (out >> b) & 1); delayMicroseconds(2);
    digitalWrite(s_sck, HIGH); delayMicroseconds(2);
    in = (in << 1) | digitalRead(s_miso);
    digitalWrite(s_sck, LOW); delayMicroseconds(2);
  }
  return in;
}
static bool rd(uint8_t reg, uint8_t* buf, size_t n) {
  if (s_spi) {
    digitalWrite(s_cs, LOW); delayMicroseconds(5);
    spi_xfer(reg | 0x80); spi_xfer(0x00);            // BMP3 SPI read: addr|0x80, dummy, data…
    for (size_t i = 0; i < n; i++) buf[i] = spi_xfer(0x00);
    digitalWrite(s_cs, HIGH); delayMicroseconds(5);
    return true;
  }
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)s_addr, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
static bool wr(uint8_t reg, uint8_t val) {
  if (s_spi) {
    digitalWrite(s_cs, LOW); delayMicroseconds(5);
    spi_xfer(reg & 0x7F); spi_xfer(val);
    digitalWrite(s_cs, HIGH); delayMicroseconds(5);
    return true;
  }
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static const char* probe(uint8_t addr) {
  s_addr = addr;
  uint8_t id = 0;
  if (s_spi) { rd(REG_CHIP_ID, &id, 1); if (id != 0x60 && id != 0x50) { delay(2); rd(REG_CHIP_ID, &id, 1); } }
  else
  if (!rd(REG_CHIP_ID, &id, 1)) return nullptr;
  if (id == 0x60) return "BMP390";
  if (id == 0x50) return "BMP388";
  return nullptr;
}

const char* bmp390_name() { return s_name; }

static const char* begin_common();
const char* bmp390_begin() {
  s_spi = false;
  s_name = probe(0x77);
  if (!s_name) s_name = probe(0x76);
  return begin_common();
}
const char* bmp390_begin_spi(int cs, int sck, int mosi, int miso) {
  s_spi = true; s_cs = cs; s_sck = sck; s_mosi = mosi; s_miso = miso;
  pinMode(s_cs, OUTPUT);   digitalWrite(s_cs, HIGH);
  pinMode(s_sck, OUTPUT);  digitalWrite(s_sck, LOW);
  pinMode(s_mosi, OUTPUT); digitalWrite(s_mosi, LOW);
  pinMode(s_miso, INPUT);
  s_name = probe(0);
  return begin_common();
}
static const char* begin_common() {
  if (!s_name) return nullptr;

  wr(REG_CMD, 0xB6);
  delay(10);

  uint8_t c[21];
  if (!rd(REG_CALIB, c, 21)) { s_name = nullptr; return nullptr; }
  uint16_t T1 = c[0] | (c[1] << 8), T2 = c[2] | (c[3] << 8);  int8_t T3 = (int8_t)c[4];
  int16_t  P1 = (int16_t)(c[5] | (c[6] << 8)), P2 = (int16_t)(c[7] | (c[8] << 8));
  int8_t   P3 = (int8_t)c[9], P4 = (int8_t)c[10];
  uint16_t P5 = c[11] | (c[12] << 8), P6 = c[13] | (c[14] << 8);
  int8_t   P7 = (int8_t)c[15], P8 = (int8_t)c[16];
  int16_t  P9 = (int16_t)(c[17] | (c[18] << 8));
  int8_t   P10 = (int8_t)c[19], P11 = (int8_t)c[20];

  t1 = T1 * 256.0;                       // / 2^-8
  t2 = T2 / 1073741824.0;                // / 2^30
  t3 = T3 / 281474976710656.0;           // / 2^48
  p1 = (P1 - 16384.0) / 1048576.0;       // (x - 2^14) / 2^20
  p2 = (P2 - 16384.0) / 536870912.0;     // (x - 2^14) / 2^29
  p3 = P3 / 4294967296.0;                // / 2^32
  p4 = P4 / 137438953472.0;              // / 2^37
  p5 = P5 * 8.0;                         // / 2^-3
  p6 = P6 / 64.0;                        // / 2^6
  p7 = P7 / 256.0;                       // / 2^8
  p8 = P8 / 32768.0;                     // / 2^15
  p9 = P9 / 281474976710656.0;           // / 2^48
  p10 = P10 / 281474976710656.0;         // / 2^48
  p11 = P11 / 36893488147419103232.0;    // / 2^65

  wr(REG_OSR, 0x03);                     // press ×8, temp ×1
  wr(REG_CONFIG, 0x00);                  // IIR off — forced 단발엔 필터 불필요
  return s_name;
}

Bme280Reading bmp390_read() {
  Bme280Reading r = {};
  if (!s_name) return r;
  if (!wr(REG_PWR_CTRL, 0x13)) return r;             // press_en | temp_en | forced

  uint8_t st = 0;
  for (int i = 0; i < 20; i++) {                     // ×8/×1 ≈ 20 ms
    delay(5);
    if (!rd(REG_STATUS, &st, 1)) return r;
    if ((st & 0x60) == 0x60) break;
  }
  if ((st & 0x60) != 0x60) return r;

  uint8_t d[6];
  if (!rd(REG_DATA, d, 6)) return r;
  double up = (double)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16));
  double ut = (double)((uint32_t)d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16));

  double a = ut - t1;
  double t = a * t2 + a * a * t3;                    // °C

  double out1 = p5 + p6 * t + p7 * t * t + p8 * t * t * t;
  double out2 = up * (p1 + p2 * t + p3 * t * t + p4 * t * t * t);
  double out3 = up * up * (p9 + p10 * t) + up * up * up * p11;
  double pa = out1 + out2 + out3;

  if (t < -50.0 || t > 100.0 || pa < 25000.0 || pa > 130000.0) return r;   // 스펙 밖 = 읽기 오류
  r.temp_c = (float)t;
  r.press_hpa = (float)(pa / 100.0);
  r.has_humidity = false;
  r.ok = true;
  return r;
}
