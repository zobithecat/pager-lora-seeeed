#pragma once
#include <Arduino.h>
#include "bme280.h"   // Bme280Reading을 공용 측정값 구조체로 같이 쓴다 (has_humidity=false)

// 최소 BMP390/BMP388 드라이버 (I2C, forced mode 단발 측정). CJMCU-390 = BMP390L.
// 기압 + 온도만 나온다 — 습도 없음. 주소 0x77(SDO=VDD) / 0x76(SDO=GND), CSB는 High여야 I2C.

// Wire.begin()은 호출자가 먼저. 0x76/0x77 둘 다 시도. 찾으면 "BMP390" / "BMP388", 못 찾으면 nullptr.
const char* bmp390_begin();                                   // I2C
const char* bmp390_begin_spi(int cs, int sck, int mosi, int miso);  // SPI 비트뱅 mode 0 (권장)
const char* bmp390_name();
Bme280Reading bmp390_read();          // ~25 ms 블로킹
