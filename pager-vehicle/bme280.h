#pragma once
#include <Arduino.h>

// 최소 BME280 드라이버 (I2C, forced mode 단발 측정). 외부 라이브러리 의존 없음.
// "BME280"으로 팔리는 보드 중 상당수가 실제론 BMP280(습도 없음, chip id 0x58)이라
// 둘 다 받아들이고, BMP280이면 has_humidity=false로 알린다.

enum class Bme280Chip : uint8_t { None = 0, BME280, BMP280 };

struct Bme280Reading {
  bool  ok;
  float temp_c;
  float hum_pct;       // has_humidity == false면 의미 없음
  float press_hpa;
  bool  has_humidity;
};

// Wire.begin()은 호출자가 먼저 해둘 것. addr에서 못 찾으면 다른 주소(0x76↔0x77)도 시도.
Bme280Chip bme280_begin(uint8_t addr);
Bme280Chip bme280_chip();
// forced 측정 1회 (~10 ms 블로킹). 센서가 없거나 I2C 오류면 ok=false.
Bme280Reading bme280_read();
