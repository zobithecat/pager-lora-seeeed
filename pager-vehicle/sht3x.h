#pragma once
#include <Arduino.h>
#include "bme280.h"   // Bme280Reading 공용 구조체 (press_hpa = NAN, has_humidity = true)

// 최소 SHT30/31/35 드라이버 (I2C, single-shot 고반복성). 온도 + 습도, 기압 없음.
// 주소 0x44(ADDR=GND, 기본) / 0x45(ADDR=VDD) 자동 탐색. Wire.begin()은 호출자가 먼저.
bool sht3x_begin();
Bme280Reading sht3x_read();   // ~16 ms 블로킹
