#pragma once
#include <stdint.h>

// 자모 (U+3131 ~ U+3163) 51자에 대한 16x16 비트맵 폰트.
// row-major, MSB-first per byte. row당 2 byte (16 픽셀 폭) × 16 rows = 32 bytes.
//
// U8g2의 drawBitmap(x, y, cnt, h, ptr)으로 그린다. cnt=2, h=16.
// drawBitmap의 y는 비트맵 top-left (drawUTF8과 좌표계 다름).
//
// 데이터는 tools/extract_jamo.py로 GNU Unifont에서 추출.

extern const uint8_t JAMO_FONT[51][32];

// codepoint가 자모 범위면 비트맵 포인터, 아니면 nullptr.
const uint8_t* jamo_font_lookup(uint32_t codepoint);
