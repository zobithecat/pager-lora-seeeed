#pragma once
#include <stdint.h>

// USB HID Keyboard Usage IDs (page 0x07)
enum : uint8_t {
  HID_NONE      = 0x00,
  HID_A         = 0x04,
  HID_Z         = 0x1D,
  HID_1         = 0x1E,
  HID_0         = 0x27,
  HID_ENTER     = 0x28,
  HID_ESC       = 0x29,
  HID_BACKSPACE = 0x2A,
  HID_TAB       = 0x2B,
  HID_SPACE     = 0x2C,
  HID_MINUS     = 0x2D,
  HID_EQUAL     = 0x2E,
  HID_LBRACKET  = 0x2F,
  HID_RBRACKET  = 0x30,
  HID_BACKSLASH = 0x31,
  HID_SEMICOLON = 0x33,
  HID_QUOTE     = 0x34,
  HID_GRAVE     = 0x35,
  HID_COMMA     = 0x36,
  HID_PERIOD    = 0x37,
  HID_SLASH     = 0x38,
  HID_CAPSLOCK  = 0x39,
  HID_RIGHT     = 0x4F,
  HID_LEFT      = 0x50,
  HID_DOWN      = 0x51,
  HID_UP        = 0x52,
  HID_LANG1     = 0x90,   // 한/영 (Korean keyboards)
  HID_LANG2     = 0x91,   // 한자
};

// Modifier byte bits
constexpr uint8_t MOD_LCTRL  = 0x01;
constexpr uint8_t MOD_LSHIFT = 0x02;
constexpr uint8_t MOD_LALT   = 0x04;
constexpr uint8_t MOD_LGUI   = 0x08;
constexpr uint8_t MOD_RCTRL  = 0x10;
constexpr uint8_t MOD_RSHIFT = 0x20;
constexpr uint8_t MOD_RALT   = 0x40;
constexpr uint8_t MOD_RGUI   = 0x80;

// Convert HID usage code + shift to printable ASCII. 0 if not printable.
char hid_to_ascii(uint8_t usage, bool shift);
