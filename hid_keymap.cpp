#include "hid_keymap.h"

char hid_to_ascii(uint8_t usage, bool shift) {
  if (usage >= HID_A && usage <= HID_Z) {
    char c = 'a' + (usage - HID_A);
    return shift ? (char)(c - 32) : c;
  }
  if (usage >= HID_1 && usage <= 0x26) {        // 1..9
    static const char shifted[] = "!@#$%^&*(";
    return shift ? shifted[usage - HID_1] : (char)('1' + (usage - HID_1));
  }
  if (usage == HID_0) return shift ? ')' : '0';
  switch (usage) {
    case HID_SPACE:     return ' ';
    case HID_TAB:       return '\t';
    case HID_MINUS:     return shift ? '_' : '-';
    case HID_EQUAL:     return shift ? '+' : '=';
    case HID_LBRACKET:  return shift ? '{' : '[';
    case HID_RBRACKET:  return shift ? '}' : ']';
    case HID_BACKSLASH: return shift ? '|' : '\\';
    case HID_SEMICOLON: return shift ? ':' : ';';
    case HID_QUOTE:     return shift ? '"' : '\'';
    case HID_GRAVE:     return shift ? '~' : '`';
    case HID_COMMA:     return shift ? '<' : ',';
    case HID_PERIOD:    return shift ? '>' : '.';
    case HID_SLASH:     return shift ? '?' : '/';
  }
  return 0;
}
