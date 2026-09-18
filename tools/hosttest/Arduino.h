#pragma once
#include <string>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cmath>
#include <type_traits>
#define IRAM_ATTR
#define ESP32 1
extern uint32_t g_now;
inline uint32_t millis() { return g_now; }
inline void delay(uint32_t ms) { g_now += ms; }
#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 0; }
inline uint32_t esp_random() { static uint32_t x = 12345; x = x * 1664525u + 1013904223u; return x >> 8; }
class String {
 public:
  std::string s;
  String() {}
  String(const char* p) : s(p ? p : "") {}
  String(const std::string& p) : s(p) {}
  template <typename T, typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, char>::value, int>::type = 0>
  String(T v) : s(std::to_string((long long)v)) {}
  String(float v, unsigned d = 2) { char b[32]; snprintf(b, sizeof b, "%.*f", (int)d, (double)v); s = b; }
  size_t length() const { return s.size(); }
  const char* c_str() const { return s.c_str(); }
  char charAt(size_t i) const { return i < s.size() ? s[i] : 0; }
  char operator[](size_t i) const { return charAt(i); }
  String substring(size_t a) const { return a >= s.size() ? String() : String(s.substr(a)); }
  String substring(size_t a, size_t b) const { if (b > s.size()) b = s.size(); return a >= b ? String() : String(s.substr(a, b - a)); }
  int indexOf(char c, size_t from = 0) const { auto p = s.find(c, from); return p == std::string::npos ? -1 : (int)p; }
  int indexOf(const char* c, size_t from = 0) const { auto p = s.find(c, from); return p == std::string::npos ? -1 : (int)p; }
  bool startsWith(const char* p) const { return s.rfind(p, 0) == 0; }
  void replace(const char* a, const char* b) { std::string A(a), B(b); size_t p = 0; while ((p = s.find(A, p)) != std::string::npos) { s.replace(p, A.size(), B); p += B.size(); } }
  void trim() { size_t a = s.find_first_not_of(" \t\r\n"); if (a == std::string::npos) { s.clear(); return; } size_t b = s.find_last_not_of(" \t\r\n"); s = s.substr(a, b - a + 1); }
  long toInt() const { return atol(s.c_str()); }
  String& operator+=(const String& o) { s += o.s; return *this; }
  String& operator+=(const char* o) { s += o; return *this; }
  String& operator+=(char c) { s += c; return *this; }
  bool operator==(const String& o) const { return s == o.s; }
  bool operator==(const char* o) const { return s == o; }
  bool operator!=(const char* o) const { return s != o; }
};
inline String operator+(const String& a, const String& b) { return String(a.s + b.s); }
inline String operator+(const String& a, const char* b) { return String(a.s + b); }
inline String operator+(const char* a, const String& b) { return String(std::string(a) + b.s); }
struct SerialMock {
  bool quiet = true;
  void printf(const char* f, ...) { if (quiet) return; va_list ap; va_start(ap, f); vprintf(f, ap); va_end(ap); }
  void println(const char* p = "") { if (!quiet) puts(p); }
};
extern SerialMock Serial;
