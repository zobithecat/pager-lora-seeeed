// SX1262 raw SPI probe — RadioLib 없이 칩 응답만 본다. Wio-SX1262 B2B 경로와 헤더 경로 둘 다 시도.
#include <Arduino.h>
#include <SPI.h>
struct Pins { const char* name; int cs, dio1, busy, rst; };
static const Pins kSets[] = {
  {"B2B    (cs41 dio1_39 busy40 rst42)", 41, 39, 40, 42},
  {"HEADER (cs4=D3 dio1_1=D0 busy2=D1 rst3=D2)", 4, 1, 2, 3},
};
static bool waitBusyLow(int busy, uint32_t ms) { uint32_t t = millis(); while (digitalRead(busy)) { if (millis() - t > ms) return false; } return true; }
static void probe(const Pins& p) {
  Serial.printf("\n== %s\n", p.name);
  pinMode(p.cs, OUTPUT); digitalWrite(p.cs, HIGH);
  pinMode(p.busy, INPUT_PULLUP);  int bu = digitalRead(p.busy);
  pinMode(p.busy, INPUT_PULLDOWN); int bd = digitalRead(p.busy);
  pinMode(p.busy, INPUT);
  Serial.printf("  BUSY pullup=%d pulldown=%d (%s)\n", bu, bd, bu != bd ? "FLOATING - not connected" : "driven");
  pinMode(p.rst, OUTPUT); digitalWrite(p.rst, LOW); delay(5); digitalWrite(p.rst, HIGH); delay(10);
  bool ok = waitBusyLow(p.busy, 100);
  Serial.printf("  after reset: BUSY went low=%d\n", ok);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(p.cs, LOW); SPI.transfer(0xC0); uint8_t st = SPI.transfer(0x00); digitalWrite(p.cs, HIGH);
  waitBusyLow(p.busy, 50);
  uint8_t ver[17] = {0};
  digitalWrite(p.cs, LOW); SPI.transfer(0x1D); SPI.transfer(0x03); SPI.transfer(0x20); SPI.transfer(0x00);
  for (int i = 0; i < 16; i++) ver[i] = SPI.transfer(0x00);
  digitalWrite(p.cs, HIGH); SPI.endTransaction();
  Serial.printf("  GetStatus=0x%02X  (0x00/0xFF/0x7F = no chip; 0x22/0x2A/0xA2 = alive)\n", st);
  Serial.print("  version: \""); for (int i = 0; i < 16; i++) Serial.print((ver[i] >= 32 && ver[i] < 127) ? (char)ver[i] : '.'); Serial.print("\"  hex:");
  for (int i = 0; i < 16; i++) Serial.printf(" %02X", ver[i]); Serial.println();
  // 확실한 MOSI 검사: ReadRegister 0x0740 (LoRa sync word MSB, 리셋 기본값 0x14).
  //   0x14 → 명령이 칩에 들어감(MOSI OK). 상태 바이트(0xA?/0x2?)만 반복 → 칩이 NOP만 받음 = MOSI 끊김.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  waitBusyLow(p.busy, 50);
  digitalWrite(p.cs, LOW); SPI.transfer(0x1D); SPI.transfer(0x07); SPI.transfer(0x40); uint8_t s0 = SPI.transfer(0x00); uint8_t sw = SPI.transfer(0x00); digitalWrite(p.cs, HIGH);
  SPI.endTransaction();
  Serial.printf("  reg 0x0740 = 0x%02X (status byte 0x%02X)  -> %s\n", sw, s0, sw == 0x14 ? "MOSI OK (cmd reached chip)" : "cmd NOT reaching chip (MOSI/SCK/NSS)");
  pinMode(p.rst, INPUT); pinMode(p.cs, INPUT);
}
void setup() { Serial.begin(115200); delay(1500); SPI.begin(7, 8, 9, -1); }
void loop() {
  Serial.println("\n######## SX1262 probe ########");
  for (auto& p : kSets) probe(p);
  delay(4000);
}
