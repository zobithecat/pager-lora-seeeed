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
  // MOSI 검증: SetStandby(STDBY_XOSC) → 상태 mode 비트(6:4)가 010(RC) → 011(XOSC)으로 바뀌어야 한다.
  // MOSI가 끊기면 칩은 NOP만 받아 상태를 계속 돌려주되 mode는 안 바뀐다.
  digitalWrite(p.cs, LOW); SPI.transfer(0x80); SPI.transfer(0x01); digitalWrite(p.cs, HIGH);
  waitBusyLow(p.busy, 50); delay(2);
  digitalWrite(p.cs, LOW); SPI.transfer(0xC0); uint8_t st2 = SPI.transfer(0x00); digitalWrite(p.cs, HIGH);
  waitBusyLow(p.busy, 50);
  Serial.printf("  MOSI test: mode before=%u after SetStandby(XOSC)=%u -> %s\n", (st >> 4) & 7, (st2 >> 4) & 7, ((st2 >> 4) & 7) == 3 ? "MOSI OK" : "MOSI NOT REACHING CHIP");
  uint8_t ver[17] = {0};
  digitalWrite(p.cs, LOW); SPI.transfer(0x1D); SPI.transfer(0x03); SPI.transfer(0x20); SPI.transfer(0x00);
  for (int i = 0; i < 16; i++) ver[i] = SPI.transfer(0x00);
  digitalWrite(p.cs, HIGH); SPI.endTransaction();
  Serial.printf("  GetStatus=0x%02X  (0x00/0xFF/0x7F = no chip; 0x22/0x2A/0xA2 = alive)\n", st);
  Serial.print("  version: \""); for (int i = 0; i < 16; i++) Serial.print((ver[i] >= 32 && ver[i] < 127) ? (char)ver[i] : '.'); Serial.println("\"");
  pinMode(p.rst, INPUT); pinMode(p.cs, INPUT);
}
void setup() { Serial.begin(115200); delay(1500); SPI.begin(7, 8, 9, -1); }
void loop() {
  Serial.println("\n######## SX1262 probe ########");
  for (auto& p : kSets) probe(p);
  delay(4000);
}
