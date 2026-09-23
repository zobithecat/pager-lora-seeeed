// GPIO 출력 생존 검사: 지정 핀들을 3초 HIGH / 3초 LOW로 번갈아 구동하며 Serial에 찍는다.
// 멀티미터로 해당 패드(또는 그 패드가 이어진 홀)–GND 전압이 3.3 V ↔ 0 V로 바뀌는지 본다.
#include <Arduino.h>
static const int pins[] = {9 /*D10 MOSI*/, 7 /*D8 SCK*/, 44 /*D7*/};
void setup() { Serial.begin(115200); delay(1000); for (int p : pins) pinMode(p, OUTPUT); }
void loop() {
  for (int lvl = 1; lvl >= 0; lvl--) {
    for (int p : pins) digitalWrite(p, lvl);
    Serial.printf("GPIO9(D10/MOSI) GPIO7(D8/SCK) GPIO44(D7) = %s  (multimeter: expect %s)\n", lvl ? "HIGH" : "LOW", lvl ? "3.3 V" : "0 V");
    delay(3000);
  }
}
