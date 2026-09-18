#pragma once
#include <Arduino.h>

// BLE 대시보드 링크 — 우리가 Peripheral, 폰(Bluefy의 Web Bluetooth)이 Central.
// Nordic UART Service 위에 줄 단위 프로토콜을 얹는다:
//   장치 → 폰 (TX notify) : JSON 한 줄 + '\n'.  MTU에 맞춰 조각내 보내고 폰이 '\n'으로 재조립
//   폰 → 장치 (RX write)  : 텍스트 명령 한 줄 + '\n'   (GET / MSG <text> / ID <name> / BCN)
//
// NimBLE 콜백은 NimBLE 태스크에서 돌기 때문에 여기서는 큐에만 넣는다. 명령 처리와
// 송신은 전부 메인 루프(ble_dash_tick / ble_dash_poll_command)에서 한다.

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // 폰 → 장치 (write)
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // 장치 → 폰 (notify)

void ble_dash_begin();
void ble_dash_tick();                          // main loop에서 매번: 송신 FIFO를 조금씩 비운다
bool ble_dash_ready();                         // 연결됨 + 폰이 notify 구독함
bool ble_dash_consume_just_ready();            // 방금 구독이 시작됐으면 1회 true (→ 전체 상태 재생)
bool ble_dash_poll_command(String* out);       // 폰이 보낸 명령 한 줄 (개행 제외)
bool ble_dash_send_line(const String& json);   // 구독자 없거나 FIFO가 차면 false (그 줄은 버림)
void ble_dash_set_slow_adv(bool slow);         // 주차 중엔 광고 간격을 늘려 전류를 아낀다

// JSON 문자열 값 escape (따옴표는 호출자가 두른다). UTF-8은 그대로 통과.
String json_escape(const String& s);
