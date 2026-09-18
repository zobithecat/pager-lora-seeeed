#pragma once
#include <Arduino.h>
#include <stdint.h>

// BLE HID Host (Central). NimBLE-Arduino v2 사용.
//
// 모델:
//   - 0x1812 (HID over GATT) 광고하는 모든 디바이스에 자동 연결 시도.
//   - 연결되면 HID service의 notify-capable input characteristic 전부에 구독.
//   - 첫 8바이트를 boot keyboard report로 가정해 (modifier, _, k1..k6) 콜백 호출.
//   - Battery Service (0x180F)의 0x2A19 값을 읽고 notify 가능하면 구독.
//   - 연결 끊기면 자동 재스캔.

typedef void (*HidReportCb)(uint8_t modifiers, const uint8_t* keys /* 6 bytes */);
typedef void (*BleStateCb)(int /* BleVisualState as int */);
typedef void (*BatteryCb)(uint8_t pct);

void ble_host_begin(HidReportCb on_report, BleStateCb on_state, BatteryCb on_batt);
void ble_host_loop();

// 저장된 모든 BLE bond/페어링 정보 삭제하고 깨끗한 상태에서 재스캔.
// 키보드와 페어링이 꼬였을 때 사용.
void ble_host_clear_bonds_and_restart();
