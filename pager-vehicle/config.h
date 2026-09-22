#pragma once

#include "lora_rf.h"       // shared LoRa PHY params — single source of truth (byte-identical 전 리포)

// ===== 차량 페이저 (P01) =====
// Hardware: Seeed XIAO ESP32S3 + Wio-SX1262 kit (B2B) + BME280 (I2C) + LiPo (XIAO BAT 패드)
//   LoRa SX1262: SPI(SCK=7 MISO=8 MOSI=9) + NSS=41 DIO1=39 BUSY=40 RST=42
//                RF switch = SX1262 DIO2 (internal), TCXO = DIO3 @ 1.8V
//   I2C(센서):   SDA=GPIO5 (D4), SCL=GPIO44 (D7)  ← D5/D6는 이 보드에서 불량
//   전원: 주행 중 = USB 5V, 주차 중 = LiPo. XIAO가 USB 있을 때 LiPo를 충전한다.
//   OLED/키보드 없음 — UI는 폰의 Bluefy 웹 대시보드(BLE) 하나뿐.

// Relay-layer node id (role P + 1-byte hex). 봉투 src / dedup identity (PROTOCOL §3).
// P00 = 키보드 페이저, P01 = 이 차량 페이저. 같은 id가 둘 떠 있으면 서로를 자기 echo로
// 버리므로 반드시 유일해야 한다.
#define NODE_ID            "P01"
#define NVS_NAMESPACE      "pagerveh"
#define DEFAULT_DISPLAY_ID "CAR01"   // HB/채팅 prefix에 쓰는 표시 이름 (대시보드에서 변경 → NVS)
#define MAX_DISPLAY_ID_BYTES 24
#define FW_VERSION         "veh-1.0"

// ----- I2C / BME280 -----
#define I2C_SDA_PIN        5         // D4
// SCL은 XIAO 기본 D5(GPIO6)가 아니라 D7(GPIO44)다. 이 개체는 D5(GPIO6)와 D6(GPIO43)이 아무
// 배선 없이도 Low로 잡혀 있었다(2026-09-22 핀 프로브 실측 — 브리지 또는 핀 손상). D5/D6는 안 쓴다.
#define I2C_SCL_PIN        44        // D7
#define BME280_ADDR        0x76      // 못 찾으면 0x77도 자동 시도
// BMP390/388(CJMCU-390)의 CSB/SDO도 GPIO에 있다 → 센서를 SPI(비트뱅)로 읽는다:
//   CS=CSB(GPIO2)  SCK=I2C_SCL_PIN  MOSI=I2C_SDA_PIN  MISO=SDO(GPIO4).
// I2C로는 이 보드가 끝내 응답하지 않았고 SPI로는 바로 잡혔다(2026-09-22). BME280은 여전히
// 같은 SDA/SCL에서 I2C로 먼저 탐색한다. 아래 두 핀이 -1이면 BMP3도 I2C(0x76/0x77)로 시도.
// GPIO3(D2)은 ESP32-S3 스트래핑 핀이라 피했다. -1이면 그 핀은 안 건드린다(외부 점퍼로 처리).
#define VEH_ENV_CSB_PIN    2         // D1
#define VEH_ENV_SDO_PIN    4         // D3
#define VEH_ENV_SDO_LEVEL  LOW       // → 0x76
#define VEH_SENSOR_MS      10000UL   // forced-mode 측정 주기 (자체 발열 최소화)
// 주차된 차 실내는 한여름 70°C를 넘는다. LiPo는 60°C 위에서 위험하므로 비콘/대시보드에
// 과열 플래그를 세운다. (센서가 본체 근처에 있으면 이 값이 곧 배터리 주변 온도다.)
#define VEH_TEMP_WARN_C    60.0f

// ----- 주차 감지 -----
// 추가 배선 없이 "USB 호스트가 이 장치를 열거(enumerate)했는가"로만 판단한다.
//   호스트 있음 = 주행(D), 없음 = 주차(P, LiPo 구동)
// ★ 데이터선이 없는 단순 USB 충전기에 꽂으면 항상 "주차"로 보인다. 그 경우 5V 핀을
//   100k/100k로 분압해 GPIO에 넣고 VEH_VBUS_SENSE_PIN에 그 핀 번호를 주면 된다.
#define VEH_VBUS_SENSE_PIN  (-1)     // -1 = 미사용 (USB 호스트 감지만)
#define VEH_POWER_DEBOUNCE_MS 10000UL // 시동 크랭킹 중 순간 끊김에 흔들리지 않게
// 배터리 전압: XIAO ESP32S3는 BAT가 ADC에 안 물려 있어서 분압 배선이 있어야 읽힌다.
//     BAT+ ──[R1 200k]──┬──[R2 200k]── GND
//                       └── D0 / A0 (GPIO1)          (Seeed 위키 권장 회로. 상시 소모 ~10 µA)
//   R1 = R2면 DIVIDER 2.0. 다른 값을 쓰면 (R1+R2)/R2. 잡음이 크면 A0–GND에 100 nF.
// 배선이 없으면 핀이 떠서 엉뚱한 값이 읽히므로, 2.5–4.5 V를 벗어난 값은 "측정 불가"로 버린다
// → 저항을 달기 전에도 이 설정 그대로 둬도 된다. 기능을 아예 끄려면 -1.
#define VEH_VBAT_ADC_PIN    1
#define VEH_VBAT_DIVIDER    2.0f
#define VEH_VBAT_CAL        1.000f   // 멀티미터 실측 / 표시값. 저항 오차(±1–5 %) 보정용
#define VEH_BATT_LOW_PCT    15       // 주차 중 이 이하로 떨어지면 비콘에 'L' 플래그 + 즉시 1발

// ----- 상태 비콘 (!CAR, PROTOCOL_CAR.md) -----
#define VEH_BEACON_PARKED_MS   300000UL  // 주차 중 5분마다. §8a beacon-class → 스트림에 양보
#define VEH_BEACON_DRIVING_MS  0UL       // 주행 중엔 끔 (0). HB는 역할과 무관하게 계속 나간다
#define VEH_BEACON_TTL         3         // = RELAY_TTL_MESH. 주차장 → 건물 안까지 relay를 타야 의미가 있다
#define VEH_PARKED_CPU_MHZ     80        // 주차 중 CPU 클럭 (LiPo 절약). BLE/LoRa는 80MHz로 충분

// ----- BLE 대시보드 (폰이 Central, 우리가 Peripheral — Nordic UART Service) -----
#define BLE_DEVICE_NAME    "PAGER-" NODE_ID
// 0 = 페어링 없이 연결. 6자리 값을 주면 폰→장치 쓰기(= LoRa 송신 권한)에 passkey 본딩을
// 요구한다. 주차된 차 옆을 지나가는 사람이 이 노드 이름으로 메시에 글을 쓰는 걸 막는다.
#define VEH_BLE_PASSKEY    0
#define VEH_CHAT_HISTORY   16        // 폰이 없을 때 받은 메시지를 RAM에 보관했다가 연결 시 재생

// ----- LoRa SX1262 -----
#define LORA_SCK_PIN         7
#define LORA_MISO_PIN        8
#define LORA_MOSI_PIN        9
#define LORA_NSS_PIN         41
#define LORA_DIO1_PIN        39
#define LORA_BUSY_PIN        40
#define LORA_RST_PIN         42
#define LORA_RXEN_PIN        38     // RF 스위치 RX enable (수신 중 HIGH). TX 경로는 SX1262 DIO2가 제어
#define LORA_TCXO_V          1.8f
#define LORA_MAX_LINE_BYTES  60     // 프로토콜 한 줄 = 1 LoRa 패킷 본문 상한 (§5)

#define LORA_SF              RF_SF
#define LORA_BW_HZ           RF_BW_HZ
#define LORA_CR              RF_CR_INDEX
#define LORA_HAS_CRC         (RF_CRC_ON ? 1 : 0)
#define LORA_PREAMBLE_SYM    RF_PREAMBLE

#define LORA_HB_TX_MS         60000
#define LORA_HB_TIMEOUT_MS   180000
#define LORA_NODE_MAX         16

// PROTOCOL §2 regulatory: 이 노드는 실외·무인으로 돈다 → legal profile.
//   EIRP ≤ 14 dBm (안테나 이득 포함 — 이득 큰 안테나를 달면 더 내릴 것), 모든 TX 전에 LBT.
//   주파수는 플릿과 들리려면 lora_rf.h(922.0)를 따를 수밖에 없다 — README 참고.
#define LORA_TX_DBM           14
#define LORA_LBT              1
#define LORA_PONG_BROADCAST   0      // v1.21: 페이저는 주소지정 PING(<dst>=P01)에만 답한다

// Debug
#define DEBUG_SERIAL 1
#if DEBUG_SERIAL
  #define LOGF(...) Serial.printf(__VA_ARGS__)
#else
  #define LOGF(...) do { } while (0)
#endif
