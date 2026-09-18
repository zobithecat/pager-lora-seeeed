#pragma once

#include "lora_rf.h"       // shared LoRa PHY params — single source of truth (3 repos)

// Hardware (Seeed XIAO ESP32S3 + Wio-SX1262 kit, B2B connector)
//   LoRa SX1262: SPI(SCK=7 MISO=8 MOSI=9) + NSS=41 DIO1=39 BUSY=40 RST=42
//                RF switch = SX1262 DIO2 (internal), TCXO = DIO3 @ 1.8V
//   I2C(OLED):   SDA=GPIO5 (D4), SCL=GPIO6 (D5)  ← XIAO 기본 I2C. GPIO8/9는
//                이제 LoRa SPI가 쓰므로 OLED I2C를 여기로 옮김.
//   Serial debug + 업로드: 내장 USB (USB CDC On Boot=Enabled 필수)
#define I2C_SDA_PIN   5            // XIAO D4 (SX1262가 8/9 SPI를 점유 → 여기로 이동)
#define I2C_SCL_PIN   6            // XIAO D5
#define OLED_I2C_ADDR 0x3C

// Display layout (128x64)
#define DISPLAY_W       128
#define DISPLAY_H       64
#define STATUS_BAR_H    10
#define TEXT_AREA_Y     12
#define TEXT_LINE_H     16          // unifont height
#define TEXT_LINES      3           // 3 * 16 = 48 px
#define TEXT_BUF_BYTES  512         // TX 타이핑 버퍼 한도
#define HISTORY_BUF_BYTES 8192      // RX history (여러 메시지 누적, 1MB 메시지 ~14개 분량)

// BLE
#define BLE_DEVICE_NAME  "XIAO-S3 HID Host"
#define HID_MAX_KEYS     6          // boot protocol slot count

// 이 이름과 정확히 일치하는 디바이스만 연결 시도. ""면 HID 광고하는 모든 기기.
#define TARGET_DEVICE_NAME "MINI Keyboard"

// 키 반복 (ms)
#define KEY_REPEAT_INITIAL_MS 500
#define KEY_REPEAT_RATE_MS    50

// LoRa SX1262 (Wio-SX1262, SPI). RadioLib 드라이버. DX-LR02 UART/AT 방식에서 이관.
// 핀은 XIAO ESP32S3 ↔ Wio-SX1262 B2B 커넥터 기준 (Meshtastic seeed_xiao_s3 변종과 동일).
//   NSS=41 DIO1=39 BUSY=40 RST=42, SPI(SCK=7 MISO=8 MOSI=9).
//   RF 안테나 스위치는 SX1262 내부 DIO2가 제어(setDio2AsRfSwitch), TCXO는 DIO3 @1.8V.
#define LORA_SCK_PIN         7
#define LORA_MISO_PIN        8
#define LORA_MOSI_PIN        9
#define LORA_NSS_PIN         41     // SX126X chip select
#define LORA_DIO1_PIN        39     // SX126X IRQ (RxDone/TxDone)
#define LORA_BUSY_PIN        40     // SX126X BUSY
#define LORA_RST_PIN         42     // SX126X NRST
#define LORA_RXEN_PIN        38     // RF 스위치 RX enable (수신 중 HIGH). TX 경로는 SX1262 DIO2가 제어
#define LORA_TCXO_V          1.8f   // Wio-SX1262 TCXO ref voltage (DIO3), volts
#define LORA_MAX_LINE_BYTES  60     // chunk 한 줄 최대 바이트 (UTF-8 safe) = 1 LoRa packet 본문

// ===== LoRa RF 파라미터 (lora_rf.h 공유 PHY) =====
// SX1262는 RadioLib begin()에 SF/BW/CR/freq를 직접 설정한다 (AT 모드 불필요).
// lora.cpp가 이 값으로 ToA를 계산해 송신 패킷 간 delay를 자동 산출함. 모든 노드
// (pager ↔ T-Deck ↔ Heltec relay)가 같은 값이어야 서로 들린다. lora_rf.h에서만 수정.
#define LORA_SF              RF_SF               // from lora_rf.h (shared PHY)
#define LORA_BW_HZ           RF_BW_HZ            // Bandwidth (Hz)
#define LORA_CR              RF_CR_INDEX         // 1=4/5 .. 4=4/8  (4/6 → 2)
#define LORA_HAS_CRC         (RF_CRC_ON ? 1 : 0) // 0=off, 1=on
#define LORA_PREAMBLE_SYM    RF_PREAMBLE         // 보통 8

// Heartbeat / Beacon — 자기 ID 포함해서 주기 송신, 받는 쪽은 neighbor 추적
// SF12에선 HB 1개 RF 시간 ~3초. 너무 자주 보내면 RF 채널 점유율 ↑.
#define LORA_HB_TX_MS         60000  // 60초마다 HB 송신 (§5, ttl=1 — relay 안 탐)
#define LORA_HB_TIMEOUT_MS   180000  // 3분간 아무것도 못 들으면 link down
#define LORA_NODE_MAX         16     // discovery 테이블 크기 (봉투 src 기준)

// ===== PROTOCOL.md v1.22 역할 튜닝 (lora.cpp의 #ifndef 기본값을 덮어쓴다) =====
// 이 노드는 실내용 키보드 페이저 = experiment profile (§2). 실외/무인 운용이면
// LORA_TX_DBM을 14 이하로 내릴 것 (KR920 EIRP 한도, 안테나 이득 포함).
#define LORA_TX_DBM           RF_TX_DBM
#define LORA_LBT              1      // 모든 TX 전에 CAD (non-persistent)
#define LORA_PONG_BROADCAST   0      // v1.21: 브로드캐스트 PING엔 인프라만 답한다.
                                     //        주소지정 PING(<dst>=P00)에는 항상 답함.

// 주파수는 lora_rf.h(RF_FREQ_MHZ)로 고정. SX1262는 임의 주파수 설정 가능해서
// DX-LR02의 "채널" 개념 대신 MHz 단위로 직접 튜닝한다. 채널 개념이 필요 없어짐.

// Sender ID. byte 단위 (한글 1자 = 3바이트). 영문 24자 또는 한글 8자.
#define MAX_SENDER_ID_BYTES 24

// Relay-layer node id (role P + 1-byte hex). Mesh address used for dedup/forwarding
// — distinct from the user's display sender_id. See t-deck-os/RELAY_PROTOCOL.md.
#define NODE_ID "P00"
#define NVS_NAMESPACE "pager"

// Debug
#define DEBUG_SERIAL 1
#if DEBUG_SERIAL
  #define LOGF(...) Serial.printf(__VA_ARGS__)
#else
  #define LOGF(...) do { } while (0)
#endif
