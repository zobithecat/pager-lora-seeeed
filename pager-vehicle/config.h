#pragma once

#include "lora_rf.h"       // shared LoRa PHY params — single source of truth (byte-identical 전 리포)

// ===== 차량 페이저 (P01) =====
// Hardware: Seeed XIAO ESP32S3 + Wio-SX1262 kit (B2B) + BME280 (I2C) + LiPo (XIAO BAT 패드)
//   LoRa SX1262: SPI(SCK=7 MISO=8 MOSI=9) + 헤더 경로 NSS=4 DIO1=1 BUSY=2 RST=3 RXEN=5 (아래 참고)
//                RF switch = SX1262 DIO2 (internal), TCXO = DIO3 @ 1.8V
//   I2C(센서):   SDA=GPIO43 (D6), SCL=GPIO44 (D7)  — SHT35 @0x45 확인(2026-09-23)
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
#define I2C_SDA_PIN        43        // D6 (GPIO43)  ← Wio 헤더 관통홀 D6 (미연결 홀, 사용 가능)
#define I2C_SCL_PIN        44        // D7 (GPIO44)  ← Wio 헤더 관통홀 D7
// D5(GPIO6)는 셋 중 유일한 ADC 핀이라 배터리 분압용으로 비워둔다. XIAO 기본 I2C(D4/D5)는 못 쓴다:
// D4는 스택 상태에서 라디오 RF_SW 네트다(아래 참고).
#define BME280_ADDR        0x76      // 못 찾으면 0x77도 자동 시도
// 센서 자동 인식 순서: SHT3x(0x44/0x45) → BME280/BMP280(0x76/0x77) → BMP390/388. BMP3를 SPI로
// 읽는 경로(CSB/SDO GPIO)는 남겨두되 이 스택에선 쓸 핀이 없어 -1.
// ★ 2026-09-23 스키매틱 확인: Wio-SX1262(B2B) 헤더 홀의 DIO1/BUSY/RST/NSS/RF-SW는 라디오 신호와
//   같은 네트다. 스택 상태에서 XIAO D0~D4(GPIO1,2,3,4,5)는 전부 라디오 라인 → 절대 쓰지 말 것.
//   센서/ADC용으로 남는 핀은 D5(GPIO6)·D6(GPIO43)·D7(GPIO44)뿐.
#define VEH_ENV_CSB_PIN    (-1)      // BMP3 SPI 경로 비활성 (D1/D3이 라디오 BUSY/NSS였다)
#define VEH_ENV_SDO_PIN    (-1)
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
//     BAT+ ──[R1 2.3M]──┬──[R2 2.3M]── GND
//                       └──┬── D5 (GPIO6, ADC1_CH5)   (상시 소모 ~1 µA)
//                         ═╪═ C 100 nF~1 µF → GND     ← 필수: 소스 임피던스 1.15 MΩ은 ESP32 SAR ADC의
//                                                       샘플 커패시터를 못 채운다. 콘덴서가 샘플 전하를 대준다.
//   R1 = R2면 DIVIDER 2.0. 다른 값을 쓰면 (R1+R2)/R2. 실측과 다르면 VEH_VBAT_CAL로 보정.
// 배선이 없으면 핀이 떠서 엉뚱한 값이 읽히므로, 2.5–4.5 V를 벗어난 값은 "측정 불가"로 버린다
// → 저항을 달기 전에도 이 설정 그대로 둬도 된다. 기능을 아예 끄려면 -1.
#define VEH_VBAT_ADC_PIN    6        // D5 (GPIO6). 배선 전엔 값이 2.5–4.5 V 범위 밖이라 "측정 불가"로 나온다
#define VEH_VBAT_DIVIDER    2.0f
#define VEH_VBAT_CAL        1.000f   // 멀티미터 실측 / 표시값. 저항 오차(±1–5 %) 보정용
#define VEH_BATT_LOW_PCT    15       // 주차 중 이 이하로 떨어지면 비콘에 'L' 플래그 + 즉시 1발

// ----- 주차 중 딥슬립 -----
// 주차 후 VEH_AWAKE_MS 동안은 완전히 깨어 있다(폰 연결·채팅·디스커버리 전부). 그 뒤엔:
//   딥슬립 → VEH_SLEEP_WAKE_S 마다 타이머로 깨서 센서 측정 + !CAR 비콘 + 짧은 BLE 광고 창 → 다시 슬립.
//   슬립 중에도 SX1262는 RX 듀티사이클로 듣고 있다가 패킷이 오면 DIO1로 ESP32를 깨운다(ext0).
//   깨우는 신호(30분 다시 깨어 있음): 우리 앞으로 온 PING(<dst>=P01), 채팅 메시지 수신, 폰 BLE 연결.
//   그 외 프레임(남의 HB/!RB 등)으로 깼으면 VEH_WAKE_SHORT_MS 뒤 다시 잔다.
// 슬립 중 평균 ~2 mA(듀티사이클 RX 포함) + 3분마다 ~15초 각성 → 1000 mAh로 1주 이상.
// 대가: 슬립 중엔 폰이 바로 못 붙는다(다음 타이머 창까지 최대 VEH_SLEEP_WAKE_S 대기, 또는 T-Deck에서
// 주소지정 PING을 쏘면 즉시 30분 각성). 받은 채팅 히스토리(RAM)는 슬립하면 사라진다.
// 요구: LORA_DIO1_PIN이 RTC GPIO(ESP32-S3: GPIO0~21)여야 한다 — 헤더 경로(GPIO1)는 OK, B2B(39)는 불가.
#define VEH_SLEEP_ENABLE       1
#define VEH_SLEEP_WAKE_S       180       // 타이머 wake 주기 = 슬립 중 비콘 주기 (PROTOCOL_CAR: ≥ 60 s)
#define VEH_AWAKE_MS           1800000UL // 주차 직후 / 깨우는 신호 뒤 각성 시간 (30분)
#define VEH_WAKE_WINDOW_MS     15000UL   // 타이머 wake 창: 비콘 + BLE 광고 + USB(주행) 재감지 여유
#define VEH_WAKE_SHORT_MS      25000UL   // 남의 프레임으로 깼을 때: 이어지는 청크/응답을 받을 시간
#define VEH_BLE_LINGER_MS      60000UL   // 폰이 끊긴 뒤 이만큼은 더 깨어 있는다(재연결 여유)

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
// ★ 이 개체는 XIAO와 Wio-SX1262를 2.54 mm 핀헤더로 납땜해 스택했고, 그 과정에서 B2B 커넥터가
//   떨어졌다(2026-09-23 프로브: B2B BUSY floating, 헤더 BUSY driven). 라디오 제어선은 모듈 헤더
//   홀(DIO1/BUSY/RST/NSS/RF-SW) = XIAO D0~D4로 들어온다. SPI(7/8/9)는 두 경로가 같다.
//   B2B가 제대로 물린 정상 키트라면 41/39/40/42/38 (Meshtastic seeed_xiao_s3)로 되돌릴 것.
#define LORA_NSS_PIN         4      // D3 = NSS
#define LORA_DIO1_PIN        1      // D0 = DIO1
#define LORA_BUSY_PIN        2      // D1 = BUSY
#define LORA_RST_PIN         3      // D2 = RST (ESP32-S3 스트래핑 핀이지만 부팅 후 출력으로 쓰는 건 무방)
#define LORA_RXEN_PIN        5      // D4 = RF_SW (수신 중 HIGH). TX 경로는 SX1262 DIO2가 제어
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
