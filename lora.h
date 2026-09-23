#pragma once
#include <Arduino.h>

// LoRa SX1262 모듈 (Wio-SX1262, SPI + RadioLib) — mesh PROTOCOL.md v1.22 엔드포인트 스택.
// 스펙 원본: ../gopher-over-lora/lora/PROTOCOL.md  (봉투/PHY는 relay.h / lora_rf.h)
//
// ★ 이 파일과 lora.cpp는 키보드 페이저(루트 스케치)와 차량 페이저(pager-vehicle/)가
//   byte-identical로 공유한다. 역할 차이는 전부 각 스케치의 config.h 매크로와 콜백으로
//   표현한다. 한쪽을 고치면 다른 쪽에 복사할 것 (README "공유 파일" 참고).
//
// 전송 단위: 프로토콜 한 줄 = LoRa 패킷 1개, 전부 R| 봉투로 감싼다 (§4).
//   L0  HB / PING / PONG                      — 스택이 소비 (§5)
//   L1  "!<TYPE>\t…"                          — 시스템 라인. 채팅에 절대 안 섞임
//   L2  [SOF] → 청크(≤60B, "!"는 "!!"로 escape) → [EOF]   — 사용자 텍스트
//
// 메인 루프에서 lora_tick()을 자주 호출해야 수신/HB/예약 송신이 동작한다.
// 모든 콜백은 lora_tick() 컨텍스트(= 메인 루프)에서 불린다.

// 수신 프레임의 봉투/링크 메타.
struct LoraRxInfo {
  char    src[4];     // 봉투 src (3자 routing id) — origin이지 forwarder가 아니다 (§4)
  int     rssi;       // dBm. hops != 0 이면 이건 마지막 relay의 RSSI다 (§5 !RB 주석)
  float   snr;        // dB
  uint8_t ttl;        // 수신 시점의 raw ttl (§10: hops는 해석, ttl이 사실)
  int8_t  hops;       // 0=direct, 1, 2, -1=unknown (ttl=3으로 출발했다고 못 믿는 타입)
  bool    partial;    // L2 전용: [EOF] 없이 idle 만료/강제 종료된 메시지 (§5 v1.19)
};

typedef void (*LoraRxMsgCb)(const String& text, const LoraRxInfo& info);
typedef void (*LoraConnStateCb)(bool connected);
// 모든 L1 라인이 (스택 자체 처리 후) 여기로도 온다. type은 '!' 뺀 토큰 ("CAR", "AL" …).
typedef void (*LoraL1Cb)(const String& type, const String& args, const LoraRxInfo& info);

// hw_reset=false: 딥슬립에서 DIO1로 깬 직후처럼 SX1262가 이미 설정된 채 살아 있을 때. 칩을 리셋하지
// 않고 재초기화하며, 수신 버퍼에 남아 있는(우리를 깨운) 패킷이 있으면 먼저 읽어 콜백으로 넘긴다.
// → 콜백(lora_set_callbacks 등)은 lora_begin 전에 등록해 둘 것.
void lora_begin(bool hw_reset = true);
void lora_tick();                                      // main loop에서 매번 호출
void lora_send_message(const String& text);            // L2. 비동기: 큐에 넣고 즉시 반환
bool lora_tx_consume_done();                            // 큐 송신 완료 시 1회 true (main loop용)
bool lora_consume_range(int* out_seq, uint32_t* out_count);  // 주소지정 PING 응답 시 1회 true
void lora_set_callbacks(LoraRxMsgCb on_msg, LoraConnStateCb on_conn);
void lora_set_l1_callback(LoraL1Cb on_l1);
bool lora_connected();

// L1 한 줄 송신 (예약). lora_tick의 idle 지점에서 LBT 후 나간다. line은 '!'로 시작, ≤60B.
//   max_defer_ms == 0 : 요청/응답류 — §8a 양보 대상 아님, 바로 나감
//   max_defer_ms  > 0 : beacon-class — 스트림이 채널을 예약 중이면 양보하되, 이 시간
//                       (= 자기 주기 1회, §8a MUST)을 넘기면 그냥 보낸다
// 큐가 찼거나 줄이 규격 밖이면 false.
bool lora_send_l1(const String& line, uint8_t ttl, uint32_t max_defer_ms);

// ===== Discovery — 들리는 모든 노드 =====
// HB뿐 아니라 유효한 봉투를 하나라도 들으면 src 기준으로 테이블에 올린다. 단 relay는
// 자기 id로 아무것도 안 쏘므로(!RS opt-in 제외) 여기 안 보이는 게 정상이다 (§1).
struct LoraNode {
  bool     valid;
  char     id[4];            // 봉투 src
  String   name;             // HB의 display id (없으면 "")
  String   last_type;        // 마지막 프레임 종류: "HB" "PING" "PONG" "TEXT" 또는 L1 type
  String   info_type;        // 마지막 상태성 L1 type (RB/RS/CAR/FS/CS) — info의 해석 키
  String   info;             // 그 L1의 args 원문 (탭 구분, ≤64B)
  int      rssi;             // 마지막 DIRECT 프레임의 RSSI (relay 경유분은 반영 안 함)
  float    snr;
  bool     rssi_valid;
  int8_t   hops;             // 마지막 프레임의 hops (-1 unknown)
  int      peer_rssi;        // 상대가 HB에 실어 보낸 "자기 쪽 수신 RSSI"
  bool     peer_rssi_valid;
  uint32_t last_seen_ms;
  uint32_t frames;
};
int  lora_nodes_snapshot(LoraNode* out, int max);      // 유효 노드 복사, 개수 반환
int  lora_nodes_count();
bool lora_nodes_consume_changed();                     // 테이블이 바뀌었으면 1회 true

struct LoraStats {
  uint32_t rx_ok, rx_dup, rx_own, rx_bad;   // bad = CRC/grammar/binary 아닌 쓰레기
  uint32_t tx_frames, lbt_defers;
  int      last_rssi; bool last_rssi_valid;
  int      radio_status;                     // radio.begin() 결과. 0=OK, -2=SX1262 응답 없음(미장착/접촉불량)
  uint32_t resumed_len;                      // lora_begin(false)이 RX 버퍼에서 건져 처리한 패킷 길이 (0=없음)
};
void lora_get_stats(LoraStats* out);

// ===== 딥슬립 협조 =====
// 송신할 게 하나도 없고(L1 큐·PONG·L2 큐·진행 중 프레임) 라디오가 한가하면 true.
bool lora_idle();
// HB 송신 on/off. 딥슬립 노드가 비콘 창/남의 프레임으로 잠깐 깼을 때 매번 HB를 내면 채널만 먹고
// 이웃 테이블엔 아무 정보도 안 준다 → 잠깐 깬 동안엔 끈다 (존재 확인은 !CAR가 맡는다).
void lora_set_hb_enabled(bool on);
// 라디오를 RX 듀티사이클(프리앰블 감지 창만 주기적으로 여는 저전력 수신)로 두고 라디오 뮤텍스를
// 잡은 채 반환한다 — 호출 직후 esp_deep_sleep_start() 할 것. 패킷이 오면 DIO1이 High가 되므로
// LORA_DIO1_PIN을 ext0 wake 소스로 쓴다 (RTC GPIO여야 함). 실패 시 false(라디오는 일반 RX).
bool lora_sleep_arm();
int  lora_dio1_pin();

// SX1262는 주파수 고정(lora_rf.h). DX-LR02 채널 개념 없음 — 호출 시 안내만 출력.
// .ino의 'Cnn' Serial 명령 호환용으로 시그니처만 유지.
void lora_set_channel(uint8_t channel);

// 진단: 컴파일된 RF 설정(freq/SF/BW/CR/CRC/power) + SX1262 핀 매핑 dump.
void lora_probe_at();

// 진단: SX1262엔 UART transparent 브리지가 없음 — 안내 출력 후 즉시 반환.
void lora_bridge_loop();

// 진단: RF 설정 dump (lora_probe_at와 동일). 예전 AT+HELP 대체.
void lora_query_at_help();

// 진단: 마지막으로 수신한 RF 패킷의 RSSI/SNR 출력 (radio.getRSSI/getSNR).
// 양수면 saturation 의심 (너무 가까움), -50 ~ -90 정상, -110+ 약함.
void lora_query_rssi();

// HB에 실을 display id (cosmetic — 라우팅/dedup은 NODE_ID만 쓴다, §3).
void lora_set_my_id(const String& id);

// 노드 테이블 + dedup 링 통계 Serial 출력.
void lora_dump_neighbors();

// 진단 주입: 봉투째("R|src|pktid|ttl|payload") 수신 경로에 넣는다 — 실제 RF 없이 dedup·분류·콜백까지
// 전부 탄다. 벤치에서 주소지정 PING/채팅 수신 동작을 확인할 때 쓴다.
void lora_test_inject(const String& envelope_line);
