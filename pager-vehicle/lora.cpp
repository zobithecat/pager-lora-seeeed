#include "lora.h"
#include "config.h"
#include <RadioLib.h>
#include <SPI.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "relay.h"          // shared relay layer (NODE_ID comes from config.h)

// ===== 역할별 튜닝 (각 스케치의 config.h가 덮어쓴다) =====
#ifndef LORA_TX_DBM
#define LORA_TX_DBM          RF_TX_DBM   // "a node may lower it" (§2). 실외 노드는 ≤14
#endif
#ifndef LORA_LBT
#define LORA_LBT             1           // §2 legal profile: 모든 TX 전에 CAD (non-persistent)
#endif
#ifndef LORA_PONG_BROADCAST
#define LORA_PONG_BROADCAST  0           // §10 v1.21: 브로드캐스트 PING엔 인프라만 답한다
#endif
#ifndef LORA_FRAME_IDLE_MS
#define LORA_FRAME_IDLE_MS   20000UL     // §5 v1.19: 열린 L2 프레임 idle 만료 (권장 20–30 s)
#endif
#ifndef LORA_RX_FRAMES
#define LORA_RX_FRAMES       4           // 동시에 열어둘 수 있는 송신자별 L2 프레임 수
#endif
#ifndef LORA_NODE_MAX
#define LORA_NODE_MAX        16
#endif
#ifndef LORA_NODE_STALE_MS
#define LORA_NODE_STALE_MS   900000UL    // 15분 무소식이면 discovery 테이블에서 제거
#endif
#define LORA_RX_MSG_MAX_BYTES 2048       // 한 L2 메시지 재조립 상한 (폭주 방어)
#define L1_TXQ_N              4
#define LBT_BACKOFF_MIN_MS    15         // heltec-relay와 같은 non-persistent 백오프
#define LBT_BACKOFF_SPAN_MS   85
#define LBT_MAX_DEFERS        20         // CAD가 계속 busy(또는 고장)여도 결국은 보낸다

// ===== Radio (Wio-SX1262, SPI + RadioLib) =====
// PHY params (freq/SF/BW/CR/sync/CRC) come from lora_rf.h and MUST match every node.
static SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

// DIO1 fires on RxDone — and also on TxDone / CadDone during our own transmit or LBT.
// radio_start_rx() clears the flag after every TX/CAD so those edges are never mistaken
// for a received packet. The ISR only touches this flag — no SPI from interrupt context.
// All radio SPI access is serialized by s_radio_mtx.
static volatile bool s_rx_flag = false;
#if defined(ESP32)
IRAM_ATTR
#endif
static void lora_on_dio1() { s_rx_flag = true; }

// ===== Async TX =====
// An L2 send ([SOF]/body/[EOF] + per-packet pacing) blocks for seconds. It runs in its OWN
// task holding s_radio_mtx for the WHOLE frame; lora_tick try-locks and skips while it is
// held. That lock is also what implements §5 v1.19 "a frame is not reentrant": HB, PONG
// and queued L1 lines all leave from lora_tick, so none of them can land between our own
// [SOF] and [EOF].
struct LoraTxMsg { char text[560]; };
static SemaphoreHandle_t s_radio_mtx = nullptr;  // guards all radio SPI access
static QueueHandle_t     s_tx_q      = nullptr;  // main loop → TX task
static volatile bool     s_tx_done   = false;    // TX task finished one send
static void do_send_blocking(const char* text);
static void lora_tx_task(void*);

// ===== LoRa ToA (Time on Air) — Semtech AN1200.13 =====
static uint32_t lora_toa_ms(uint8_t sf, uint32_t bw_hz, uint8_t cr,
                            uint16_t payload_bytes, uint8_t preamble = LORA_PREAMBLE_SYM,
                            bool has_crc = (LORA_HAS_CRC != 0)) {
  bool low_dr = (sf >= 11 && bw_hz <= 125000);   // low data rate optimization

  float t_sym_ms = (float)(1UL << sf) * 1000.0f / (float)bw_hz;
  float t_pre_ms = ((float)preamble + 4.25f) * t_sym_ms;

  int h = 0;                          // explicit header
  int crc_bit = has_crc ? 1 : 0;
  int de = low_dr ? 1 : 0;

  int num = 8 * (int)payload_bytes - 4 * sf + 28 + 16 * crc_bit - 20 * h;
  int den = 4 * (sf - 2 * de);
  int ceil_term = (num + (den > 0 ? den - 1 : 0)) / den;
  if (ceil_term < 0) ceil_term = 0;
  int n_pay = 8 + ceil_term * (cr + 4);

  float total_ms = t_pre_ms + (float)n_pay * t_sym_ms;
  return (uint32_t)(total_ms + 0.5f);
}
static inline uint32_t toa_ms(size_t bytes) {
  return lora_toa_ms(LORA_SF, LORA_BW_HZ, LORA_CR, (uint16_t)bytes);
}

// ===== Callbacks =====
static LoraRxMsgCb     s_cb_msg  = nullptr;
static LoraConnStateCb s_cb_conn = nullptr;
static LoraL1Cb        s_cb_l1   = nullptr;

// ===== L2 재조립 — 봉투 src별 프레임 (§5 v1.19 MUST) =====
// 파일 스코프 버퍼 하나로는 A의 프레임이 열려 있는 동안 B의 bare 라인이 A의 메시지에
// 끼어든다. 송신자마다 프레임을 따로 두고, 각자 idle 타이머로 만료시킨다.
struct RxFrame {
  bool       open;
  String     msg;
  uint32_t   sof_ms, last_ms;
  uint16_t   chunks;
  LoraRxInfo info;          // 가장 최근 청크 기준
};
static RxFrame s_frames[LORA_RX_FRAMES];

// ===== 링크 상태 =====
static uint32_t s_last_rx_ms    = 0;
static uint32_t s_next_hb_ms    = 0;
static bool     s_connected     = true;
static String   s_my_id;                 // display id (HB/PONG에 실음)
static int      s_my_rssi_cached = 0;
static bool     s_my_rssi_valid  = false;

// ===== §8a 채널 예약 =====
static uint32_t s_chan_reserved_until = 0;

// ===== Range (주소지정 PING → PONG) =====
static volatile int      s_range_last_seq = -1;
static volatile uint32_t s_range_count    = 0;
static volatile bool     s_range_new      = false;
static bool              s_pong_pending   = false;   // §8: responder MAY hold only one
static uint32_t          s_pong_due_ms    = 0;

// ===== L1 TX 큐 (idle 지점에서 틱당 1개) =====
struct L1Tx { String line; uint8_t ttl; bool beacon; uint32_t deadline_ms; };
static L1Tx    s_l1_txq[L1_TXQ_N];
static uint8_t s_l1_txq_head = 0, s_l1_txq_count = 0;

// ===== LBT =====
static uint32_t s_lbt_hold_until = 0;
static uint8_t  s_lbt_streak     = 0;
static int      s_begin_status   = -1;   // radio.begin() 결과 (진단용)
static int      s_last_cad       = 0;    // 마지막 scanChannel() 원시 반환값 (진단용)
static uint32_t s_resumed_len    = 0;
static bool     s_hb_enabled     = true;

// ===== Discovery 테이블 / 통계 =====
static LoraNode  s_nodes[LORA_NODE_MAX];
static bool      s_nodes_changed = false;
static LoraStats s_stats = {};
static RelaySeen s_relay_seen;

void lora_set_my_id(const String& id) { s_my_id = id; }
bool lora_connected() { return s_connected; }
void lora_set_callbacks(LoraRxMsgCb on_msg, LoraConnStateCb on_conn) {
  s_cb_msg  = on_msg;
  s_cb_conn = on_conn;
}
void lora_set_l1_callback(LoraL1Cb on_l1) { s_cb_l1 = on_l1; }
bool lora_idle() {
  if (!s_radio_mtx || !s_tx_q) return false;
  if (s_l1_txq_count > 0 || s_pong_pending) return false;
  if (uxQueueMessagesWaiting(s_tx_q) > 0) return false;
  if (xSemaphoreTake(s_radio_mtx, 0) != pdTRUE) return false;   // TX 태스크가 프레임 송신 중
  xSemaphoreGive(s_radio_mtx);
  for (auto& f : s_frames) if (f.open) return false;             // 남의 메시지를 받는 중
  return true;
}

void lora_set_hb_enabled(bool on) { s_hb_enabled = on; }

int lora_dio1_pin() { return LORA_DIO1_PIN; }

bool lora_sleep_arm() {
  if (!s_radio_mtx) return false;
  xSemaphoreTake(s_radio_mtx, portMAX_DELAY);
  s_rx_flag = false;
  // 프리앰블(8심볼)을 놓치지 않는 최소 청취 창을 RadioLib이 계산한다. SF9/BW125에서 대략
  // 25 % 듀티 → SX1262 평균 ~1.3 mA. 패킷이 들어오면 RxDone → DIO1 High → ESP32 ext0 wake.
  int st = radio.startReceiveDutyCycleAuto();
  if (st != RADIOLIB_ERR_NONE) {
    LOGF("[LORA] duty-cycle RX failed: %d — plain RX instead\n", st);
    radio.startReceive();
  }
  return st == RADIOLIB_ERR_NONE;                    // 뮤텍스는 일부러 안 돌려준다: 곧 딥슬립
}

void lora_get_stats(LoraStats* out) {
  if (!out) return;
  *out = s_stats;
  out->last_rssi = s_my_rssi_cached;
  out->last_rssi_valid = s_my_rssi_valid;
  out->radio_status = s_begin_status;
  out->resumed_len = s_resumed_len;
}

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------
static LoraNode* node_touch(const LoraRxInfo& info, const char* type) {
  uint32_t now = millis();
  LoraNode* n = nullptr;
  LoraNode* free_slot = nullptr;
  LoraNode* oldest = &s_nodes[0];
  for (int i = 0; i < LORA_NODE_MAX; i++) {
    LoraNode& c = s_nodes[i];
    if (c.valid && strcmp(c.id, info.src) == 0) { n = &c; break; }
    if (!c.valid) { if (!free_slot) free_slot = &c; continue; }
    if ((int32_t)(oldest->last_seen_ms - c.last_seen_ms) > 0) oldest = &c;
  }
  if (!n) {
    n = free_slot ? free_slot : oldest;          // 가득 차면 가장 오래된 노드를 밀어낸다
    *n = LoraNode();
    n->valid = true;
    strlcpy(n->id, info.src, sizeof(n->id));
    n->hops = -1;
    LOGF("[NODE] +new %s (%s) rssi=%d hops=%d\n", n->id, type, info.rssi, info.hops);
  }
  n->last_seen_ms = now;
  n->frames++;
  n->last_type = type;
  n->hops = info.hops;
  if (info.hops == 0) {                          // relay 경유분의 RSSI는 relay의 것 — 버린다
    n->rssi = info.rssi;
    n->snr  = info.snr;
    n->rssi_valid = true;
  }
  s_nodes_changed = true;
  return n;
}

static void nodes_purge_stale(uint32_t now) {
  for (int i = 0; i < LORA_NODE_MAX; i++) {
    LoraNode& c = s_nodes[i];
    if (c.valid && (uint32_t)(now - c.last_seen_ms) > LORA_NODE_STALE_MS) {
      LOGF("[NODE] -stale %s (age=%lus)\n", c.id, (unsigned long)(now - c.last_seen_ms) / 1000);
      c = LoraNode();
      s_nodes_changed = true;
    }
  }
}

int lora_nodes_snapshot(LoraNode* out, int max) {
  int k = 0;
  for (int i = 0; i < LORA_NODE_MAX && k < max; i++)
    if (s_nodes[i].valid) out[k++] = s_nodes[i];
  return k;
}
int lora_nodes_count() {
  int k = 0;
  for (int i = 0; i < LORA_NODE_MAX; i++) if (s_nodes[i].valid) k++;
  return k;
}
bool lora_nodes_consume_changed() {
  if (!s_nodes_changed) return false;
  s_nodes_changed = false;
  return true;
}

void lora_dump_neighbors() {
  uint32_t now = millis();
  Serial.println("[NODE] === heard nodes ===");
  int count = 0;
  for (int i = 0; i < LORA_NODE_MAX; i++) {
    const LoraNode& c = s_nodes[i];
    if (!c.valid) continue;
    Serial.printf("  %s \"%s\"  age=%lus  last=%s hops=%d frames=%lu",
                  c.id, c.name.c_str(), (unsigned long)(now - c.last_seen_ms) / 1000,
                  c.last_type.c_str(), c.hops, (unsigned long)c.frames);
    if (c.rssi_valid)      Serial.printf("  rssi=%d snr=%.1f", c.rssi, (double)c.snr);
    if (c.peer_rssi_valid) Serial.printf("  peer_rssi=%d", c.peer_rssi);
    if (c.info.length())   Serial.printf("  !%s %s", c.info_type.c_str(), c.info.c_str());
    Serial.println();
    count++;
  }
  if (count == 0) Serial.println("  (empty)");
  Serial.printf("[NODE] %d / %d slots.  rx ok=%lu dup=%lu own=%lu bad=%lu  tx=%lu lbt_defer=%lu\n",
                count, LORA_NODE_MAX, (unsigned long)s_stats.rx_ok, (unsigned long)s_stats.rx_dup,
                (unsigned long)s_stats.rx_own, (unsigned long)s_stats.rx_bad,
                (unsigned long)s_stats.tx_frames, (unsigned long)s_stats.lbt_defers);
  Serial.printf("[DEDUP] ring=%d hits=%lu misses=%lu late=%lu widest_hit=%lus horizon=%lus\n",
                RELAY_SEEN_N, (unsigned long)s_relay_seen.hits, (unsigned long)s_relay_seen.misses,
                (unsigned long)s_relay_seen.late, (unsigned long)s_relay_seen.widest_hit / 1000,
                (unsigned long)relay_horizon(s_relay_seen, now) / 1000);
}

// ---------------------------------------------------------------------------
// Radio primitives (all called with s_radio_mtx held)
// ---------------------------------------------------------------------------
static void radio_start_rx() {
  s_rx_flag = false;
  int st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) LOGF("[LORA] startReceive failed: %d\n", st);
}

// One non-persistent CAD sense. true = clear to transmit now. On busy: back off a random
// interval WITH THE RADIO LISTENING (scanChannel leaves it in standby) and report false.
static bool lbt_clear() {
#if LORA_LBT
  if ((int32_t)(millis() - s_lbt_hold_until) < 0) return false;
  if (s_lbt_streak >= LBT_MAX_DEFERS) { s_lbt_streak = 0; return true; }
  s_last_cad = radio.scanChannel();
  if (s_last_cad == RADIOLIB_CHANNEL_FREE) { s_lbt_streak = 0; return true; }
  s_lbt_streak++;
  s_stats.lbt_defers++;
  s_lbt_hold_until = millis() + LBT_BACKOFF_MIN_MS + (esp_random() % LBT_BACKOFF_SPAN_MS);
  radio_start_rx();
  return false;
#else
  return true;
#endif
}

// Wrap one protocol line (R|src|pktid|ttl|line) and push it as ONE LoRa packet.
// Callers re-arm RX (radio_start_rx) after finishing a burst.
static size_t lora_emit(const String& line, uint8_t ttl) {
  String w = relay_wrap(line, ttl);
  int st = radio.transmit(w);
  if (st != RADIOLIB_ERR_NONE) LOGF("[LORA] transmit failed: %d\n", st);
  s_stats.tx_frames++;
  return w.length();
}

// ---------------------------------------------------------------------------
// Init / diagnostics
// ---------------------------------------------------------------------------
// SX1262 하드웨어 리셋을 RadioLib에 맡기지 않고 직접 한다.
// RadioLib(7.7.1)의 reset()은 NRST를 올린 직후 곧바로 BUSY를 보고 명령을 보내는데, XIAO ESP32S3
// 에서는 칩이 BUSY를 올리기도 전에 그 검사가 통과해 부팅 중인 칩에 명령이 들어간다. 그러면
// 버전 레지스터가 쓰레기/0으로 읽혀 begin()이 -2(CHIP_NOT_FOUND)로 죽는다 — 10번 재시도해도
// 매번 같은 레이스를 반복한다. (2026-09-18 실측: 기본 리셋 → -2, 아래 시퀀스 → 매번 0.
// RadioLib SPI 디버그를 켜면 print 지연 덕에 우연히 성공해서 더 헷갈린다.)
static void radio_hw_reset() {
  pinMode(LORA_BUSY_PIN, INPUT);
  pinMode(LORA_RST_PIN, OUTPUT);
  digitalWrite(LORA_RST_PIN, LOW);
  delay(2);
  digitalWrite(LORA_RST_PIN, HIGH);
  delay(10);                                         // 칩 부팅 — BUSY가 올라올 시간을 먼저 준다
  uint32_t t0 = millis();
  while (digitalRead(LORA_BUSY_PIN) && (uint32_t)(millis() - t0) < 100) delay(1);
  delay(5);
}

static void radio_read_packet();   // defined below (receive section)

void lora_begin(bool hw_reset) {
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  if (hw_reset) radio_hw_reset();
  radio.resetOnStartup = false;                      // 위에서 이미 했다 — RadioLib의 레이스를 피한다
  int st = radio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR_DENOM,
                       RF_SYNC_WORD, LORA_TX_DBM, RF_PREAMBLE, LORA_TCXO_V);
  s_begin_status = st;
  if (st != RADIOLIB_ERR_NONE) {
    LOGF("[LORA] SX1262 begin FAILED: %d (check wiring/TCXO)\n", st);
  } else {
    LOGF("[LORA] SX1262 begin OK\n");
  }

  // Wio-SX1262: 안테나 스위치는 SX1262 내부 DIO2가 제어. CRC는 lora_rf.h를 따른다 —
  // explicit header가 CRC-present 비트를 싣기 때문에 노드별로 켜도 flag day가 아니다.
  radio.setDio2AsRfSwitch(true);
#ifdef LORA_RXEN_PIN
  // Wio-SX1262의 RF 스위치는 DIO2(TX 경로)만으로 끝나지 않는다: RX 경로 enable이 ESP32
  // GPIO에 따로 나와 있어 수신 중엔 HIGH로 잡아줘야 한다 (Meshtastic SX126X_RXEN=38과 동일).
  radio.setRfSwitchPins(LORA_RXEN_PIN, RADIOLIB_NC);
#endif
  radio.setCRC(RF_CRC_ON ? 2 : 0);

  radio.setPacketReceivedAction(lora_on_dio1);
  if (!hw_reset) {
    // 딥슬립 중 DIO1(RxDone)로 깬 경우: begin()은 IRQ만 지우고 수신 버퍼는 남긴다. 버퍼에
    // 패킷이 있으면 지금 읽는다 — 이 패킷이 우리를 깨운 프레임이다.
    size_t len = radio.getPacketLength();
    if (len > 0 && len < 256) {
      LOGF("[LORA] resume: %u B pending in RX buffer (woke us)\n", (unsigned)len);
      s_resumed_len = len;
      radio_read_packet();                           // 처리 후 RX 재무장까지 함
    } else {
      radio.startReceive();
    }
  } else {
    radio.startReceive();
  }

  LOGF("[LORA] SF%u BW%lu CR4/%u CRC=%d freq=%.1fMHz pwr=%ddBm LBT=%d → ToA(80B)=%lums\n",
       LORA_SF, (unsigned long)LORA_BW_HZ, LORA_CR + 4, LORA_HAS_CRC,
       (double)RF_FREQ_MHZ, LORA_TX_DBM, LORA_LBT, (unsigned long)toa_ms(80));
  LOGF("[LORA] node=%s  frame_idle=%lums  pong_broadcast=%d\n",
       NODE_ID, (unsigned long)LORA_FRAME_IDLE_MS, LORA_PONG_BROADCAST);

  s_last_rx_ms = millis();
  s_next_hb_ms = millis() + 3000 + (esp_random() % 3000);   // 부팅 직후 1발 → discovery에 빨리 뜬다

  s_radio_mtx = xSemaphoreCreateMutex();
  s_tx_q      = xQueueCreate(3, sizeof(LoraTxMsg));
  xTaskCreate(lora_tx_task, "lora_tx", 8192, nullptr, 1, nullptr);
  relay_begin();             // seed relay pktid counter (random)
}

void lora_set_channel(uint8_t channel) {
  LOGF("[LORA] channel command ignored — SX1262 uses fixed freq %.1f MHz (edit lora_rf.h). req=%u\n",
       (double)RF_FREQ_MHZ, channel);
}

void lora_probe_at() {
  Serial.println("[PROBE] SX1262 (SPI) — no AT layer. RF config:");
  Serial.printf("  node=%s  freq=%.3f MHz  SF%u  BW=%.0f kHz  CR=4/%u  CRC=%s  pwr=%d dBm  preamble=%u  LBT=%d\n",
                NODE_ID, (double)RF_FREQ_MHZ, RF_SF, (double)RF_BW_KHZ, RF_CR_DENOM,
                RF_CRC_ON ? "on" : "off", LORA_TX_DBM, RF_PREAMBLE, LORA_LBT);
  Serial.printf("  pins: nss=%d dio1=%d busy=%d rst=%d sck=%d miso=%d mosi=%d\n",
                LORA_NSS_PIN, LORA_DIO1_PIN, LORA_BUSY_PIN, LORA_RST_PIN,
                LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN);
  // begin=0이면 SX1262 초기화 성공. last_cad: CHANNEL_FREE / LORA_DETECTED / 그 외 = 에러 코드.
  Serial.printf("  radio: begin=%d  last_cad=%d  lbt_streak=%u  lbt_defers=%lu\n",
                s_begin_status, s_last_cad, (unsigned)s_lbt_streak, (unsigned long)s_stats.lbt_defers);
}

void lora_query_at_help() { lora_probe_at(); }

void lora_query_rssi() {
  if (s_radio_mtx) xSemaphoreTake(s_radio_mtx, portMAX_DELAY);
  float rssi = radio.getRSSI();
  float snr  = radio.getSNR();
  if (s_radio_mtx) xSemaphoreGive(s_radio_mtx);
  Serial.printf("[RSSI] last packet: %.1f dBm  SNR=%.1f dB%s\n",
                (double)rssi, (double)snr,
                s_my_rssi_valid ? "" : "  (no packet received yet)");
}

void lora_bridge_loop() {
  Serial.println("[BRIDGE] not applicable on SX1262 (no UART transparent mode).");
}

// ---------------------------------------------------------------------------
// Send (L2)
// ---------------------------------------------------------------------------
// Post-transmit pacing. RadioLib's blocking transmit() already stays on-air for 1x ToA,
// so ~2x ToA of quiet AFTER it lets the half-duplex relay forward each packet before the
// next one starts (§8: T-Deck uses the same 2x ToA + margin).
static uint32_t lora_packet_delay(size_t wrapped_bytes) {
  uint32_t d = toa_ms(wrapped_bytes) * 2 + 50;
  if (d < 300) d = 300;
  return d;
}

// TX-task flavour of LBT: block (task context, not the main loop) until clear.
static void lbt_wait_blocking() {
  while (!lbt_clear()) vTaskDelay(pdMS_TO_TICKS(10));
}

static void emit_paced(const String& line) {
  lbt_wait_blocking();
  size_t n = lora_emit(line, RELAY_TTL_MESH);
  radio_start_rx();                      // 간격 동안은 듣고 있는다 (다음 CAD도 여기서 출발)
  vTaskDelay(pdMS_TO_TICKS(lora_packet_delay(n)));
}

// L2 escape (§5): '!'로 시작하는 사용자 청크는 "!!…"로 보낸다. escape 1바이트는 자를 때
// 미리 예약한다 (v1.8) — 60B로 자른 뒤 붙이면 61B가 되어 ≤60B 불변식이 깨진다.
static void send_chunked(const String& msg, size_t max_bytes) {
  const char* p = msg.c_str();
  size_t n = msg.length();
  size_t i = 0;
  while (i < n) {
    bool   esc    = (p[i] == '!');
    size_t budget = esc ? max_bytes - 1 : max_bytes;
    size_t end = i, bytes = 0;
    while (end < n) {
      uint8_t b = (uint8_t)p[end];
      size_t sz = 1;
      if      ((b & 0xE0) == 0xC0) sz = 2;
      else if ((b & 0xF0) == 0xE0) sz = 3;
      else if ((b & 0xF8) == 0xF0) sz = 4;
      if (bytes + sz > budget) break;
      if (end + sz > n) break;
      bytes += sz;
      end   += sz;
    }
    if (end == i) end = i + 1;
    String chunk = msg.substring(i, end);
    emit_paced(esc ? "!" + chunk : chunk);
    i = end;
  }
}

// The actual blocking transmit — runs ONLY in lora_tx_task, holding s_radio_mtx.
static void do_send_blocking(const char* text) {
  LOGF("[LORA] >>> SOF (len=%u)\n", (unsigned)strlen(text));
  emit_paced("[SOF]");

  String body = text;
  body.trim();
  body.replace("\n", "[NL]");
  if (body.length() > 0) send_chunked(body, LORA_MAX_LINE_BYTES);

  emit_paced("[EOF]");                   // 청크가 실패했어도 [EOF]는 반드시 나간다 (§5)
  LOGF("[LORA] >>> EOF\n");
}

static void lora_tx_task(void*) {
  LoraTxMsg m;
  for (;;) {
    if (xQueueReceive(s_tx_q, &m, portMAX_DELAY) == pdTRUE) {
      if (s_radio_mtx) xSemaphoreTake(s_radio_mtx, portMAX_DELAY);
      do_send_blocking(m.text);
      if (s_radio_mtx) xSemaphoreGive(s_radio_mtx);
      s_tx_done = true;
    }
  }
}

void lora_send_message(const String& text) {
  if (!s_tx_q) return;
  LoraTxMsg m;
  size_t n = text.length();
  if (n >= sizeof(m.text)) n = sizeof(m.text) - 1;
  memcpy(m.text, text.c_str(), n);
  m.text[n] = '\0';
  if (xQueueSend(s_tx_q, &m, 0) != pdTRUE)
    LOGF("[LORA] TX queue full — message dropped\n");
}

bool lora_tx_consume_done() {
  if (!s_tx_done) return false;
  s_tx_done = false;
  return true;
}

bool lora_consume_range(int* out_seq, uint32_t* out_count) {
  if (!s_range_new) return false;
  s_range_new = false;
  if (out_seq)   *out_seq   = s_range_last_seq;
  if (out_count) *out_count = s_range_count;
  return true;
}

bool lora_send_l1(const String& line, uint8_t ttl, uint32_t max_defer_ms) {
  if (line.length() < 2 || line.charAt(0) != '!' || line.charAt(1) == '!' ||
      line.length() > LORA_MAX_LINE_BYTES || ttl < 1 || ttl > RELAY_TTL_MESH) {
    LOGF("[L1] refused (grammar/len %u): %s\n", (unsigned)line.length(), line.c_str());
    return false;
  }
  if (s_l1_txq_count >= L1_TXQ_N) {
    LOGF("[L1] TX queue full — dropped: %s\n", line.c_str());
    return false;
  }
  L1Tx& e = s_l1_txq[(s_l1_txq_head + s_l1_txq_count) % L1_TXQ_N];
  e.line = line;
  e.ttl = ttl;
  e.beacon = (max_defer_ms > 0);
  e.deadline_ms = millis() + max_defer_ms;
  s_l1_txq_count++;
  return true;
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------
// args를 앞쪽 탭 n-1개만 기준으로 쪼갠다. 마지막 조각은 남은 전부 (봉투 파싱과 같은 원칙).
static int l1_split(const String& args, String* out, int n) {
  int got = 0, pos = 0;
  while (got < n - 1) {
    int t = args.indexOf('\t', pos);
    if (t < 0) break;
    out[got++] = args.substring(pos, t);
    pos = t + 1;
  }
  out[got++] = args.substring(pos);
  return got;
}

static long b36_decode(const String& s) {
  if (s.length() == 0) return -1;
  long v = 0;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    int d;
    if      (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'A' && c <= 'Z') d = c - 'A' + 10;
    else if (c >= 'a' && c <= 'z') d = c - 'a' + 10;
    else return -1;
    v = v * 36 + d;
    if (v > 100000) return -1;
  }
  return v;
}

// §8a: 스트림 announce(!GR/!BR/!VA)를 들으면 그 스트림의 airtime을 예약.
//   reserve ≈ n × ToA(full frame) × (1 + 1.3 × ttl)
static void chan_reserve_announce(int n_chunks, uint8_t ttl) {
  if (n_chunks <= 0) n_chunks = 20;                   // n 미상(!VA) → 보수적 추정
  uint32_t est = (uint32_t)((uint64_t)n_chunks * toa_ms(79) * (10 + 13 * ttl) / 10);
  uint32_t until = millis() + est;
  if ((int32_t)(until - s_chan_reserved_until) > 0) s_chan_reserved_until = until;
  LOGF("[L1] stream reserve %lums (n=%d ttl=%u)\n", (unsigned long)est, n_chunks, (unsigned)ttl);
}
// §8a: 스트림 프레임이 실제로 흐르는 동안 3×ToA(그 프레임)씩 예약 갱신.
static void chan_reserve_renew(size_t frame_bytes) {
  uint32_t until = millis() + 3 * toa_ms(frame_bytes);
  if ((int32_t)(until - s_chan_reserved_until) > 0) s_chan_reserved_until = until;
}

static void frame_deliver(RxFrame& f, bool partial, const char* why) {
  String msg = f.msg;
  f.open = false;
  f.msg = "";
  msg.replace("[NL]", "\n");
  if (msg.length() == 0) {
    LOGF("[LORA] <<< %s: empty frame from %s\n", why, f.info.src);
    return;
  }
  f.info.partial = partial;
  LOGF("[LORA] <<< MSG from %s (%u B, %u chunks, %lums) [%s]\n", f.info.src,
       (unsigned)msg.length(), (unsigned)f.chunks, (unsigned long)(f.last_ms - f.sof_ms), why);
  if (s_cb_msg) s_cb_msg(msg, f.info);
}

static RxFrame* frame_find(const char* src) {
  for (auto& f : s_frames) if (f.open && strcmp(f.info.src, src) == 0) return &f;
  return nullptr;
}

static RxFrame* frame_open(const LoraRxInfo& info, uint32_t now) {
  RxFrame* slot = nullptr;
  for (auto& f : s_frames) if (!f.open) { slot = &f; break; }
  if (!slot) {                                        // 다 찼으면 가장 오래 조용한 프레임을 마감
    slot = &s_frames[0];
    for (auto& f : s_frames)
      if ((int32_t)(slot->last_ms - f.last_ms) > 0) slot = &f;
    frame_deliver(*slot, true, "evicted");
  }
  slot->open = true;
  slot->msg = "";
  slot->sof_ms = slot->last_ms = now;
  slot->chunks = 0;
  slot->info = info;
  return slot;
}

static void handle_l1_line(const String& line, const LoraRxInfo& info_in, size_t frame_bytes) {
  int tab = line.indexOf('\t');                       // line[0] == '!'
  String type = (tab >= 0) ? line.substring(1, tab) : line.substring(1);
  String args = (tab >= 0) ? line.substring(tab + 1) : String("");
  LoraRxInfo info = info_in;

  // hops는 "ttl=3으로 출발했다"는 가정 위의 해석이다 (§10). ttl=1-by-design 비콘은
  // direct, !CAR은 MESH로 출발, 나머지(라우터의 hop-adaptive reply 등)는 unknown.
  bool status_type = false;
  if (type == "RB" || type == "RS") { info.hops = 0; status_type = true; }
  else if (type == "CAR")           { status_type = true; }
  else if (type == "FS" || type == "CS") { info.hops = -1; status_type = true; }
  else                              { info.hops = -1; }

  // §8a — 이 노드는 뉴스/북/보이스를 소비하지 않지만 비콘 양보에는 참여한다.
  if (type == "GR") {                                 // !GR\t<art_id>\t<n>\t<crc>
    String f[3]; int n = (l1_split(args, f, 3) >= 2) ? (int)b36_decode(f[1]) : -1;
    chan_reserve_announce(n, info.ttl);
  } else if (type == "BR") {                          // !BR\t<book_id>\t<page>\t<n>\t<crc>
    String f[4]; int n = (l1_split(args, f, 4) >= 3) ? (int)b36_decode(f[2]) : -1;
    chan_reserve_announce(n, info.ttl);
  } else if (type == "VA") {
    chan_reserve_announce(-1, info.ttl);
  } else if (type == "GD" || type == "BD" || type == "VN" || type == "VT") {
    chan_reserve_renew(frame_bytes);
  }

  LoraNode* n = node_touch(info, type.c_str());
  if (status_type) {
    n->info_type = type;
    n->info = args.substring(0, 64);
  }
  if (s_cb_l1) s_cb_l1(type, args, info);
}

static void handle_hb(const String& line, LoraRxInfo& info) {
  //   "HB" | "HB\t<id>" | "HB\t<id>\trssi=<v>"   — ttl=1 by design → 항상 direct
  info.hops = 0;
  String id; int rssi = 0; bool has_rssi = false;
  int p1 = line.indexOf('\t');
  if (p1 >= 0) {
    int p2 = line.indexOf('\t', p1 + 1);
    if (p2 < 0) {
      id = line.substring(p1 + 1);
    } else {
      id = line.substring(p1 + 1, p2);
      String tail = line.substring(p2 + 1);
      if (tail.startsWith("rssi=")) { rssi = tail.substring(5).toInt(); has_rssi = true; }
    }
  }
  LoraNode* n = node_touch(info, "HB");
  if (id.length() > 0) n->name = id.substring(0, 24);
  if (has_rssi) { n->peer_rssi = rssi; n->peer_rssi_valid = true; }
  LOGF("[BEACON] <<< %s \"%s\" rssi=%d\n", info.src, id.c_str(), info.rssi);
}

static void handle_ping(const String& line, const LoraRxInfo& info, size_t frame_bytes) {
  // PING\t<seq>\t<id>[\t<dst>]
  String f[4];
  int nf = l1_split(line, f, 4);
  node_touch(info, "PING");
  if (nf < 2) return;
  bool addressed = (nf >= 4 && f[3].length() > 0);
  bool for_me = addressed ? (f[3] == NODE_ID || (s_my_id.length() && f[3] == s_my_id))
                          : (LORA_PONG_BROADCAST != 0);
  if (!for_me) return;                   // §10 v1.21: 브로드캐스트엔 인프라만 답한다

  // §8: 요청엔 절대 즉답하지 않는다 — "방금 받은 패킷"의 ToA × 4 만큼 기다린다. relay가
  // 이 PING을 forward 중이라 즉답은 (a) 그 forward와 충돌하고 (b) relay가 못 듣는다.
  s_range_last_seq = f[1].toInt();
  s_range_count = s_range_count + 1;
  s_range_new    = true;
  s_pong_due_ms  = millis() + 4 * toa_ms(frame_bytes);
  s_pong_pending = true;                 // 하나만 보류 — 새 PING이 옛 것을 덮는다 (§8)
  LOGF("[RANGE] PING #%d from %s → PONG in %lums\n", s_range_last_seq, info.src,
       (unsigned long)(4 * toa_ms(frame_bytes)));
}

static bool is_legacy_system_line(const String& line) {
  return line.startsWith("CS ")  || line.startsWith("CS\t") ||
         line.startsWith("SYS ") || line.startsWith("SYS\t");
}

static void process_rx_line(const String& line, LoraRxInfo& info, size_t frame_bytes) {
  uint32_t now = millis();
  RxFrame* f = frame_find(info.src);

  // L2 프레임 마커. 이 타입들은 RELAY_TTL_MESH로 출발하므로 hops 해석이 유효하다.
  if (line == "[SOF]") {
    if (f) frame_deliver(*f, true, "new SOF while open");   // 이전 [EOF] 유실
    frame_open(info, now);
    node_touch(info, "TEXT");
    return;
  }
  if (line == "[EOF]") {
    if (f) { f->info = info; f->last_ms = now; frame_deliver(*f, false, "EOF"); }
    else   LOGF("[LORA] <<< orphan [EOF] from %s (SOF lost)\n", info.src);
    return;
  }

  // L1 — 홑 '!'는 프레임 한복판이어도 항상 L1이고, 프레임 상태/idle 타이머를 건드리지
  // 않는다. "!!…"는 프레임이 열려 있을 때만 escape된 사용자 텍스트다.
  String payload = line;
  if (line.length() > 0 && line.charAt(0) == '!') {
    if (f && line.startsWith("!!")) {
      payload = line.substring(1);
    } else {
      handle_l1_line(line, info, frame_bytes);
      return;
    }
  }

  // 열린 프레임 안의 bare 라인은 정의상 사용자 텍스트다. L0/레거시 패턴 검사는 반드시
  // 이 뒤에 온다 (v1.8) — 안 그러면 "PING…"으로 시작하는 청크가 조용히 먹힌다.
  if (f) {
    if (f->msg.length() + payload.length() <= LORA_RX_MSG_MAX_BYTES) f->msg += payload;
    f->chunks++;
    f->last_ms = now;
    f->info = info;
    return;
  }

  if (line == "HB" || line.startsWith("HB\t"))  { handle_hb(line, info); return; }
  if (line.startsWith("PING\t"))                { handle_ping(line, info, frame_bytes); return; }
  if (line.startsWith("PONG\t"))                { node_touch(info, "PONG"); return; }
  if (is_legacy_system_line(line)) {
    LOGF("[L1] legacy bare system line — dropped: \"%s\"\n", line.c_str());
    return;
  }

  // [SOF] 없이 온 bare 라인 = SOF가 유실된 청크. 바이트는 도착했으므로 버리지 않고
  // "잘림" 표시를 달아 전달한다 (§5 v1.19의 delivered-marked 원칙).
  node_touch(info, "TEXT");
  info.partial = true;
  String msg = payload;
  msg.replace("[NL]", "\n");
  LOGF("[LORA] <<< orphan chunk from %s (%u B)\n", info.src, (unsigned)msg.length());
  if (s_cb_msg) s_cb_msg(msg, info);
}

// 봉투 계층: R| 헤더 검증/해제 → 자기 echo와 중복(direct + relayed 사본) 제거 → 원문 처리.
static void rx_dispatch(const String& line, int rssi, float snr) {
  String src, orig; uint32_t pktid; uint8_t ttl;
  // relay_parse가 §4 grammar(ttl ≤ 3, src = [A-Z][0-9A-F]{2})까지 본다. 유효한 R| 프레임이
  // 아니면 손상/노이즈 락 — 버린다 (§9).
  if (!relay_parse(line, src, pktid, ttl, orig) || ttl == 0 || orig.length() == 0) {
    s_stats.rx_bad++;
    LOGF("[DROP] bad envelope (%uB): \"%s\"\n", (unsigned)line.length(),
         line.substring(0, 40).c_str());
    return;
  }
  if (src == NODE_ID) { s_stats.rx_own++; return; }
  if (relay_seen(s_relay_seen, src, pktid, millis())) { s_stats.rx_dup++; return; }
  s_stats.rx_ok++;

  LoraRxInfo info = {};
  strlcpy(info.src, src.c_str(), sizeof(info.src));
  info.rssi = rssi;
  info.snr  = snr;
  info.ttl  = ttl;
  info.hops = (int8_t)(RELAY_TTL_MESH - ttl);

  s_last_rx_ms = millis();
  if (!s_connected) {
    s_connected = true;
    if (s_cb_conn) s_cb_conn(true);
  }
  process_rx_line(orig, info, line.length());
}

static void radio_read_packet() {
  uint8_t buf[256];
  size_t len = radio.getPacketLength();
  if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
  int st = radio.readData(buf, len);
  int   rssi = (int)lroundf(radio.getRSSI());
  float snr  = radio.getSNR();
  radio_start_rx();                                   // 처리보다 재무장이 먼저

  if (st != RADIOLIB_ERR_NONE || len == 0) {
    s_stats.rx_bad++;
    if (st != RADIOLIB_ERR_NONE) LOGF("[LORA] readData err: %d (crc/len — dropped)\n", st);
    return;
  }
  s_my_rssi_cached = rssi;
  s_my_rssi_valid  = true;

  // 보이스 청크 (VOICE.md §4.2): 유일한 바이너리 프레임 클래스. 소비는 안 하지만
  // 흐르는 스트림이므로 §8a 예약을 갱신한다.
  if (buf[0] == 0xC2) { chan_reserve_renew(len); return; }

  // §4 v1.16: 수신자는 뒤에 붙은 CR/LF/NUL을 벗기고, 거기에 의존하지도 않는다.
  while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n' || buf[len - 1] == 0)) len--;
  if (len == 0 || memchr(buf, 0, len)) { s_stats.rx_bad++; return; }
  buf[len] = 0;
  rx_dispatch(String((const char*)buf), rssi, snr);
}

void lora_test_inject(const String& envelope_line) {
  if (!s_radio_mtx || xSemaphoreTake(s_radio_mtx, pdMS_TO_TICKS(2000)) != pdTRUE) return;
  rx_dispatch(envelope_line, -60, 9.0f);
  xSemaphoreGive(s_radio_mtx);
}

void lora_tick() {
  // Share the radio with the TX task: if an L2 frame is in flight, skip this tick.
  if (!s_radio_mtx || xSemaphoreTake(s_radio_mtx, 0) != pdTRUE) return;

  if (s_rx_flag) {
    s_rx_flag = false;
    radio_read_packet();
  }

  uint32_t now = millis();

  // §5 v1.19: 열린 프레임은 수신자의 idle 타이머로 만료된다. L1 라인은 이 타이머를
  // 리셋하지 않는다 (프레임은 자기 청크로만 정의된다). 잘린 채로라도 표시해서 전달.
  for (auto& f : s_frames) {
    if (f.open && (uint32_t)(now - f.last_ms) > LORA_FRAME_IDLE_MS) {
      frame_deliver(f, true, "idle timeout");
    }
  }

  // ---- idle 지점 송신: 틱당 최대 1개. 우선순위 = 응답(PONG) > 요청 L1 > beacon-class ----
  bool reserved = (int32_t)(s_chan_reserved_until - now) > 0;
  bool sent = false;

  if (s_pong_pending && (int32_t)(now - s_pong_due_ms) >= 0) {
    if (lbt_clear()) {                                // 응답은 §8a 양보 대상이 아니다
      s_pong_pending = false;
      lora_emit("PONG\t" + String(s_range_last_seq) + "\t" +
                (s_my_id.length() ? s_my_id : String(NODE_ID)), RELAY_TTL_MESH);
      LOGF("[RANGE] >>> PONG #%d sent\n", s_range_last_seq);
      sent = true;
    }
  } else if (s_l1_txq_count > 0) {
    // beacon-class는 예약 중이면 양보 — 단 자기 주기 1회(deadline)를 넘기진 않는다 (MUST:
    // 위조 announce로 비콘을 무한 침묵시키는 DoS의 상한). 양보 중인 비콘이 뒤의 요청을
    // 막지 않도록, 지금 나갈 수 있는 첫 항목을 head로 끌어온다.
    auto yields = [&](const L1Tx& x) {
      return x.beacon && reserved && (int32_t)(now - x.deadline_ms) < 0;
    };
    for (uint8_t k = 1; k < s_l1_txq_count && yields(s_l1_txq[s_l1_txq_head]); k++) {
      L1Tx& other = s_l1_txq[(s_l1_txq_head + k) % L1_TXQ_N];
      if (!yields(other)) { L1Tx tmp = other; other = s_l1_txq[s_l1_txq_head]; s_l1_txq[s_l1_txq_head] = tmp; }
    }
    L1Tx& e = s_l1_txq[s_l1_txq_head];
    bool yield = yields(e);
    if (!yield && lbt_clear()) {
      lora_emit(e.line, e.ttl);
      LOGF("[L1] >>> %s\n", e.line.c_str());
      e.line = "";
      s_l1_txq_head = (s_l1_txq_head + 1) % L1_TXQ_N;
      s_l1_txq_count--;
      sent = true;
    }
  } else if (s_hb_enabled && (int32_t)(now - s_next_hb_ms) >= 0) {
    bool overdue_full_period = (int32_t)(now - s_next_hb_ms) >= (int32_t)LORA_HB_TX_MS;
    if ((!reserved || overdue_full_period) && lbt_clear()) {
      String hb = "HB";
      if (s_my_id.length() > 0) { hb += '\t'; hb += s_my_id; }
      if (s_my_rssi_valid)      { hb += "\trssi="; hb += String(s_my_rssi_cached); }
      lora_emit(hb, RELAY_TTL_LOCAL);                 // HB stays local (ttl=1, never relayed)
      LOGF("[BEACON] >>> %s\n", hb.c_str());
      s_next_hb_ms = now + LORA_HB_TX_MS + (esp_random() % 4000);   // 다른 노드와 위상 고정 방지
      sent = true;
    }
  }
  if (sent) radio_start_rx();

  nodes_purge_stale(now);

  if (s_connected && (uint32_t)(now - s_last_rx_ms) > LORA_HB_TIMEOUT_MS) {
    s_connected = false;
    if (s_cb_conn) s_cb_conn(false);
  }

  xSemaphoreGive(s_radio_mtx);
}
