#include "ble_dash.h"
#include "config.h"
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// iOS가 권장하는 광고 간격 (0.625 ms 단위). 주행 152.5 ms / 주차 546.25 ms.
#define ADV_FAST_UNITS   244
#define ADV_SLOW_UNITS   874
#define TX_FIFO_BYTES    8192
#define TX_FRAG_MAX      180          // iOS MTU 185 → ATT payload 182
#define TX_FRAG_GAP_MS   6            // notify 사이 간격 — mbuf 고갈 방지
#define CMD_MAX_BYTES    600          // "MSG " + 채팅 본문(≤512B)

struct DashCmd { char s[CMD_MAX_BYTES]; };

static NimBLEServer*         s_server = nullptr;
static NimBLECharacteristic* s_tx     = nullptr;
static QueueHandle_t         s_cmd_q  = nullptr;

// NimBLE 태스크가 쓰고 메인 루프가 읽는 플래그들
static volatile bool     s_connected  = false;
static volatile bool     s_subscribed = false;
static volatile bool     s_just_ready = false;
static volatile uint16_t s_conn_handle = 0;
static bool              s_slow_adv   = false;

// 송신 FIFO — 메인 루프 전용 (send_line / tick 둘 다 메인 루프에서만 호출)
static uint8_t  s_fifo[TX_FIFO_BYTES];
static size_t   s_fifo_head = 0, s_fifo_len = 0;
static uint32_t s_last_frag_ms = 0;

static void adv_start() {
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->stop();
  uint16_t units = s_slow_adv ? ADV_SLOW_UNITS : ADV_FAST_UNITS;
  adv->setMinInterval(units);
  adv->setMaxInterval(units);
  adv->start();
}

class ServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& ci) override {
    s_conn_handle = ci.getConnHandle();
    s_subscribed  = false;
    s_connected   = true;
    LOGF("[BLE] phone connected (handle=%u)\n", (unsigned)ci.getConnHandle());
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    s_connected  = false;
    s_subscribed = false;
    LOGF("[BLE] phone disconnected (reason=%d) — advertising again\n", reason);
    adv_start();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override {
    LOGF("[BLE] MTU=%u\n", (unsigned)mtu);
  }
};

class TxCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t sub) override {
    s_subscribed = (sub & 0x0001) != 0;
    if (s_subscribed) s_just_ready = true;
    LOGF("[BLE] notify %s\n", s_subscribed ? "subscribed" : "unsubscribed");
  }
};

// 폰은 긴 명령을 여러 번의 write로 쪼개 보낼 수 있다 → '\n'까지 누적.
class RxCb : public NimBLECharacteristicCallbacks {
  DashCmd m_cur = {};
  size_t  m_len = 0;
  bool    m_overflow = false;
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    NimBLEAttValue v = c->getValue();
    const uint8_t* d = v.data();
    for (size_t i = 0; i < v.size(); i++) {
      char ch = (char)d[i];
      if (ch == '\r') continue;
      if (ch != '\n') {
        if (m_len < sizeof(m_cur.s) - 1) m_cur.s[m_len++] = ch;
        else m_overflow = true;
        continue;
      }
      m_cur.s[m_len] = '\0';
      if (m_len > 0 && !m_overflow && s_cmd_q) xQueueSend(s_cmd_q, &m_cur, 0);
      m_len = 0;
      m_overflow = false;
    }
  }
};

static ServerCb s_server_cb;
static TxCb     s_tx_cb;
static RxCb     s_rx_cb;

void ble_dash_begin() {
  s_cmd_q = xQueueCreate(4, sizeof(DashCmd));

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(247);

  uint32_t rx_props = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
#if VEH_BLE_PASSKEY
  // 쓰기 경로(= 이 노드 이름으로 LoRa에 송신할 권한)만 인증을 요구한다. 상태/수신 채팅은
  // 어차피 LoRa에서 평문으로 떠다니는 정보라 notify 쪽은 열어둔다.
  NimBLEDevice::setSecurityAuth(true, true, true);              // bond + MITM + SC
  NimBLEDevice::setSecurityPasskey(VEH_BLE_PASSKEY);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
  rx_props |= NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN;
#endif

  s_server = NimBLEDevice::createServer();
  s_server->setCallbacks(&s_server_cb, false);
  s_server->advertiseOnDisconnect(false);                       // 간격을 고르려고 직접 재시작한다

  NimBLEService* svc = s_server->createService(NUS_SERVICE_UUID);
  s_tx = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  s_tx->setCallbacks(&s_tx_cb);
  NimBLECharacteristic* rx = svc->createCharacteristic(NUS_RX_UUID, rx_props, CMD_MAX_BYTES);
  rx->setCallbacks(&s_rx_cb);

  // 128-bit UUID(18B) + flags(3B)면 31B 광고에 이름이 안 들어간다 → 이름은 scan response로.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setName(BLE_DEVICE_NAME);
  adv->enableScanResponse(true);
  adv_start();
  LOGF("[BLE] advertising as \"%s\" (NUS)%s\n", BLE_DEVICE_NAME,
       VEH_BLE_PASSKEY ? " — passkey required for writes" : "");
}

void ble_dash_end() {
  // NimBLEDevice::deinit()는 호스트 태스크 종료 경로에서 널 포인터를 불러 panic한다(NimBLE 2.5.1 +
  // esp32 core 3.3.x, PC=0 in host_task). 딥슬립은 어차피 BLE 컨트롤러 전원을 내리므로 광고만 멈춘다.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  if (adv) adv->stop();
  s_connected = s_subscribed = false;
}

bool ble_dash_ready() { return s_connected && s_subscribed; }

bool ble_dash_consume_just_ready() {
  if (!s_just_ready) return false;
  s_just_ready = false;
  return true;
}

bool ble_dash_poll_command(String* out) {
  DashCmd c;
  if (!s_cmd_q || xQueueReceive(s_cmd_q, &c, 0) != pdTRUE) return false;
  if (out) *out = c.s;
  return true;
}

void ble_dash_set_slow_adv(bool slow) {
  if (slow == s_slow_adv) return;
  s_slow_adv = slow;
  if (!s_connected) adv_start();          // 연결 중이면 다음 disconnect 때 새 간격이 적용된다
}

bool ble_dash_send_line(const String& json) {
  if (!ble_dash_ready()) return false;
  size_t n = json.length() + 1;           // + '\n'
  if (n > TX_FIFO_BYTES - s_fifo_len) {
    LOGF("[BLE] TX fifo full — line dropped (%u B)\n", (unsigned)n);
    return false;                         // 줄 단위 원자성: 반쪽짜리 JSON은 절대 안 넣는다
  }
  size_t w = (s_fifo_head + s_fifo_len) % TX_FIFO_BYTES;
  for (size_t i = 0; i < n; i++) {
    s_fifo[w] = (i + 1 == n) ? '\n' : (uint8_t)json[i];
    w = (w + 1) % TX_FIFO_BYTES;
  }
  s_fifo_len += n;
  return true;
}

void ble_dash_tick() {
  if (!ble_dash_ready()) { s_fifo_head = s_fifo_len = 0; return; }
  if (s_fifo_len == 0) return;
  uint32_t now = millis();
  if ((uint32_t)(now - s_last_frag_ms) < TX_FRAG_GAP_MS) return;

  uint16_t mtu = s_server->getPeerMTU(s_conn_handle);
  size_t frag = (mtu > 23 ? mtu : 23) - 3;
  if (frag > TX_FRAG_MAX) frag = TX_FRAG_MAX;
  if (frag > s_fifo_len)  frag = s_fifo_len;

  uint8_t tmp[TX_FRAG_MAX];
  for (size_t i = 0; i < frag; i++) tmp[i] = s_fifo[(s_fifo_head + i) % TX_FIFO_BYTES];
  s_last_frag_ms = now;
  if (!s_tx->notify(tmp, frag, s_conn_handle)) return;   // 혼잡 — 다음 틱에 같은 조각 재시도
  s_fifo_head = (s_fifo_head + frag) % TX_FIFO_BYTES;
  s_fifo_len -= frag;
}

String json_escape(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if      (c == '"')  o += "\\\"";
    else if (c == '\\') o += "\\\\";
    else if (c == '\n') o += "\\n";
    else if (c == '\t') o += "\\t";
    else if (c == '\r') o += "\\r";
    else if (c < 0x20)  { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
    else                o += (char)c;
  }
  return o;
}
