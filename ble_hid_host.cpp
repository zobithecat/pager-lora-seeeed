#include "ble_hid_host.h"
#include "config.h"
#include "display.h"

#include <NimBLEDevice.h>

// ===== Service / Characteristic UUIDs =====
static const NimBLEUUID SVC_HID(    (uint16_t)0x1812 );
static const NimBLEUUID SVC_BATT(   (uint16_t)0x180F );
static const NimBLEUUID CHR_REPORT( (uint16_t)0x2A4D );
static const NimBLEUUID CHR_BOOT_KBD_IN( (uint16_t)0x2A22 );
static const NimBLEUUID CHR_BOOT_MOUSE_IN( (uint16_t)0x2A33 );
static const NimBLEUUID CHR_PROTO_MODE( (uint16_t)0x2A4E );
static const NimBLEUUID CHR_HID_CTRL(   (uint16_t)0x2A4C );
static const NimBLEUUID CHR_BATT_LEVEL( (uint16_t)0x2A19 );

// ===== State =====
enum class HostState { Idle, Scanning, ConnectPending, Connecting, Connected };

static HostState           s_state = HostState::Idle;
static NimBLEAddress       s_target_addr;
static bool                s_have_target = false;
static NimBLEClient*       s_client = nullptr;
static uint32_t            s_last_state_change_ms = 0;
static bool                s_attrs_cached = false;   // 첫 디스커버리 끝났나
static uint32_t            s_last_direct_try_ms = 0; // 직접 connect 마지막 시도

static HidReportCb         s_cb_report = nullptr;
static BleStateCb          s_cb_state  = nullptr;
static BatteryCb           s_cb_batt   = nullptr;

static void set_state(HostState st, BleVisualState vis) {
  s_state = st;
  s_last_state_change_ms = millis();
  if (s_cb_state) s_cb_state((int)vis);
}

// ===== Notification handler =====
// Boot keyboard report format: [mod, reserved, k1, k2, k3, k4, k5, k6]
// Some keyboards prepend a Report ID byte.
static void on_notify(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool /*isNotify*/) {
  const NimBLEUUID uuid = chr->getUUID();
  if (uuid == CHR_BATT_LEVEL) {
    if (len >= 1 && s_cb_batt) s_cb_batt(data[0]);
    return;
  }
  // Boot-mouse (0x2A33) reports are [buttons, dx, dy, wheel] — NOT a keyboard
  // report. The length-guessing parser below read a mouse delta as a pressed
  // key (stuck phantom 'f'), so drop mouse notifications outright.
  if (uuid == CHR_BOOT_MOUSE_IN) return;

  // dump raw for debugging
  LOGF("[HID] len=%u data=", (unsigned)len);
  for (size_t i = 0; i < len && i < 16; i++) LOGF("%02X ", data[i]);
  LOGF("\n");

  // A keyboard report is [mod, reserved, k1..k6] = 8 bytes (optionally prefixed
  // by a report-ID byte). Anything shorter is a mouse/consumer/other report; the
  // old `len >= 3` fallback misparsed those into phantom keystrokes, so require a
  // full 8-byte report and ignore everything else.
  if (len >= 8) {
    const uint8_t* p = data + (len - 8);   // last 8 bytes = the keyboard report
    uint8_t mod = p[0];
    uint8_t keys[6];
    for (int i = 0; i < 6; i++) keys[i] = p[2 + i];
    if (s_cb_report) s_cb_report(mod, keys);
  } else {
    LOGF("[HID] non-keyboard report ignored (len=%u)\n", (unsigned)len);
  }
}

// ===== Client callbacks =====
class ClientCb : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* /*c*/) override {
    LOGF("[BLE] onConnect (link up)\n");
  }
  void onDisconnect(NimBLEClient* /*c*/, int reason) override {
    LOGF("[BLE] onDisconnect reason=%d\n", reason);
    // 본드된 디바이스면 scan으로 재광고 잡아 재연결하도록 ConnectPending으로
    if (NimBLEDevice::getNumBonds() > 0 && s_have_target) {
      set_state(HostState::ConnectPending, BleVisualState::Connecting);
    } else {
      set_state(HostState::Idle, BleVisualState::Disconnected);
    }
  }
  void onAuthenticationComplete(NimBLEConnInfo& info) override {
    LOGF("[BLE] auth complete: encrypted=%d bonded=%d\n",
         info.isEncrypted(), info.isBonded());
  }
  // IO_NO_INPUT_OUTPUT + Just Works를 쓰므로 passkey 콜백은 정의 안 함 (기본값 사용).
} s_client_cb;

// ===== Scan callbacks =====
class ScanCb : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (s_state != HostState::Scanning) return;

    // A bonded keyboard re-advertises *directed* (no service UUIDs, often no
    // name) when it wakes, which the HID-service filter below misses — so it
    // could never auto-reconnect after a drop/sleep. Accept our already-bonded
    // device by address directly; only unknown devices need the HID/name filter.
    bool is_bonded = s_have_target && dev->getAddress() == s_target_addr;
    if (is_bonded) {
      LOGF("[SCAN] bonded keyboard re-advertising: %s  rssi=%d\n",
           dev->getAddress().toString().c_str(), dev->getRSSI());
    } else {
      if (!dev->isAdvertisingService(SVC_HID)) return;     // fresh-pairing path
      const char* want = TARGET_DEVICE_NAME;
      bool name_filter = (want && want[0] != '\0');
      bool match = !name_filter ||
                   (dev->haveName() && dev->getName() == want);
      LOGF("[SCAN] HID device: %s  rssi=%d  name=%s%s\n",
           dev->getAddress().toString().c_str(),
           dev->getRSSI(),
           dev->haveName() ? dev->getName().c_str() : "(no name)",
           match ? "" : "  (filtered)");
      if (!match) return;
    }

    s_target_addr = dev->getAddress();
    s_have_target = true;
    NimBLEDevice::getScan()->stop();
    set_state(HostState::Connecting, BleVisualState::Connecting);   // -> loop runs do_connect
  }
  void onScanEnd(const NimBLEScanResults& /*r*/, int /*reason*/) override {
    if (s_state == HostState::Scanning) {
      // 짧게 쉬었다가 다시 스캔 (loop에서 처리)
    }
  }
} s_scan_cb;

// ===== Service discovery & subscribe =====
static bool subscribe_hid_inputs(NimBLERemoteService* hid) {
  bool any = false;
  // refresh=true 필수: NimBLE v2는 명시적으로 요청하지 않으면 char 전체를
  // discovery하지 않음. getCharacteristic(uuid)는 그 하나만 fetch.
  std::vector<NimBLERemoteCharacteristic*> chars = hid->getCharacteristics(true);

  // 진단용: HID service의 모든 char 덤프
  LOGF("[HID] service has %u characteristics:\n", (unsigned)chars.size());
  for (auto* c : chars) {
    LOGF("  UUID=%s handle=0x%04X props=%c%c%c%c%c\n",
         c->getUUID().toString().c_str(),
         c->getHandle(),
         c->canRead()            ? 'R' : '-',
         c->canWrite()           ? 'W' : '-',
         c->canWriteNoResponse() ? 'w' : '-',
         c->canNotify()          ? 'N' : '-',
         c->canIndicate()        ? 'I' : '-');
  }

  // 1) Protocol Mode를 Boot(0x00)로 강제 시도. 결과 로깅.
  if (NimBLERemoteCharacteristic* pm = hid->getCharacteristic(CHR_PROTO_MODE)) {
    uint8_t boot = 0x00;
    bool ok = false;
    if (pm->canWriteNoResponse()) ok = pm->writeValue(&boot, 1, false);
    else if (pm->canWrite())       ok = pm->writeValue(&boot, 1, true);
    LOGF("[HID] protocol mode = boot: %s\n", ok ? "OK" : "FAIL/skipped");
  } else {
    LOGF("[HID] no protocol mode char (0x2A4E) - keyboard likely report-only\n");
  }

  // 2-a) Bulk discovery 결과에서 notify char 구독. 단 boot-mouse(0x2A33)는
  //      이 호스트가 키보드 전용이라 구독 안 함 (마우스 리포트가 키로 오파싱되던
  //      원인 + 불필요한 트래픽/배터리 낭비).
  for (auto* c : chars) {
    if (!c->canNotify()) continue;
    if (c->getUUID() == CHR_BOOT_MOUSE_IN) {
      LOGF("[HID] skip boot-mouse 0x2A33 (keyboard-only host)\n");
      continue;
    }
    if (c->subscribe(true, on_notify)) {
      LOGF("[HID] subscribed UUID=%s handle=0x%04X\n",
           c->getUUID().toString().c_str(), c->getHandle());
      any = true;
    } else {
      LOGF("[HID] subscribe FAIL UUID=%s\n", c->getUUID().toString().c_str());
    }
  }

  // 2-b) Bulk이 실패해도 알려진 input UUID는 개별 lookup으로 한 번 더 시도.
  //      (이 키보드의 bulk discovery 응답이 비어있는 quirk 대응)
  auto try_known = [&](const NimBLEUUID& uuid, const char* tag) {
    NimBLERemoteCharacteristic* c = hid->getCharacteristic(uuid);
    if (!c || !c->canNotify()) return;
    // 이미 구독했으면 skip
    for (auto* x : chars) if (x == c) return;
    if (c->subscribe(true, on_notify)) {
      LOGF("[HID] subscribed (fallback) %s handle=0x%04X\n", tag, c->getHandle());
      any = true;
    }
  };
  try_known(CHR_BOOT_KBD_IN, "0x2A22 Boot Kbd Input");
  try_known(CHR_REPORT,      "0x2A4D Report");

  // 3) HID Control Point에 Exit Suspend(0x01) 전송.
  if (NimBLERemoteCharacteristic* hc = hid->getCharacteristic(CHR_HID_CTRL)) {
    uint8_t exit_suspend = 0x01;
    bool ok = false;
    if (hc->canWriteNoResponse()) ok = hc->writeValue(&exit_suspend, 1, false);
    else if (hc->canWrite())      ok = hc->writeValue(&exit_suspend, 1, true);
    LOGF("[HID] exit-suspend: %s\n", ok ? "OK" : "FAIL/skipped");
  }
  return any;
}

static void subscribe_battery(NimBLEClient* client) {
  NimBLERemoteService* bat = client->getService(SVC_BATT);
  if (!bat) return;
  NimBLERemoteCharacteristic* lvl = bat->getCharacteristic(CHR_BATT_LEVEL);
  if (!lvl) return;
  if (lvl->canRead()) {
    NimBLEAttValue v = lvl->readValue();
    if (v.length() >= 1 && s_cb_batt) {
      uint8_t pct = v.data()[0];
      LOGF("[BAT] level=%u%%\n", pct);
      s_cb_batt(pct);
    }
  }
  if (lvl->canNotify()) lvl->subscribe(true, on_notify);
}

// Synchronous connect to the target the scan just saw advertising (so it links
// fast — the block is brief). The scan MUST be fully stopped first; connecting
// mid-scan fails with EBUSY. 8 s timeout — NOTE NimBLE v2's setConnectTimeout is
// MILLISECONDS (the old `5` meant 5 ms, which made every connect fail instantly).
static bool do_connect() {
  if (!s_client) {
    s_client = NimBLEDevice::createClient();
    s_client->setClientCallbacks(&s_client_cb, false);
  }
  s_client->setConnectTimeout(8000);   // ms

  NimBLEScan* scan = NimBLEDevice::getScan();
  if (scan->isScanning()) {
    scan->stop();
    for (int i = 0; i < 60 && scan->isScanning(); i++) delay(5);   // wait until stopped
  }

  LOGF("[BLE] connecting to %s (cached=%d)\n",
       s_target_addr.toString().c_str(), s_attrs_cached);
  s_last_direct_try_ms = millis();

  bool delete_attrs = !s_attrs_cached;
  if (!s_client->connect(s_target_addr, delete_attrs)) {
    LOGF("[BLE] connect failed\n");
    return false;
  }
  if (!s_client->secureConnection()) {
    LOGF("[BLE] secureConnection failed\n");
  }
  if (!s_attrs_cached) {
    if (!s_client->discoverAttributes()) {
      LOGF("[BLE] discoverAttributes returned false (continuing)\n");
    }
  }
  NimBLERemoteService* hid = s_client->getService(SVC_HID);
  if (!hid) {
    LOGF("[BLE] no HID service\n");
    s_client->disconnect();
    return false;
  }
  if (!subscribe_hid_inputs(hid)) {
    LOGF("[BLE] no input subscription possible\n");
    s_client->disconnect();
    return false;
  }
  subscribe_battery(s_client);

  s_attrs_cached = true;          // 다음 연결부턴 디스커버리 스킵
  set_state(HostState::Connected, BleVisualState::Connected);
  LOGF("[BLE] connected & subscribed\n");
  return true;
}

static void start_scan() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&s_scan_cb, false);
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(50);
  scan->start(0, false);   // 무한 스캔
  set_state(HostState::Scanning, BleVisualState::Scanning);
  LOGF("[BLE] scan started\n");
}

// ===== Public API =====
void ble_host_begin(HidReportCb on_report, BleStateCb on_state, BatteryCb on_batt) {
  s_cb_report = on_report;
  s_cb_state  = on_state;
  s_cb_batt   = on_batt;

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(9);                          // +9 dBm
  // NoInputNoOutput + JustWorks. MITM은 NoIO와 호환 불가라서 반드시 false.
  NimBLEDevice::setSecurityAuth(true, false, true);   // bond, !mitm, sc
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  // 본드된 디바이스 있으면 scan 생략하고 바로 connect 시도
  int n = NimBLEDevice::getNumBonds();
  if (n > 0) {
    s_target_addr = NimBLEDevice::getBondedAddress(0);
    s_have_target = true;
    LOGF("[BLE] %d bond(s) on NVS. fast-reconnect to %s\n",
         n, s_target_addr.toString().c_str());
    set_state(HostState::ConnectPending, BleVisualState::Connecting);
  } else {
    set_state(HostState::Idle, BleVisualState::Init);
    start_scan();
  }
}

void ble_host_clear_bonds_and_restart() {
  LOGF("[BLE] clearing all bonds...\n");
  if (s_client && s_client->isConnected()) s_client->disconnect();
  NimBLEDevice::getScan()->stop();
  NimBLEDevice::deleteAllBonds();
  if (s_client) {
    NimBLEDevice::deleteClient(s_client);
    s_client = nullptr;
  }
  s_have_target = false;
  s_attrs_cached = false;                              // 캐시도 무효화
  set_state(HostState::Idle, BleVisualState::Init);
  delay(200);
  start_scan();
}

void ble_host_loop() {
  uint32_t now = millis();
  switch (s_state) {
    case HostState::Idle:
      if (now - s_last_state_change_ms > 1000) start_scan();
      break;
    case HostState::Scanning:
      // NimBLE가 알아서 콜백 보냄. 광고가 한 번에 안 잡힐 때 대비해서
      // 60초마다 스캔을 재시작 (일부 키보드는 directed adv 사용).
      if (now - s_last_state_change_ms > 60000) {
        NimBLEDevice::getScan()->stop();
        start_scan();
      }
      break;
    case HostState::ConnectPending:
      // 타겟이 있어도 바로 connect 안 하고 scan으로 키보드 재광고를 먼저 잡는다.
      // (async connect는 이 키보드 재광고를 못 잡고 timeout=13 났음; scan은 잡음.)
      start_scan();
      break;
    case HostState::Connecting:
      // scan이 키보드를 찾으면 여기로 옴 -> 동기 connect (방금 광고 중이라 빠름).
      if (s_have_target) {
        if (!do_connect()) {
          LOGF("[BLE] connect failed -> rescan\n");
          start_scan();
        }
      } else {
        start_scan();
      }
      break;
    case HostState::Connected:
      // 작동 중. NimBLE 콜백이 일을 함.
      break;
  }
}
