#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <Preferences.h>
#include <esp_system.h>       // esp_reset_reason()
#include "config.h"
#include "display.h"
#include "ble_hid_host.h"
#include "hangul_ime.h"
#include "hid_keymap.h"
#include "keymap_dubeolsik.h"
#include "lora.h"

// ===== Global state =====
// g_buffer/g_ime/g_hangul_mode/g_prev_* 는 main loop에서만 만진다.
// BLE 콜백은 raw HID report를 큐로만 보낸다 (race 회피).
static String       g_buffer;                            // 작성 중 (TX 뷰)
static String       g_history;                           // 송수신 로그 (RX 뷰)
static HangulIME    g_ime;
static bool         g_hangul_mode    = false;
static uint8_t      g_prev_mods      = 0;
static uint8_t      g_prev_keys[HID_MAX_KEYS] = {0};

// 화면 뷰 모드
enum ViewMode { VIEW_TX = 0, VIEW_RX = 1, VIEW_CONFIG = 2 };
static ViewMode     g_view = VIEW_TX;
static int          g_rx_scroll = 0;     // RX 뷰 스크롤 (라인 단위). 큰 수면 render가 마지막 페이지로 clamp.
static uint32_t     g_rx_count  = 0;     // 부팅 후 받은 메시지 수 (dialog 표시용)

// Sender ID (NVS 저장). 빈 문자열이면 prefix 없이 송신.
static String       g_sender_id;
static String       g_config_buf;       // config 모드에서 편집 중인 값

static volatile bool g_dirty = true;
static volatile bool g_clear_held = false;     // BLE 끊김 → held key state clear

// Serial 채널 명령 누적 ("Cnn\n" 형식)
static bool   g_serial_channel_mode = false;
static String g_serial_channel_buf;

// 키 반복용: 현재 눌려있는 키 슬롯 (release 또는 disconnect까지)
struct HeldKey {
  uint8_t  usage;            // 0 = 빈 슬롯
  uint8_t  shift;            // 첫 press 시점의 shift 상태
  uint32_t next_fire_ms;     // 다음 반복 발사 시각
};
static HeldKey g_held[HID_MAX_KEYS] = {};

// HID report 큐 (NimBLE task → main loop)
struct HidReportEvt { uint8_t mods; uint8_t keys[HID_MAX_KEYS]; };
static QueueHandle_t s_hid_q = nullptr;

// ===== Forward decls =====
static void handle_key(uint8_t usage, bool shift);
static void process_hid_report(uint8_t mods, const uint8_t* keys);
static void on_hid_report_cb(uint8_t mods, const uint8_t* keys);
static void on_ble_state(int s);
static void on_battery(uint8_t pct);
static void buffer_trim();
static void buffer_remove_last_utf8_char();
static void held_add(uint8_t usage, bool shift, uint32_t now);
static void held_remove(uint8_t usage);
static void held_clear();
static void held_tick(uint32_t now);
static void handle_send();
static void send_message(const String& text);
static void on_lora_msg(const String& text_in, const LoraRxInfo& info);
static void on_lora_conn(bool connected);
static void history_append(const char* prefix, const String& text);
static void toggle_view();
static void render_current_view();
static void load_sender_id();
static void save_sender_id(const String& id);
static void enter_config_mode();
static void exit_config_mode();
static void handle_config_key(uint8_t usage, bool shift);

// ===== Setup / Loop =====
void setup() {
  Serial.begin(115200);
  // Fire-and-forget logging: never block the loop on a full USB-CDC buffer.
  Serial.setTxTimeoutMs(0);
  delay(300);
  LOGF("\n==== pager-lora-seeeed / XIAO-S3 + Wio-SX1262 / BLE HID Host ====\n");
  LOGF("[BOOT] reset_reason=%d (1=POR 4=PANIC 5=INT_WDT 6=TASK_WDT 7=WDT 9=BROWNOUT 11=USB)\n",
       (int)esp_reset_reason());
  LOGF("Keys:  LAlt=Kor/Eng  Cmd+Enter=send  Cmd+.=TX/RX  Ctrl+.=Config\n");
  LOGF("Serial commands:\n");
  LOGF("  `       toggle hangul mode\n");
  LOGF("  ~       clear all BLE bonds and rescan\n");
  LOGF("  Cnn\\n   (no-op on SX1262 — freq fixed in lora_rf.h)\n");
  LOGF("  X       dump SX1262 RF config + pin map\n");
  LOGF("  H       dump SX1262 RF config (same as X)\n");
  LOGF("  R       show last RX packet RSSI/SNR\n");
  LOGF("  N       dump heard-node table + dedup stats (discovery)\n");
  LOGF("  B       (n/a on SX1262 — no UART bridge)\n");

  s_hid_q = xQueueCreate(16, sizeof(HidReportEvt));

  load_sender_id();
  LOGF("[CFG] sender_id=\"%s\"\n", g_sender_id.c_str());

  display_begin();
  display_set_hangul_mode(g_hangul_mode);
  display_set_ble_state(BleVisualState::Init);
  render_current_view();

  lora_begin();
  lora_set_callbacks(on_lora_msg, on_lora_conn);
  lora_set_my_id(g_sender_id);     // BEACON에 포함시킬 우리 ID
  display_set_lora_state(lora_connected());
  ble_host_begin(on_hid_report_cb, on_ble_state, on_battery);
}

void loop() {
  ble_host_loop();

  if (g_clear_held) {
    g_clear_held = false;
    held_clear();
    memset(g_prev_keys, 0, sizeof(g_prev_keys));
    g_prev_mods = 0;
    // ★ 큐에 쌓여있던 "눌림" 이벤트들(연결이 끊겨 release는 영영 안 옴)이 아래
    //   드레인 루프에서 처리되며 g_held를 도로 채워 held_tick이 무한반복(버블링)함.
    //   held를 지울 땐 이 stale 큐도 반드시 비운다.
    if (s_hid_q) xQueueReset(s_hid_q);
  }

  // HID 이벤트 큐 드레인
  HidReportEvt evt;
  while (s_hid_q && xQueueReceive(s_hid_q, &evt, 0) == pdTRUE) {
    process_hid_report(evt.mods, evt.keys);
  }

  // 키 반복 tick
  held_tick(millis());

  // LoRa RX/HB 처리
  lora_tick();

  // Serial 입력
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;

    // 채널 명령 모드: "Cnn\n" 누적
    if (g_serial_channel_mode) {
      if (c == '\r' || c == '\n') {
        int ch = g_serial_channel_buf.toInt();
        g_serial_channel_mode = false;
        g_serial_channel_buf = "";
        if (ch > 0 && ch < 128) {
          Serial.printf("[CMD] LoRa channel -> %d\n", ch);
          lora_set_channel((uint8_t)ch);
        } else {
          Serial.println("[CMD] invalid channel (1..127)");
        }
      } else if (c >= '0' && c <= '9') {
        g_serial_channel_buf += (char)c;
      }
      continue;
    }
    if (c == 'C') {
      g_serial_channel_mode = true;
      g_serial_channel_buf = "";
      Serial.println("[CMD] channel? (e.g. 90<Enter>)");
      continue;
    }
    if (c == 'X') {
      Serial.println("[CMD] LoRa RF config dump");
      lora_probe_at();
      continue;
    }
    if (c == 'H') {
      Serial.println("[CMD] LoRa RF config dump");
      lora_query_at_help();
      continue;
    }
    if (c == 'R') {
      Serial.println("[CMD] LoRa last-packet RSSI/SNR");
      lora_query_rssi();
      continue;
    }
    if (c == 'N') {
      Serial.println("[CMD] LoRa neighbor dump");
      lora_dump_neighbors();
      continue;
    }
    if (c == 'B') {
      Serial.println("[CMD] LoRa bridge (n/a on SX1262)");
      lora_bridge_loop();
      continue;
    }

    if (c == '~') {
      Serial.println("[CMD] clearing bonds...");
      ble_host_clear_bonds_and_restart();
    } else if (c == '`') {
      g_buffer += g_ime.commit_all();
      g_hangul_mode = !g_hangul_mode;
      display_set_hangul_mode(g_hangul_mode);
    } else if (c == 8 || c == 127) {
      auto r = g_ime.backspace();
      if (r.remove_buffer_char) buffer_remove_last_utf8_char();
    } else if (c == '\r' || c == '\n') {
      g_buffer += g_ime.commit_all();
      g_buffer += '\n';
    } else if (c >= 'a' && c <= 'z' && g_hangul_mode) {
      uint16_t j = dubeolsik_lookup((char)c, false);
      if (j) g_buffer += g_ime.input_jamo(j);
    } else if (c >= 'A' && c <= 'Z' && g_hangul_mode) {
      uint16_t j = dubeolsik_lookup((char)(c - 'A' + 'a'), true);
      if (j) g_buffer += g_ime.input_jamo(j);
    } else if (c >= 32 && c < 127) {
      g_buffer += g_ime.commit_all();
      g_buffer += (char)c;
    }
    buffer_trim();
    g_dirty = true;
  }

  // 다이얼로그 만료 → 본문으로 복귀 redraw 트리거
  if (display_dialog_consume_expiry()) g_dirty = true;

  // 비동기 LoRa 송신 완료 → "송신됨" 표시
  if (lora_tx_consume_done()) {
    display_show_dialog("송신됨", 1000);
    g_dirty = true;
  }

  // Range 테스트: T-Deck PING에 PONG 응답할 때마다 화면에 표시 (응답상황)
  {
    int r_seq; uint32_t r_cnt;
    if (lora_consume_range(&r_seq, &r_cnt)) {
      char buf[24];
      // 128px OLED 한 줄(unifont ~8px/자) → "Range " 빼고 짧게 (≈12자).
      snprintf(buf, sizeof(buf), "PONG #%d (%lu)", r_seq, (unsigned long)r_cnt);
      display_show_dialog(buf, 2500);
      g_dirty = true;
    }
  }

  if (g_dirty) {
    g_dirty = false;
    render_current_view();
  }

  delay(2);
}

// ===== BLE notification callback (NimBLE task 컨텍스트) =====
// 절대 g_buffer/g_ime/I2C 만지지 말 것. 큐로만 넘긴다.
static void on_hid_report_cb(uint8_t mods, const uint8_t* keys) {
  if (!s_hid_q) return;
  HidReportEvt evt;
  evt.mods = mods;
  memcpy(evt.keys, keys, HID_MAX_KEYS);
  xQueueSend(s_hid_q, &evt, 0);    // queue full이면 그냥 drop
}

// ===== Main-loop HID 처리 =====
static void process_hid_report(uint8_t mods, const uint8_t* keys) {
  // Left Alt press edge → 한/영 토글. config 모드면 config_buf로 commit.
  if ((mods & MOD_LALT) && !(g_prev_mods & MOD_LALT)) {
    if (g_view == VIEW_CONFIG) g_config_buf += g_ime.commit_all();
    else                       g_buffer      += g_ime.commit_all();
    g_hangul_mode = !g_hangul_mode;
    display_set_hangul_mode(g_hangul_mode);
    LOGF("[MODE] hangul=%s\n", g_hangul_mode ? "ON" : "OFF");
  }
  g_prev_mods = mods;

  bool shift = (mods & (MOD_LSHIFT | MOD_RSHIFT)) != 0;
  bool ctrl  = (mods & (MOD_LCTRL  | MOD_RCTRL )) != 0;
  bool gui   = (mods & (MOD_LGUI   | MOD_RGUI  )) != 0;
  uint32_t now = millis();

  // 1) 새로 눌린 키 = current에는 있고 prev에는 없는 것
  for (int i = 0; i < HID_MAX_KEYS; i++) {
    uint8_t k = keys[i];
    if (k == 0) continue;
    bool was = false;
    for (int j = 0; j < HID_MAX_KEYS; j++) {
      if (g_prev_keys[j] == k) { was = true; break; }
    }
    if (was) continue;

    // Cmd(GUI) + Enter → 전송
    if (gui && k == HID_ENTER) {
      handle_send();
      continue;
    }
    // Cmd(GUI) + . → TX/RX 뷰 토글
    if (gui && k == HID_PERIOD) {
      toggle_view();
      continue;
    }
    // Ctrl + . → Config 메뉴 토글
    if (ctrl && k == HID_PERIOD) {
      if (g_view == VIEW_CONFIG) exit_config_mode();
      else                       enter_config_mode();
      continue;
    }
    // Config 뷰: 키 전용 처리
    if (g_view == VIEW_CONFIG) {
      handle_config_key(k, shift);
      continue;
    }
    if (ctrl) continue;
    handle_key(k, shift);
    held_add(k, shift, now);
  }

  // 2) release된 키 = prev에는 있고 current에는 없는 것
  for (int i = 0; i < HID_MAX_KEYS; i++) {
    uint8_t pk = g_prev_keys[i];
    if (pk == 0) continue;
    bool still = false;
    for (int j = 0; j < HID_MAX_KEYS; j++) {
      if (keys[j] == pk) { still = true; break; }
    }
    if (!still) held_remove(pk);
  }

  memcpy(g_prev_keys, keys, HID_MAX_KEYS);
  buffer_trim();
  g_dirty = true;
}

// ===== Key repeat =====
static void held_add(uint8_t usage, bool shift, uint32_t now) {
  // 이미 들어있으면 갱신만
  for (auto& h : g_held) {
    if (h.usage == usage) {
      h.shift = shift;
      h.next_fire_ms = now + KEY_REPEAT_INITIAL_MS;
      return;
    }
  }
  for (auto& h : g_held) {
    if (h.usage == 0) {
      h.usage = usage;
      h.shift = shift;
      h.next_fire_ms = now + KEY_REPEAT_INITIAL_MS;
      return;
    }
  }
}

static void held_remove(uint8_t usage) {
  for (auto& h : g_held) {
    if (h.usage == usage) h.usage = 0;
  }
}

static void held_clear() {
  for (auto& h : g_held) h.usage = 0;
}

static void held_tick(uint32_t now) {
  bool fired = false;
  for (auto& h : g_held) {
    if (h.usage == 0) continue;
    if ((int32_t)(now - h.next_fire_ms) < 0) continue;
    handle_key(h.usage, h.shift);
    h.next_fire_ms = now + KEY_REPEAT_RATE_MS;
    fired = true;
  }
  if (fired) {
    buffer_trim();
    g_dirty = true;
  }
}

static void handle_key(uint8_t usage, bool shift) {
  LOGF("[KEY] 0x%02X shift=%d hangul=%d\n", usage, shift, g_hangul_mode);

  switch (usage) {
    case HID_BACKSPACE: {
      auto r = g_ime.backspace();
      if (r.remove_buffer_char) buffer_remove_last_utf8_char();
      return;
    }
    case HID_ENTER:
      g_buffer += g_ime.commit_all();
      g_buffer += '\n';
      return;
    case HID_SPACE:
      g_buffer += g_ime.commit_all();
      g_buffer += ' ';
      return;
    case HID_TAB:
      g_buffer += g_ime.commit_all();
      g_buffer += ' ';
      return;
    case HID_LANG1:
    case HID_LANG2:
      g_buffer += g_ime.commit_all();
      g_hangul_mode = !g_hangul_mode;
      display_set_hangul_mode(g_hangul_mode);
      return;
    case HID_ESC:
      g_buffer += g_ime.commit_all();
      return;
    case HID_UP:
      if (g_view == VIEW_RX) {
        g_rx_scroll -= TEXT_LINES;
        if (g_rx_scroll < 0) g_rx_scroll = 0;
      }
      return;
    case HID_DOWN:
      if (g_view == VIEW_RX) {
        g_rx_scroll += TEXT_LINES;             // 상한은 render에서 clamp
      }
      return;
    default:
      break;
  }

  if (g_hangul_mode && usage >= HID_A && usage <= HID_Z) {
    char q = 'a' + (usage - HID_A);
    uint16_t j = dubeolsik_lookup(q, shift);
    if (j) g_buffer += g_ime.input_jamo(j);
    return;
  }

  char c = hid_to_ascii(usage, shift);
  if (c) {
    g_buffer += g_ime.commit_all();
    g_buffer += c;
  }
}

// ===== Send (Cmd+Enter) =====
static void send_message(const String& text) {
  LOGF("[SEND] len=%u text=\"%s\"\n", (unsigned)text.length(), text.c_str());
  lora_send_message(text);
}

static void handle_send() {
  g_buffer += g_ime.commit_all();
  if (g_buffer.length() == 0) return;

  // 송신은 prefix 붙여서 (수신측이 누가 보냈는지 알 수 있게).
  String to_send = g_buffer;
  if (g_sender_id.length() > 0) {
    to_send = "[" + g_sender_id + "] " + to_send;
  }

  // 1) "송신중" dialog 표시.
  display_show_dialog("송신중...", 30000);
  render_current_view();

  // 2) 송신은 비동기 — lora_tx_task가 백그라운드에서 보냄. 즉시 반환하므로 메인
  //    루프가 안 막힌다(인터럽트 워치독/프리즈/BLE 끊김 방지). 완료되면 아래
  //    loop()의 lora_tx_consume_done()가 "송신됨"으로 교체.
  send_message(to_send);

  // 내 history엔 원본만 (내 ID는 굳이 다시 안 봐도 됨)
  history_append("> ", g_buffer);

  g_buffer = "";
  g_ime.reset();
  held_clear();
  g_dirty = true;
}

// ===== View / History =====
static void toggle_view() {
  g_view = (g_view == VIEW_TX) ? VIEW_RX : VIEW_TX;
  display_set_view_label(g_view == VIEW_TX ? "TX" : "RX");
  if (g_view == VIEW_RX) g_rx_scroll = 0x7FFFFFFF;   // 다음 render에서 마지막 페이지로 clamp
  LOGF("[VIEW] %s\n", g_view == VIEW_TX ? "TX" : "RX");
  g_dirty = true;
}

static void render_current_view() {
  if (g_view == VIEW_CONFIG) {
    String shown = g_config_buf;
    shown += g_ime.preview();
    display_render_config("Sender ID:", shown);
    return;
  }
  if (g_view == VIEW_TX) {
    display_render(g_buffer, g_ime.preview());
  } else {
    // RX: 스크롤 적용 + render가 clamp한 값을 다음 페이지 이동에 쓰도록 저장
    g_rx_scroll = display_render_history(g_history, g_rx_scroll);
  }
}

// ===== NVS Sender ID =====
static void load_sender_id() {
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, true);          // read-only
  g_sender_id = prefs.getString("sender_id", "");
  prefs.end();
}

static void save_sender_id(const String& id) {
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString("sender_id", id);
  prefs.end();
  g_sender_id = id;
}

// ===== Config mode =====
static void enter_config_mode() {
  // 작성 중인 한글 자모 commit해서 메인 버퍼로 보존하고 IME 리셋
  g_buffer += g_ime.commit_all();
  g_ime.reset();

  g_view = VIEW_CONFIG;
  g_config_buf = g_sender_id;
  display_set_view_label("CFG");
  held_clear();
  LOGF("[CFG] enter (current id=\"%s\")\n", g_sender_id.c_str());
  g_dirty = true;
}

static void exit_config_mode() {
  g_ime.reset();                             // config 중 IME 상태 비움
  g_view = VIEW_TX;
  display_set_view_label("TX");
  g_config_buf = "";
  g_dirty = true;
}

// config 모드에서 byte 단위 UTF-8 마지막 글자 삭제
static void config_buf_remove_last_char() {
  int n = (int)g_config_buf.length();
  if (n == 0) return;
  do { n--; }
  while (n > 0 && ((uint8_t)g_config_buf.charAt(n) & 0xC0) == 0x80);
  g_config_buf.remove(n);
}

static void handle_config_key(uint8_t usage, bool shift) {
  if (usage == HID_ESC) {
    LOGF("[CFG] cancel\n");
    exit_config_mode();
    return;
  }
  if (usage == HID_ENTER) {
    g_config_buf += g_ime.commit_all();      // 조합 중인 자모 확정
    save_sender_id(g_config_buf);
    LOGF("[CFG] saved id=\"%s\"\n", g_sender_id.c_str());
    exit_config_mode();
    display_show_dialog("ID saved", 1000);
    return;
  }
  if (usage == HID_BACKSPACE) {
    auto r = g_ime.backspace();
    if (r.remove_buffer_char) config_buf_remove_last_char();
    g_dirty = true;
    return;
  }

  // 길이 초과면 더 안 받음 (한글 1자 = 3바이트 여유 보고 컷)
  if ((int)g_config_buf.length() + 4 > MAX_SENDER_ID_BYTES) return;

  // 한글 모드 + 알파벳 → IME로
  if (g_hangul_mode && usage >= HID_A && usage <= HID_Z) {
    char q = 'a' + (usage - HID_A);
    uint16_t j = dubeolsik_lookup(q, shift);
    if (j) g_config_buf += g_ime.input_jamo(j);
    g_dirty = true;
    return;
  }

  // ASCII (영문/숫자/_-만 허용. 공백/특수문자 제외)
  char c = hid_to_ascii(usage, shift);
  if (c == 0) return;
  bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '_' || c == '-';
  if (ok) {
    g_config_buf += g_ime.commit_all();      // 한글 조합 중이면 먼저 commit
    g_config_buf += c;
    g_dirty = true;
  }
}

// history에 한 줄 추가. prefix는 "> " (송신) 또는 "< " (수신).
// 한도(HISTORY_BUF_BYTES) 초과 시 앞쪽부터 줄 단위로 trim (메시지 중간이 잘리지 않도록).
static void history_append(const char* prefix, const String& text) {
  if (g_history.length() > 0) g_history += '\n';
  g_history += prefix;
  g_history += text;
  // 한도 넘었으면 한 줄씩 통째로 drop (다음 \n까지 한꺼번에 잘라냄)
  while ((int)g_history.length() > HISTORY_BUF_BYTES) {
    int nl = g_history.indexOf('\n');
    if (nl < 0) {
      // 단일 메시지가 buf보다 큰 극단적 케이스 — UTF-8 안전하게 앞쪽 1자만 drop
      int n = 1;
      while (n < (int)g_history.length() &&
             ((uint8_t)g_history.charAt(n) & 0xC0) == 0x80) n++;
      g_history.remove(0, n);
    } else {
      g_history.remove(0, nl + 1);   // \n 포함해서 제거
    }
  }
}

// ===== Buffer utilities =====
static void buffer_remove_last_utf8_char() {
  int n = (int)g_buffer.length();
  if (n == 0) return;
  do {
    n--;
  } while (n > 0 && ((uint8_t)g_buffer.charAt(n) & 0xC0) == 0x80);
  g_buffer.remove(n);
}

static void buffer_trim() {
  while (g_buffer.length() > TEXT_BUF_BYTES) {
    int n = 1;
    while (n < (int)g_buffer.length() &&
           ((uint8_t)g_buffer.charAt(n) & 0xC0) == 0x80) n++;
    g_buffer.remove(0, n);
  }
}

// ===== LoRa callbacks =====
static void on_lora_msg(const String& text_in, const LoraRxInfo& info) {
  // [EOF] 없이 idle 만료/강제 종료된 메시지는 잘렸다고 표시해서 보여준다 (PROTOCOL §5 v1.19).
  String text = info.partial ? text_in + " [잘림]" : text_in;
  LOGF("[RX] from=%s rssi=%d hops=%d ttl=%u%s \"%s\"\n", info.src, info.rssi, info.hops,
       (unsigned)info.ttl, info.partial ? " PARTIAL" : "", text.c_str());
  history_append("< ", text);
  g_rx_count++;

  // 내용 대신 카운터만 dialog로 표시 — 빠르게 사라지는 알림 용도.
  // 실제 내용은 RX 뷰(history)에서 확인.
  char buf[32];
  snprintf(buf, sizeof(buf), "%lu번째 수신", (unsigned long)g_rx_count);
  display_show_dialog(buf, 1200);

  // 새 메시지의 첫 라인으로 스크롤 jump.
  // append 후 history에서 "< " + text 가 마지막 부분이므로,
  // 그 직전 byte까지의 wrap line 개수를 첫 번째로 보여줄 라인 인덱스로 사용.
  size_t new_msg_len = 2 + text.length();   // "< " + text
  size_t new_msg_byte_start = (g_history.length() > new_msg_len)
                              ? (g_history.length() - new_msg_len) : 0;
  g_rx_scroll = display_wrap_lines_before(g_history, new_msg_byte_start);

  if (g_view != VIEW_CONFIG && g_view != VIEW_RX) {
    g_view = VIEW_RX;
    display_set_view_label("RX");
  }
  g_dirty = true;
}

static void on_lora_conn(bool connected) {
  LOGF("[LORA] link %s\n", connected ? "up" : "down");
  display_set_lora_state(connected);
  g_dirty = true;
}

// ===== BLE state / battery callbacks =====
static void on_ble_state(int s) {
  BleVisualState st = (BleVisualState)s;
  display_set_ble_state(st);

  switch (st) {
    case BleVisualState::Scanning:
    case BleVisualState::Connecting:
      // 슬립 깨어난 키보드는 패킷 처리 중이라 사용자가 계속 키 누르면
      // BLE 핸드셰이크 못 끝냄. "한 번 톡 누르고 손 떼" 안내를 길게 표시.
      display_show_dialog("Tap key, wait...", 30000);
      break;
    case BleVisualState::Connected:
      display_show_dialog("Connected!", 1500);
      break;
    case BleVisualState::Init:
    case BleVisualState::Disconnected:
    default:
      break;
  }

  if (st != BleVisualState::Connected) {
    // 연결이 안 된 모든 상태(특히 본드 키보드가 끊겼을 때 거치는 Connecting)에서
    // 눌린 키 강제 해제. 안 그러면 키를 누른 채 끊겨 release를 못 받은 키가
    // g_held에 남아 held_tick이 영원히 재발사함 (유령 v/f 키).
    g_clear_held = true;
  }
  g_dirty = true;
}

static void on_battery(uint8_t pct) {
  display_set_battery_pct((int)pct);
  g_dirty = true;
}
