#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_system.h>       // esp_reset_reason()
#include "config.h"
#include "lora.h"
#include "bme280.h"
#include "bmp390.h"
#include "sht3x.h"
#include "ble_dash.h"
#include "driver/gpio.h"   // gpio_reset_pin (I2C 진단), gpio_hold_en (딥슬립)
#include <esp_sleep.h>
#include <time.h>
#if !ARDUINO_USB_MODE
#include "tusb.h"             // tud_mounted / tud_suspended (USB-OTG/TinyUSB 모드)
#endif

// ===== 차량 페이저 (P01) =====
// 하는 일 네 가지:
//   1. SHT3x / BME280 / BMP3xx로 실내 온습도(기압) 측정
//   2. 들리는 LoRa 노드 전부를 discovery 테이블로 유지 (lora.cpp)
//   3. 주차 중(LiPo 구동)엔 !CAR 상태 비콘을 주기 송신 (PROTOCOL_CAR.md)
//   4. 폰의 Bluefy 웹 대시보드에 BLE로 붙어 discovery / 비콘 / 채팅 제공
// 모든 상태는 메인 루프에서만 만진다. BLE 콜백은 ble_dash.cpp가 큐로 넘겨준다.

static String   g_display_id;
static uint32_t g_boot_cpu_mhz = 240;

// ---- 딥슬립을 건너 살아남는 것들 (RTC slow memory) ----
RTC_DATA_ATTR static uint32_t g_rtc_boots        = 0;
RTC_DATA_ATTR static time_t   g_rtc_first_boot   = 0;    // RTC 시계는 딥슬립 중에도 간다 → 가동시간 기준
RTC_DATA_ATTR static uint32_t g_rtc_beacon_count = 0;
RTC_DATA_ATTR static bool     g_rtc_was_parked   = false;
RTC_DATA_ATTR static time_t   g_rtc_next_bcn     = 0;    // 주차 중 다음 !CAR 예정 (RTC 벽시계). 어떤 이유로 깼든 지났으면 보낸다
static esp_sleep_wakeup_cause_t g_wake_cause = ESP_SLEEP_WAKEUP_UNDEFINED;
static bool     g_woke_from_sleep = false;
static uint32_t g_awake_until_ms  = 0;       // 주차 중 이 시각이 지나면 딥슬립
static const char* g_awake_why    = "";
static uint32_t g_full_until_ms   = 0;       // "온전히 깨어 있음"(HB·광고 정상) 끝나는 시각. 짧은 wake 창은 여기 안 들어간다

// 전원/주차 상태
static bool     g_parked        = false;
static bool     g_power_raw     = true;      // 디바운스 전 "차량 전원 있음"
static uint32_t g_power_raw_ms  = 0;

// 센서
static Bme280Reading g_env = {};
static uint32_t g_next_sensor_ms = 0;
static int      g_vbat_mv = -1;           // -1 = 측정 불가 (분압 배선 없음)
static float    g_vbat_filt = 0;
static bool     g_batt_low_sent = false;

// 비콘
static uint32_t g_next_beacon_ms = 0;        // 0 = 예약 없음
static String   g_last_beacon;
#define g_beacon_count g_rtc_beacon_count      // 딥슬립을 건너 누적

// 대시보드 push
static uint32_t g_next_status_ms = 0;
static uint32_t g_next_nodes_ms  = 0;
static bool     g_nodes_dirty    = true;

// 폰이 없을 때 받은 것들을 보관했다가 연결 시 재생 (RAM only)
struct HistEntry { String body; uint32_t ms; };   // body = 중괄호 없는 JSON 필드들
static HistEntry g_hist[VEH_CHAT_HISTORY];
static uint8_t   g_hist_head = 0, g_hist_count = 0;

// Serial 디버그 입력 ("M<text>\n" = 채팅 송신)
static String g_serial_line;
static bool   g_serial_msg_mode = false;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
// UTF-8 글자 중간을 자르지 않고 max_bytes 이하로 줄인다.
static String utf8_truncate(const String& s, size_t max_bytes) {
  if (s.length() <= max_bytes) return s;
  size_t n = max_bytes;
  while (n > 0 && ((uint8_t)s[n] & 0xC0) == 0x80) n--;
  return s.substring(0, n);
}

static String num_or_null(bool ok, float v, int decimals) {
  return (ok && !isnan(v)) ? String(v, (unsigned int)decimals) : String("null");
}

static void hist_push(const String& body) {
  HistEntry& e = g_hist[(g_hist_head + g_hist_count) % VEH_CHAT_HISTORY];
  e.body = body;
  e.ms = millis();
  if (g_hist_count < VEH_CHAT_HISTORY) g_hist_count++;
  else g_hist_head = (g_hist_head + 1) % VEH_CHAT_HISTORY;
}

// 이벤트 하나: 히스토리에 넣고, 폰이 붙어 있으면 바로 보낸다.
static void dash_event(const String& body) {
  hist_push(body);
  ble_dash_send_line("{" + body + ",\"age\":0}");
}

static String ls_resumed_note() {
  LoraStats ls; lora_get_stats(&ls);
  return ls.resumed_len ? " (woke by a " + String((unsigned long)ls.resumed_len) + " B frame, read from RX buffer)" : String("");
}

static uint32_t uptime_s() {
  time_t now = time(nullptr);
  if (g_rtc_first_boot == 0 || now < g_rtc_first_boot) g_rtc_first_boot = now;
  return (uint32_t)(now - g_rtc_first_boot);
}

// 깨어 있기: 지금부터 최소 ms 동안은 슬립하지 않는다 (더 긴 예약이 있으면 그대로).
// full=true: 폰 연결·채팅·주소지정 PING·주차 직후처럼 "사람이 이 노드를 찾는" 각성 → HB 정상.
// full=false: 타이머 비콘 창 / 남의 프레임으로 깬 짧은 창 → HB 끔.
static void stay_awake(uint32_t ms, const char* why, bool full = true) {
  uint32_t until = millis() + ms;
  if ((int32_t)(until - g_awake_until_ms) > 0) { g_awake_until_ms = until; g_awake_why = why; }
  if (full && (int32_t)(until - g_full_until_ms) > 0) g_full_until_ms = until;
}

// ---------------------------------------------------------------------------
// 전원 / 주차 감지
// ---------------------------------------------------------------------------
static bool vehicle_power_present() {
#if VEH_VBUS_SENSE_PIN >= 0
  return digitalRead(VEH_VBUS_SENSE_PIN) == HIGH;
#elif ARDUINO_USB_MODE
  return HWCDC::isPlugged();                       // USB-Serial-JTAG: SOF 타이머 기반
#else
  return tud_mounted() && !tud_suspended();        // TinyUSB: 호스트가 열거했고 버스가 살아 있음
#endif
}

static void read_vbat() {
#if VEH_VBAT_ADC_PIN >= 0
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) { acc += analogReadMilliVolts(VEH_VBAT_ADC_PIN); delayMicroseconds(300); }  // 고임피던스 분압: 샘플 사이 콘덴서 회복 시간
  float mv = (float)acc / 16.0f * VEH_VBAT_DIVIDER * VEH_VBAT_CAL;
  if (mv < 2500.0f || mv > 4500.0f) {            // 1셀 LiPo일 수 없는 값 = 핀이 떠 있다
    g_vbat_mv = -1;
    g_vbat_filt = 0;
    return;
  }
  // LoRa 송신 순간의 전압 강하로 %가 출렁이지 않게 완만히 따라간다 (10 s 주기 × 0.25).
  g_vbat_filt = (g_vbat_filt > 0) ? g_vbat_filt * 0.75f + mv * 0.25f : mv;
  g_vbat_mv = (int)lroundf(g_vbat_filt);
#endif
}

// 1셀 LiPo 개방전압 → 잔량 %. 부하가 가벼워서(수십 mA) OCV 곡선을 그대로 쓴다.
// USB로 충전 중일 땐 충전 전압이 읽혀 실제보다 높게 나온다 — 대시보드가 "충전 중"으로 구분.
static int battery_pct(int mv) {
  static const struct { int mv, pct; } k[] = {
    {4200, 100}, {4150, 95}, {4110, 90}, {4080, 85}, {4020, 80}, {3980, 75}, {3950, 70},
    {3910, 65},  {3870, 60}, {3850, 55}, {3840, 50}, {3820, 45}, {3800, 40}, {3790, 35},
    {3770, 30},  {3750, 25}, {3730, 20}, {3710, 15}, {3690, 10}, {3610, 5},  {3270, 0}};
  if (mv < 0) return -1;
  if (mv >= k[0].mv) return 100;
  for (size_t i = 1; i < sizeof(k) / sizeof(k[0]); i++) {
    if (mv >= k[i].mv)
      return k[i].pct + (int)lroundf((float)(mv - k[i].mv) * (k[i - 1].pct - k[i].pct) / (k[i - 1].mv - k[i].mv));
  }
  return 0;
}
static bool batt_low() { int p = battery_pct(g_vbat_mv); return g_parked && p >= 0 && p <= VEH_BATT_LOW_PCT; }

static void schedule_beacon(uint32_t in_ms) { g_next_beacon_ms = millis() + in_ms; if (!g_next_beacon_ms) g_next_beacon_ms = 1; }

static void apply_parked(bool parked, bool announce) {
  g_parked = parked;
  LOGF("[PWR] %s\n", parked ? "PARKED — on LiPo, status beacon on" : "DRIVING — vehicle power present");
  setCpuFrequencyMhz(parked ? VEH_PARKED_CPU_MHZ : g_boot_cpu_mhz);
  ble_dash_set_slow_adv(parked);
  // 상태가 바뀌면 한 발 쏜다: 주차 시작도, "차가 출발했다"도 지켜보는 쪽엔 뉴스다.
  // 그 뒤로는 주기 설정(주행 0 = 끔)을 따른다.
  if (announce) schedule_beacon(parked ? 15000 : 5000);
  if (parked && announce) stay_awake(VEH_AWAKE_MS, "parked");   // 방금 세운 차(실제 전환) — 30분은 폰이 붙을 수 있게 깨어 있는다.
                                                                 // 슬립에서 깨며 주차로 복원될 땐(announce=false) setup()이 짧은 창만 준다.
  g_next_status_ms = 0;
}

static void power_tick(uint32_t now) {
  bool raw = vehicle_power_present();
  if (raw != g_power_raw) { g_power_raw = raw; g_power_raw_ms = now; }
  bool want_parked = !g_power_raw;
  if (want_parked != g_parked && (uint32_t)(now - g_power_raw_ms) >= VEH_POWER_DEBOUNCE_MS)
    apply_parked(want_parked, true);
}

// ---------------------------------------------------------------------------
// 센서 / 비콘
// ---------------------------------------------------------------------------
// 환경 센서: BME280/BMP280 또는 BMP390/388 중 버스에 있는 쪽. 늦게 꽂혀도 10초마다 다시 찾는다.
static const char* g_env_name = nullptr;          // nullptr = 아직 못 찾음
enum class EnvKind : uint8_t { None, Bme, Bmp3, Sht };
static EnvKind     g_env_kind = EnvKind::None;
// 센서 제어 핀을 먼저 잡는다. setup() 맨 앞과, 매 probe 전에도 다시 확인(값이 흔들릴 일은
// 없지만 비용이 0이고, 나중에 핀을 잘못 건드리는 코드가 들어와도 안전).
static void env_pins_init() {
#if VEH_ENV_CSB_PIN >= 0
  pinMode(VEH_ENV_CSB_PIN, OUTPUT); digitalWrite(VEH_ENV_CSB_PIN, HIGH);
#endif
#if VEH_ENV_SDO_PIN >= 0
  pinMode(VEH_ENV_SDO_PIN, OUTPUT); digitalWrite(VEH_ENV_SDO_PIN, VEH_ENV_SDO_LEVEL);
#endif
}
static void env_probe() {
  env_pins_init();
  delay(2);
  Bme280Chip c;
  if (sht3x_begin()) { g_env_name = "SHT3x"; g_env_kind = EnvKind::Sht; }
  else if ((c = bme280_begin(BME280_ADDR)) != Bme280Chip::None) {
    g_env_name = (c == Bme280Chip::BME280) ? "BME280" : "BMP280"; g_env_kind = EnvKind::Bme;
  } else {
    const char* n = nullptr;
#if VEH_ENV_CSB_PIN >= 0 && VEH_ENV_SDO_PIN >= 0
    Wire.end();                                   // SDA/SCL 선을 SPI MOSI/SCK로 쓴다
    n = bmp390_begin_spi(VEH_ENV_CSB_PIN, I2C_SCL_PIN, I2C_SDA_PIN, VEH_ENV_SDO_PIN);
    if (!n) { env_pins_init(); Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); Wire.setTimeOut(20); }
#endif
    if (!n) n = bmp390_begin();
    if (!n) return;
    g_env_name = n; g_env_kind = EnvKind::Bmp3;
  }
  LOGF("[ENV] sensor: %s%s%s\n", g_env_name, g_env_kind == EnvKind::Bmp3 || !strcmp(g_env_name, "BMP280") ? " (no humidity)" : "", g_env_kind == EnvKind::Bmp3 ? " via SPI" : "");
}
static Bme280Reading env_read() {
  if (!g_env_name) return Bme280Reading{};
  Bme280Reading r = g_env_kind == EnvKind::Sht ? sht3x_read() : g_env_kind == EnvKind::Bmp3 ? bmp390_read() : bme280_read();
  if (!r.ok) g_env_name = nullptr;                // 뽑혔거나 버스 오류 → 다음 주기에 다시 probe
  return r;
}

static bool env_hot() { return g_env.ok && g_env.temp_c >= VEH_TEMP_WARN_C; }

static void sensor_tick(uint32_t now) {
  if ((int32_t)(now - g_next_sensor_ms) < 0) return;
  g_next_sensor_ms = now + VEH_SENSOR_MS;
  if (!g_env_name) env_probe();
  bool was_hot = env_hot();
  g_env = env_read();
  read_vbat();
  if (batt_low() && !g_batt_low_sent) {
    g_batt_low_sent = true;
    LOGF("[BATT] low: %d mV (%d%%)\n", g_vbat_mv, battery_pct(g_vbat_mv));
    schedule_beacon(1000);               // 꺼지기 전에 한 번은 알린다
  } else if (!g_parked) {
    g_batt_low_sent = false;             // 다시 충전되면 다음 주차 때 또 알릴 수 있게
  }
  if (env_hot() && !was_hot) {
    LOGF("[ENV] !!! %.1f°C ≥ %.0f°C — LiPo overheat risk\n", (double)g_env.temp_c, (double)VEH_TEMP_WARN_C);
    schedule_beacon(1000);               // 과열 진입은 주기를 기다리지 않고 알린다
  }
}

// !CAR\t<id>\t<st>\t<t_c>\t<rh>\t<hpa>\t<vbat_mv>\t<up_s>\t<nbr>      (PROTOCOL_CAR.md)
static String build_beacon() {
  String st = g_parked ? "P" : "D";
  if (env_hot()) st += 'H';
  if (batt_low()) st += 'L';
  String s = "!CAR\t" NODE_ID "\t" + st + "\t";
  s += g_env.ok ? String(g_env.temp_c, 1) : String("-");                          s += '\t';
  s += (g_env.ok && g_env.has_humidity) ? String(g_env.hum_pct, 1) : String("-"); s += '\t';
  s += (g_env.ok && !isnan(g_env.press_hpa)) ? String((int)lroundf(g_env.press_hpa)) : String("-");            s += '\t';
  s += (g_vbat_mv >= 0) ? String(g_vbat_mv) : String("-");                        s += '\t';
  s += String((unsigned long)uptime_s());                                         s += '\t';
  s += String(lora_nodes_count());
  return s;
}

static void beacon_tick(uint32_t now) {
  if (!g_next_beacon_ms || (int32_t)(now - g_next_beacon_ms) < 0) return;
  uint32_t period = g_parked ? VEH_BEACON_PARKED_MS : VEH_BEACON_DRIVING_MS;
  g_next_beacon_ms = 0;
  if (period) schedule_beacon(period + (esp_random() % 10000));    // ±지터: 다른 차와 위상 고정 방지

  String line = build_beacon();
  // beacon-class (§8a): 스트림에 양보하되 자기 주기 1회를 넘기진 않는다. 주기가 없는
  // 단발(상태 전환/수동)은 60초를 상한으로 둔다.
  if (lora_send_l1(line, VEH_BEACON_TTL, period ? period : 60000UL)) {
    if (g_parked) g_rtc_next_bcn = time(nullptr) + VEH_SLEEP_WAKE_S;   // 슬립 중 비콘 일정의 기준
    g_last_beacon = line;
    g_beacon_count++;
    ble_dash_send_line("{\"t\":\"bcn\",\"line\":\"" + json_escape(line) + "\",\"n\":" +
                       String((unsigned long)g_beacon_count) + "}");
  }
}

// ---------------------------------------------------------------------------
// 대시보드 출력
// ---------------------------------------------------------------------------
static void send_hello() {
  String s = "{\"t\":\"hello\",\"id\":\"" NODE_ID "\",\"name\":\"" + json_escape(g_display_id) +
             "\",\"fw\":\"" FW_VERSION "\",\"proto\":\"1.22\"";
  s += ",\"freq\":" + String((double)RF_FREQ_MHZ, 1) + ",\"sf\":" + String(RF_SF) +
       ",\"dbm\":" + String(LORA_TX_DBM) + ",\"bcn_s\":" + String((unsigned long)(VEH_BEACON_PARKED_MS / 1000)) +
       ",\"warn_c\":" + String((double)VEH_TEMP_WARN_C, 0) + ",\"sleep_s\":" + String(VEH_SLEEP_ENABLE ? VEH_SLEEP_WAKE_S : 0) + "}";
  ble_dash_send_line(s);
}

static void send_status() {
  LoraStats ls; lora_get_stats(&ls);
  const char* chip = g_env_name ? g_env_name : "none";
  uint32_t now = millis();
  String s = "{\"t\":\"st\",\"parked\":";
  s += g_parked ? "true" : "false";
  s += ",\"sensor\":\""; s += chip; s += "\"";
  s += ",\"temp\":" + num_or_null(g_env.ok, g_env.temp_c, 1);
  s += ",\"hum\":"  + num_or_null(g_env.ok && g_env.has_humidity, g_env.hum_pct, 1);
  s += ",\"hpa\":"  + num_or_null(g_env.ok && !isnan(g_env.press_hpa), g_env.press_hpa, 1);
  s += ",\"hot\":"; s += env_hot() ? "true" : "false";
  s += ",\"vbat\":" + (g_vbat_mv >= 0 ? String(g_vbat_mv) : String("null"));
  s += ",\"bpct\":" + (g_vbat_mv >= 0 ? String(battery_pct(g_vbat_mv)) : String("null"));
  s += ",\"blow\":"; s += batt_low() ? "true" : "false";
  s += ",\"up\":" + String((unsigned long)uptime_s());
  s += ",\"awake\":" + ((g_parked && VEH_SLEEP_ENABLE) ? String((long)((int32_t)(g_awake_until_ms - now) / 1000)) : String("null"));
  s += ",\"nodes\":" + String(lora_nodes_count());
  s += ",\"link\":"; s += lora_connected() ? "true" : "false";
  s += ",\"radio\":" + String(ls.radio_status);
  s += ",\"rssi\":" + (ls.last_rssi_valid ? String(ls.last_rssi) : String("null"));
  s += ",\"rx\":" + String((unsigned long)ls.rx_ok) + ",\"dup\":" + String((unsigned long)ls.rx_dup) +
       ",\"bad\":" + String((unsigned long)ls.rx_bad) + ",\"tx\":" + String((unsigned long)ls.tx_frames) +
       ",\"lbt\":" + String((unsigned long)ls.lbt_defers);
  s += ",\"bcn_n\":" + String((unsigned long)g_beacon_count);
  s += ",\"bcn_in\":" + (g_next_beacon_ms ? String((long)((int32_t)(g_next_beacon_ms - now) / 1000)) : String("null"));
  s += "}";
  ble_dash_send_line(s);
}

static void send_nodes() {
  static LoraNode snap[LORA_NODE_MAX];
  int n = lora_nodes_snapshot(snap, LORA_NODE_MAX);
  uint32_t now = millis();
  // 헤더가 개수를 알려주고, 폰은 그만큼 다 받은 뒤에 목록을 한 번에 교체한다.
  ble_dash_send_line("{\"t\":\"nodes\",\"n\":" + String(n) + "}");
  for (int i = 0; i < n; i++) {
    const LoraNode& c = snap[i];
    String s = "{\"t\":\"node\",\"id\":\""; s += c.id;
    s += "\",\"name\":\"" + json_escape(c.name) + "\",\"last\":\"" + json_escape(c.last_type) + "\"";
    s += ",\"it\":\"" + json_escape(c.info_type) + "\",\"info\":\"" + json_escape(c.info) + "\"";
    s += ",\"rssi\":" + (c.rssi_valid ? String(c.rssi) : String("null"));
    s += ",\"snr\":"  + (c.rssi_valid ? String(c.snr, 1) : String("null"));
    s += ",\"prssi\":" + (c.peer_rssi_valid ? String(c.peer_rssi) : String("null"));
    s += ",\"hops\":" + String((int)c.hops);
    s += ",\"age\":" + String((unsigned long)((now - c.last_seen_ms) / 1000));
    s += ",\"frames\":" + String((unsigned long)c.frames) + "}";
    ble_dash_send_line(s);
  }
}

static void send_history() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < g_hist_count; i++) {
    const HistEntry& e = g_hist[(g_hist_head + i) % VEH_CHAT_HISTORY];
    ble_dash_send_line("{" + e.body + ",\"age\":" + String((unsigned long)((now - e.ms) / 1000)) + "}");
  }
}

static void send_everything() {
  send_hello();
  send_status();
  send_nodes();
  send_history();
  if (g_last_beacon.length())
    ble_dash_send_line("{\"t\":\"bcn\",\"line\":\"" + json_escape(g_last_beacon) + "\",\"n\":" +
                       String((unsigned long)g_beacon_count) + "}");
  g_nodes_dirty = false;
}

// ---------------------------------------------------------------------------
// 채팅 / 명령
// ---------------------------------------------------------------------------
static void chat_send(const String& text_in) {
  String text = text_in;
  text.trim();
  if (text.length() == 0) return;
  text = utf8_truncate(text, 480);
  // 수신측이 누가 보냈는지 알 수 있게 prefix (키보드 페이저 / T-Deck과 같은 관례).
  lora_send_message("[" + g_display_id + "] " + text);
  LOGF("[SEND] \"%s\"\n", text.c_str());
  dash_event("\"t\":\"msg\",\"dir\":\"tx\",\"text\":\"" + json_escape(text) + "\"");
}

static void save_display_id(const String& raw) {
  String id;
  for (size_t i = 0; i < raw.length(); i++) {                // 프로토콜 구분자/제어문자 제거
    char c = raw[i];
    if ((uint8_t)c < 0x20 || c == '[' || c == ']' || c == '|') continue;
    id += c;
  }
  id.trim();
  id = utf8_truncate(id, MAX_DISPLAY_ID_BYTES);
  if (id.length() == 0) return;
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString("display_id", id);
  prefs.end();
  g_display_id = id;
  lora_set_my_id(id);
  LOGF("[CFG] display id = \"%s\"\n", id.c_str());
}

static void handle_command(const String& cmd) {
  LOGF("[DASH] cmd: %s\n", cmd.c_str());
  if (cmd == "GET") {
    send_everything();
  } else if (cmd.startsWith("MSG ")) {
    chat_send(cmd.substring(4));
  } else if (cmd.startsWith("ID ")) {
    save_display_id(cmd.substring(3));
    send_hello();
  } else if (cmd == "BCN") {
    schedule_beacon(0);
  } else {
    ble_dash_send_line("{\"t\":\"err\",\"s\":\"unknown command\"}");
  }
}

// ---------------------------------------------------------------------------
// LoRa 콜백 (lora_tick 컨텍스트 = 메인 루프)
// ---------------------------------------------------------------------------
static String rx_meta_json(const LoraRxInfo& info) {
  return "\"from\":\"" + String(info.src) + "\",\"rssi\":" + String(info.rssi) +
         ",\"snr\":" + String(info.snr, 1) + ",\"hops\":" + String((int)info.hops) +
         ",\"ttl\":" + String((unsigned)info.ttl);
}

static void on_lora_msg(const String& text, const LoraRxInfo& info) {
  stay_awake(VEH_AWAKE_MS, "chat rx");           // 누가 말을 걸었다 → 30분 깨어 있는다
  LOGF("[RX] from=%s rssi=%d hops=%d%s \"%s\"\n", info.src, info.rssi, info.hops,
       info.partial ? " PARTIAL" : "", text.c_str());
  dash_event("\"t\":\"msg\",\"dir\":\"rx\"," + rx_meta_json(info) + ",\"partial\":" +
             (info.partial ? "true" : "false") + ",\"text\":\"" + json_escape(text) + "\"");
}

static void on_lora_l1(const String& type, const String& args, const LoraRxInfo& info) {
  // 재난경보는 채팅과 별개로 폰에 띄운다. 본문은 마지막 필드 — 해석은 대시보드 몫.
  if (type == "AL") {
    LOGF("[AL] from=%s %s\n", info.src, args.c_str());
    dash_event("\"t\":\"al\"," + rx_meta_json(info) + ",\"args\":\"" + json_escape(args) + "\"");
  }
}

static void on_lora_conn(bool connected) {
  LOGF("[LORA] link %s\n", connected ? "up" : "down");
  g_next_status_ms = 0;
}

// 진단: I2C 버스 스캔 + 흔한 센서의 ID 레지스터 덤프 (Serial 'I').
static int i2c_read_reg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)addr, 1) != 1) return -1;
  return Wire.read();
}
// 선 하나의 전기적 상태: 내부 풀업/풀다운(~45k)을 번갈아 걸어 본다.
//   up=1 down=1 → 외부 풀업 있음(정상)   up=1 down=0 → 떠 있음(아무것도 안 물림)
//   up=0 down=0 → 뭔가가 Low로 강하게 잡고 있음(GND/엉뚱한 핀에 물림, 또는 모듈 전원 없음)
static void i2c_line_state(const char* name, int pin) {
  gpio_reset_pin((gpio_num_t)pin);   // I2C 드라이버가 남긴 내부 풀업/매트릭스 배선을 완전히 지운다
  pinMode(pin, INPUT_PULLUP);   delay(2); int up = digitalRead(pin);
  pinMode(pin, INPUT_PULLDOWN); delay(2); int dn = digitalRead(pin);
  pinMode(pin, INPUT);
  Serial.printf("  %s (GPIO%d): pullup=%d pulldown=%d → %s\n", name, pin, up, dn,
                up && dn ? "외부 풀업 OK" : up && !dn ? "떠 있음 — 센서에 안 물렸거나 풀업 없는 모듈"
                                          : "LOW로 잡혀 있음 — 배선 확인");
}
static void i2c_scan() {
  Wire.end();
  Serial.println("[I2C] line check");
#if VEH_ENV_CSB_PIN >= 0
  Serial.printf("  CSB (GPIO%d) driven HIGH = I2C mode  |  SDO (GPIO%d) driven %s = addr 0x%02X\n",
                VEH_ENV_CSB_PIN, VEH_ENV_SDO_PIN, VEH_ENV_SDO_LEVEL ? "HIGH" : "LOW", VEH_ENV_SDO_LEVEL ? 0x77 : 0x76);
#endif
  i2c_line_state("SDA/D6", I2C_SDA_PIN);
  i2c_line_state("SCL/D7", I2C_SCL_PIN);
  env_pins_init();
  // 정상 배선 → SDA/SCL 바꿔서 → 둘 다 50 kHz로. 어디서든 잡히면 원인이 바로 드러난다.
  struct { int sda, scl; uint32_t hz; const char* why; } tries[] = {
    {I2C_SDA_PIN, I2C_SCL_PIN, 100000, "normal"},
    {I2C_SCL_PIN, I2C_SDA_PIN, 100000, "SDA/SCL SWAPPED"},
    {I2C_SDA_PIN, I2C_SCL_PIN,  50000, "normal 50kHz"},
    {I2C_SCL_PIN, I2C_SDA_PIN,  50000, "swapped 50kHz"},
  };
  int total = 0;
  for (auto& t : tries) {
    Wire.end();
    Wire.begin(t.sda, t.scl, t.hz);
    Wire.setTimeOut(20);
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() != 0) continue;
      found++; total++;
      Serial.printf("  [%s] 0x%02X  reg[0x00]=%d reg[0xD0]=%d   (BMP390: reg0=0x60=96 / BME280: regD0=0x60=96)\n",
                    t.why, a, i2c_read_reg(a, 0x00), i2c_read_reg(a, 0xD0));
    }
    if (found) break;                         // 찾았으면 그 배선 설정을 남겨둔다
    Serial.printf("  [%s] nothing\n", t.why);
  }
  Wire.end();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(20);
  Serial.printf("[I2C] %d device(s)\n", total);
}

// 진단 'J': 센서에 SPI(비트뱅, mode 0)로 말을 건다. I2C에 침묵하는 BMP390이 여기 답하면
// 칩은 살아 있고 SPI 모드로 잠긴 것(= 전원 인가 순간 CSB가 Low였다는 뜻).
//   CS=CSB(GPIO2)  SCK=SCL선(I2C_SCL_PIN)  MOSI=SDA선(I2C_SDA_PIN)  MISO=SDO(GPIO4)
static void spi_probe() {
#if VEH_ENV_CSB_PIN >= 0 && VEH_ENV_SDO_PIN >= 0
  Wire.end();
  const int cs = VEH_ENV_CSB_PIN, sck = I2C_SCL_PIN, mosi = I2C_SDA_PIN, miso = VEH_ENV_SDO_PIN;
  gpio_reset_pin((gpio_num_t)sck); gpio_reset_pin((gpio_num_t)mosi); gpio_reset_pin((gpio_num_t)miso);
  pinMode(cs, OUTPUT); digitalWrite(cs, HIGH);
  pinMode(sck, OUTPUT); digitalWrite(sck, LOW);
  pinMode(mosi, OUTPUT); digitalWrite(mosi, LOW);
  pinMode(miso, INPUT);
  auto xfer = [&](uint8_t out) {
    uint8_t in = 0;
    for (int b = 7; b >= 0; b--) {
      digitalWrite(mosi, (out >> b) & 1); delayMicroseconds(5);
      digitalWrite(sck, HIGH); delayMicroseconds(5);
      in = (in << 1) | digitalRead(miso);
      digitalWrite(sck, LOW); delayMicroseconds(5);
    }
    return in;
  };
  // BMP3xx SPI read: [addr|0x80] [dummy] [data...]. reg 0x00 = chip id (0x60 BMP390 / 0x50 BMP388)
  digitalWrite(cs, LOW); delayMicroseconds(10);
  xfer(0x80); xfer(0x00);
  uint8_t id = xfer(0x00), rev = xfer(0x00);
  digitalWrite(cs, HIGH); delayMicroseconds(10);
  // BME280 SPI read: [addr&0x7F] [data]. reg 0xD0 = chip id (0x60)
  digitalWrite(cs, LOW); delayMicroseconds(10);
  xfer(0xD0); uint8_t id2 = xfer(0x00);
  digitalWrite(cs, HIGH);
  Serial.printf("[SPI] bmp3 reg0=0x%02X rev=0x%02X | bme280 regD0=0x%02X   (0x60/0x50 = 칩이 SPI로 응답, 0x00/0xFF = 무응답)\n", id, rev, id2);
  env_pins_init();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(20);
#else
  Serial.println("[SPI] CSB/SDO GPIO 미설정");
#endif
}

// ---------------------------------------------------------------------------
// 딥슬립
// ---------------------------------------------------------------------------
static const int kHoldPins[] = { LORA_RST_PIN, LORA_NSS_PIN, LORA_RXEN_PIN };   // 슬립 중 라디오가 RX를 유지하려면 HIGH로 잡혀 있어야

static void go_to_sleep() {
  LOGF("[SLEEP] deep sleep: next beacon in %lds, or on LoRa DIO1 (GPIO%d). up=%lus boots=%lu\n",
       (long)(g_rtc_next_bcn - time(nullptr)), lora_dio1_pin(), (unsigned long)uptime_s(), (unsigned long)g_rtc_boots);
  Serial.flush();
  ble_dash_end();
  lora_sleep_arm();                                // 라디오: 듀티사이클 RX, DIO1 = RxDone
  for (int pin : kHoldPins) gpio_hold_en((gpio_num_t)pin);
  gpio_deep_sleep_hold_en();
  // 타이머는 "다음 비콘까지 남은 시간". LoRa로 자주 깨도 비콘 일정은 밀리지 않는다.
  long remain = (long)(g_rtc_next_bcn - time(nullptr));
  if (remain < 5 || remain > VEH_SLEEP_WAKE_S) remain = VEH_SLEEP_WAKE_S;
  esp_sleep_enable_timer_wakeup((uint64_t)remain * 1000000ULL);
  if (lora_dio1_pin() <= 21) esp_sleep_enable_ext0_wakeup((gpio_num_t)lora_dio1_pin(), 1);
  g_rtc_was_parked = true;
  esp_deep_sleep_start();
}

static void sleep_tick(uint32_t now) {
#if VEH_SLEEP_ENABLE
  if (!g_parked || g_power_raw) return;            // 주행 중이거나 USB가 보이면 절대 안 잔다
  if ((int32_t)(now - g_awake_until_ms) < 0) return;
  if (!lora_idle()) return;                        // 비콘/PONG/채팅 송신·수신이 끝날 때까지
  go_to_sleep();
#endif
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup() {
  g_wake_cause = esp_sleep_get_wakeup_cause();
  g_woke_from_sleep = (g_wake_cause == ESP_SLEEP_WAKEUP_TIMER || g_wake_cause == ESP_SLEEP_WAKEUP_EXT0);
  if (g_woke_from_sleep) {                         // 슬립 중 잡아뒀던 라디오 제어핀을 다시 풀어준다
    gpio_deep_sleep_hold_dis();
    for (int pin : kHoldPins) gpio_hold_dis((gpio_num_t)pin);
  }
  g_rtc_boots++;
  env_pins_init();                // 센서가 CSB Low를 보기 전에 — 무엇보다 먼저
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);      // 주차 중엔 USB 호스트가 없다 — 로그가 루프를 막으면 안 된다
  delay(300);
  g_boot_cpu_mhz = getCpuFrequencyMhz();
  LOGF("\n==== pager-vehicle %s / XIAO-S3 + Wio-SX1262 / node %s ====\n", FW_VERSION, NODE_ID);
  LOGF("[BOOT] reset_reason=%d (1=POR 4=PANIC 5=INT_WDT 6=TASK_WDT 7=WDT 9=BROWNOUT 11=USB)  wake=%s  boots=%lu  up=%lus\n",
       (int)esp_reset_reason(),
       g_wake_cause == ESP_SLEEP_WAKEUP_TIMER ? "TIMER" : g_wake_cause == ESP_SLEEP_WAKEUP_EXT0 ? "LORA-DIO1" : "cold",
       (unsigned long)g_rtc_boots, (unsigned long)uptime_s());
  LOGF("Serial: S=status  I=i2c scan  N=nodes  X=RF config  R=last RSSI  B=beacon now  P=inject addressed PING  Z=deep sleep now  M<text>=send chat\n");

#if VEH_VBUS_SENSE_PIN >= 0
  pinMode(VEH_VBUS_SENSE_PIN, INPUT);
#endif

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, true);
  g_display_id = prefs.getString("display_id", DEFAULT_DISPLAY_ID);
  prefs.end();
  LOGF("[CFG] display id = \"%s\"\n", g_display_id.c_str());

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(20);            // SCL이 잡혀 있을 때 probe가 메인 루프를 오래 막지 않게
  env_probe();
  if (!g_env_name) LOGF("[ENV] sensor: NOT FOUND (Serial 'I' = I2C 진단)\n");

  lora_set_callbacks(on_lora_msg, on_lora_conn);   // begin() 전에: DIO1 wake 시 깨운 패킷이 begin 안에서 콜백된다
  lora_set_l1_callback(on_lora_l1);
  lora_set_my_id(g_display_id);
  // 슬립에서 깼으면 SX1262는 설정된 채 살아 있고 버퍼에 우리를 깨운 패킷이 있을 수 있다 → 리셋 없이 재개.
  // ext0로 깼는데 wake 신호가 아니면(남의 HB 등) 짧게만 깨어 있는다. 타이머면 비콘 창만큼.
  if (g_woke_from_sleep) {
    lora_set_hb_enabled(false);                    // 짧은 창에선 HB 안 낸다 — loop()가 full 각성이 되면 다시 켠다
    stay_awake(g_wake_cause == ESP_SLEEP_WAKEUP_EXT0 ? VEH_WAKE_SHORT_MS : VEH_WAKE_WINDOW_MS,
               g_wake_cause == ESP_SLEEP_WAKEUP_EXT0 ? "LoRa wake" : "timer wake", false);
  }
  lora_begin(!g_woke_from_sleep);

  ble_dash_begin();

  // 부팅 직후엔 USB 열거가 아직 안 끝났을 수 있다 → "전원 있음"으로 시작하고 디바운스가
  // 지나서도 호스트가 없으면 그때 주차로 넘어간다.
  if (g_woke_from_sleep && g_rtc_was_parked) {
    // 잠들기 전에 주차 중이었다 → 주차로 시작. USB가 붙어 있으면 power_tick이 10초 디바운스 뒤
    // 주행으로 되돌리며, 그동안엔 g_power_raw 때문에 슬립하지 않는다.
    g_power_raw = false;
    g_power_raw_ms = millis();
    apply_parked(true, false);
    // 어떤 이유로 깼든 비콘 예정 시각이 지났으면 보낸다 (남의 프레임으로 깬 김에라도).
    if (g_rtc_next_bcn == 0 || time(nullptr) >= g_rtc_next_bcn) {
      schedule_beacon(300);
      if (g_wake_cause == ESP_SLEEP_WAKEUP_EXT0) stay_awake(VEH_WAKE_WINDOW_MS, "beacon due", false);
    }
  } else {
    g_power_raw = true;
    g_power_raw_ms = millis();
    apply_parked(false, false);
  }
}

void loop() {
  uint32_t now = millis();

  lora_tick();
  ble_dash_tick();

  power_tick(now);
  sensor_tick(now);
  beacon_tick(now);
  lora_set_hb_enabled(!g_parked || !VEH_SLEEP_ENABLE || !g_woke_from_sleep || (int32_t)(now - g_full_until_ms) < 0);
  sleep_tick(now);

  if (ble_dash_ready()) stay_awake(VEH_BLE_LINGER_MS, "BLE connected");
  if (ble_dash_consume_just_ready()) {
    stay_awake(VEH_AWAKE_MS, "BLE connected");
    send_everything();
    g_next_status_ms = now + 5000;
  }
  String cmd;
  while (ble_dash_poll_command(&cmd)) handle_command(cmd);

  if (lora_tx_consume_done()) ble_dash_send_line("{\"t\":\"txdone\"}");
  {
    int seq; uint32_t cnt;
    if (lora_consume_range(&seq, &cnt)) {
      stay_awake(VEH_AWAKE_MS, "addressed PING");  // 우리 앞으로 온 PING = 깨우기 신호
      ble_dash_send_line("{\"t\":\"pong\",\"seq\":" + String(seq) + ",\"n\":" + String((unsigned long)cnt) + "}");
    }
  }

  if (lora_nodes_consume_changed()) g_nodes_dirty = true;
  if (ble_dash_ready()) {
    if ((int32_t)(now - g_next_status_ms) >= 0) {
      g_next_status_ms = now + 5000;
      send_status();
    }
    // age가 폰 쪽에서 계속 흘러가도록, 바뀐 게 없어도 30초마다는 다시 보낸다.
    if ((int32_t)(now - g_next_nodes_ms) >= 0 && (g_nodes_dirty || (int32_t)(now - g_next_nodes_ms) >= 27000)) {
      g_next_nodes_ms = now + 3000;
      g_nodes_dirty = false;
      send_nodes();
    }
  }

  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    if (g_serial_msg_mode) {
      if (c == '\r' || c == '\n') { chat_send(g_serial_line); g_serial_line = ""; g_serial_msg_mode = false; }
      else if (g_serial_line.length() < 480) g_serial_line += (char)c;
      continue;
    }
    switch (c) {
      case 'M': g_serial_msg_mode = true; g_serial_line = ""; break;
      case 'I': i2c_scan(); break;
      case 'J': spi_probe(); break;
      case 'N': lora_dump_neighbors(); break;
      case 'X': lora_probe_at(); break;
      case 'R': lora_query_rssi(); break;
      case 'B': schedule_beacon(0); Serial.println("[CMD] beacon now"); break;
      case 'P': {                            // 벤치 테스트: 우리 앞으로 온 PING 주입 → PONG + 30분 각성이어야 한다
        static uint32_t pid = 777000;
        lora_test_inject("R|TFF|" + String(pid++) + "|3|PING\t7\tTFF\t" NODE_ID);
        Serial.println("[CMD] injected addressed PING"); break; }
      case 'Z':                              // 벤치 테스트: USB 꽂힌 채로 강제 딥슬립 (USB CDC는 끊겼다가 wake 후 재열거)
        Serial.println("[CMD] forcing deep sleep now"); g_rtc_was_parked = true; go_to_sleep(); break;
      case 'S':
        Serial.printf("[STAT] %s  power_raw=%d  T=%.1fC RH=%.1f%% P=%s ok=%d  vbat=%dmV(%d%%)  nodes=%d  ble=%d  cpu=%luMHz\n",
                      g_parked ? "PARKED" : "DRIVING", (int)g_power_raw, (double)g_env.temp_c,
                      (double)g_env.hum_pct, isnan(g_env.press_hpa) ? "-" : (String(g_env.press_hpa, 1) + "hPa").c_str(), (int)g_env.ok, g_vbat_mv, battery_pct(g_vbat_mv),
                      lora_nodes_count(), (int)ble_dash_ready(), (unsigned long)getCpuFrequencyMhz());
        Serial.printf("[STAT] next beacon: %s\n", build_beacon().c_str());
        Serial.printf("[STAT] sleep: %s  awake_for=%lds (%s)  boots=%lu  this boot: %s%s\n",
                      VEH_SLEEP_ENABLE ? (g_parked ? "armed" : "driving — never") : "disabled",
                      (long)((int32_t)(g_awake_until_ms - millis()) / 1000), g_awake_why, (unsigned long)g_rtc_boots,
                      g_wake_cause == ESP_SLEEP_WAKEUP_TIMER ? "TIMER wake" : g_wake_cause == ESP_SLEEP_WAKEUP_EXT0 ? "LoRa-DIO1 wake" : "cold boot",
                      ls_resumed_note().c_str());
        break;
      default: break;
    }
  }

  delay(g_parked ? 5 : 2);
}
