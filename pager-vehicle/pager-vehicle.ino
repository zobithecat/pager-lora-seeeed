#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_system.h>       // esp_reset_reason()
#include "config.h"
#include "lora.h"
#include "bme280.h"
#include "ble_dash.h"
#if !ARDUINO_USB_MODE
#include "tusb.h"             // tud_mounted / tud_suspended (USB-OTG/TinyUSB 모드)
#endif

// ===== 차량 페이저 (P01) =====
// 하는 일 네 가지:
//   1. BME280으로 실내 온습도/기압 측정
//   2. 들리는 LoRa 노드 전부를 discovery 테이블로 유지 (lora.cpp)
//   3. 주차 중(LiPo 구동)엔 !CAR 상태 비콘을 주기 송신 (PROTOCOL_CAR.md)
//   4. 폰의 Bluefy 웹 대시보드에 BLE로 붙어 discovery / 비콘 / 채팅 제공
// 모든 상태는 메인 루프에서만 만진다. BLE 콜백은 ble_dash.cpp가 큐로 넘겨준다.

static String   g_display_id;
static uint32_t g_boot_cpu_mhz = 240;

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
static uint32_t g_beacon_count = 0;

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
  for (int i = 0; i < 16; i++) acc += analogReadMilliVolts(VEH_VBAT_ADC_PIN);
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
static bool env_hot() { return g_env.ok && g_env.temp_c >= VEH_TEMP_WARN_C; }

static void sensor_tick(uint32_t now) {
  if ((int32_t)(now - g_next_sensor_ms) < 0) return;
  g_next_sensor_ms = now + VEH_SENSOR_MS;
  if (bme280_chip() == Bme280Chip::None) bme280_begin(BME280_ADDR);   // 늦게 꽂혀도 잡는다
  bool was_hot = env_hot();
  g_env = bme280_read();
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
  s += g_env.ok ? String((int)lroundf(g_env.press_hpa)) : String("-");            s += '\t';
  s += (g_vbat_mv >= 0) ? String(g_vbat_mv) : String("-");                        s += '\t';
  s += String((unsigned long)(millis() / 1000));                                  s += '\t';
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
       ",\"warn_c\":" + String((double)VEH_TEMP_WARN_C, 0) + "}";
  ble_dash_send_line(s);
}

static void send_status() {
  LoraStats ls; lora_get_stats(&ls);
  const char* chip = bme280_chip() == Bme280Chip::BME280 ? "BME280"
                   : bme280_chip() == Bme280Chip::BMP280 ? "BMP280" : "none";
  uint32_t now = millis();
  String s = "{\"t\":\"st\",\"parked\":";
  s += g_parked ? "true" : "false";
  s += ",\"sensor\":\""; s += chip; s += "\"";
  s += ",\"temp\":" + num_or_null(g_env.ok, g_env.temp_c, 1);
  s += ",\"hum\":"  + num_or_null(g_env.ok && g_env.has_humidity, g_env.hum_pct, 1);
  s += ",\"hpa\":"  + num_or_null(g_env.ok, g_env.press_hpa, 1);
  s += ",\"hot\":"; s += env_hot() ? "true" : "false";
  s += ",\"vbat\":" + (g_vbat_mv >= 0 ? String(g_vbat_mv) : String("null"));
  s += ",\"bpct\":" + (g_vbat_mv >= 0 ? String(battery_pct(g_vbat_mv)) : String("null"));
  s += ",\"blow\":"; s += batt_low() ? "true" : "false";
  s += ",\"up\":" + String((unsigned long)(now / 1000));
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

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);      // 주차 중엔 USB 호스트가 없다 — 로그가 루프를 막으면 안 된다
  delay(300);
  g_boot_cpu_mhz = getCpuFrequencyMhz();
  LOGF("\n==== pager-vehicle %s / XIAO-S3 + Wio-SX1262 / node %s ====\n", FW_VERSION, NODE_ID);
  LOGF("[BOOT] reset_reason=%d (1=POR 4=PANIC 5=INT_WDT 6=TASK_WDT 7=WDT 9=BROWNOUT 11=USB)\n",
       (int)esp_reset_reason());
  LOGF("Serial: S=status  N=nodes  X=RF config  R=last RSSI  B=beacon now  M<text>=send chat\n");

#if VEH_VBUS_SENSE_PIN >= 0
  pinMode(VEH_VBUS_SENSE_PIN, INPUT);
#endif

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, true);
  g_display_id = prefs.getString("display_id", DEFAULT_DISPLAY_ID);
  prefs.end();
  LOGF("[CFG] display id = \"%s\"\n", g_display_id.c_str());

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Bme280Chip chip = bme280_begin(BME280_ADDR);
  LOGF("[ENV] sensor: %s\n", chip == Bme280Chip::BME280 ? "BME280"
                           : chip == Bme280Chip::BMP280 ? "BMP280 (no humidity!)" : "NOT FOUND");

  lora_begin();
  lora_set_callbacks(on_lora_msg, on_lora_conn);
  lora_set_l1_callback(on_lora_l1);
  lora_set_my_id(g_display_id);

  ble_dash_begin();

  // 부팅 직후엔 USB 열거가 아직 안 끝났을 수 있다 → "전원 있음"으로 시작하고 디바운스가
  // 지나서도 호스트가 없으면 그때 주차로 넘어간다.
  g_power_raw = true;
  g_power_raw_ms = millis();
  apply_parked(false, false);
}

void loop() {
  uint32_t now = millis();

  lora_tick();
  ble_dash_tick();

  power_tick(now);
  sensor_tick(now);
  beacon_tick(now);

  if (ble_dash_consume_just_ready()) {
    send_everything();
    g_next_status_ms = now + 5000;
  }
  String cmd;
  while (ble_dash_poll_command(&cmd)) handle_command(cmd);

  if (lora_tx_consume_done()) ble_dash_send_line("{\"t\":\"txdone\"}");
  {
    int seq; uint32_t cnt;
    if (lora_consume_range(&seq, &cnt))
      ble_dash_send_line("{\"t\":\"pong\",\"seq\":" + String(seq) + ",\"n\":" + String((unsigned long)cnt) + "}");
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
      case 'N': lora_dump_neighbors(); break;
      case 'X': lora_probe_at(); break;
      case 'R': lora_query_rssi(); break;
      case 'B': schedule_beacon(0); Serial.println("[CMD] beacon now"); break;
      case 'S':
        Serial.printf("[STAT] %s  power_raw=%d  T=%.1fC RH=%.1f%% P=%.1fhPa ok=%d  vbat=%dmV(%d%%)  nodes=%d  ble=%d  cpu=%luMHz\n",
                      g_parked ? "PARKED" : "DRIVING", (int)g_power_raw, (double)g_env.temp_c,
                      (double)g_env.hum_pct, (double)g_env.press_hpa, (int)g_env.ok, g_vbat_mv, battery_pct(g_vbat_mv),
                      lora_nodes_count(), (int)ble_dash_ready(), (unsigned long)getCpuFrequencyMhz());
        Serial.printf("[STAT] next beacon: %s\n", build_beacon().c_str());
        break;
      default: break;
    }
  }

  delay(g_parked ? 5 : 2);
}
