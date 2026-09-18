#include "display.h"
#include "config.h"
#include "jamo_font.h"

#include <U8g2lib.h>
#include <Wire.h>

// Full-buffer hardware I2C SSD1306 128x64. (SCK/SDA from Wire.begin)
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static BleVisualState s_ble_state = BleVisualState::Init;
static int             s_battery_pct = -1;
static bool            s_hangul = false;
static bool            s_lora = false;
static char            s_view_label[6] = "TX";
static char            s_page_label[10] = "";

// dialog state
static String          s_dialog_text;
static uint32_t        s_dialog_end_ms = 0;

void display_begin() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  u8g2.setI2CAddress(OLED_I2C_ADDR << 1);
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFontMode(1);
}

void display_set_ble_state(BleVisualState s)       { s_ble_state = s; }
void display_set_battery_pct(int pct)              { s_battery_pct = pct; }
void display_set_hangul_mode(bool hangul)          { s_hangul = hangul; }
void display_set_lora_state(bool connected)        { s_lora = connected; }
void display_set_view_label(const char* label) {
  strncpy(s_view_label, label ? label : "", sizeof(s_view_label) - 1);
  s_view_label[sizeof(s_view_label) - 1] = 0;
}

void display_set_page_label(const char* label) {
  strncpy(s_page_label, label ? label : "", sizeof(s_page_label) - 1);
  s_page_label[sizeof(s_page_label) - 1] = 0;
}

static const char* ble_state_str(BleVisualState s) {
  switch (s) {
    case BleVisualState::Init:         return "Init";
    case BleVisualState::Scanning:     return "Scan";
    case BleVisualState::Connecting:   return "Conn?";
    case BleVisualState::Connected:    return "Conn";
    case BleVisualState::Disconnected: return "DC";
  }
  return "?";
}

static void draw_status_bar() {
  u8g2.setFont(u8g2_font_5x8_tr);
  char buf[48];
  const char* lora_tag = s_lora ? "L+" : "L-";
  if (s_battery_pct >= 0) {
    snprintf(buf, sizeof(buf), "%s %d%% %s %s %s",
             ble_state_str(s_ble_state), s_battery_pct,
             s_hangul ? "Kor" : "Eng", lora_tag, s_view_label);
  } else {
    snprintf(buf, sizeof(buf), "%s --%% %s %s %s",
             ble_state_str(s_ble_state),
             s_hangul ? "Kor" : "Eng", lora_tag, s_view_label);
  }
  u8g2.drawStr(0, 8, buf);
  if (s_page_label[0] != 0) {
    int w = u8g2.getStrWidth(s_page_label);
    u8g2.drawStr(DISPLAY_W - w, 8, s_page_label);
  }
  u8g2.drawHLine(0, STATUS_BAR_H, DISPLAY_W);
}

// UTF-8 grapheme byte length from leading byte. (BMP까지 가정)
static int utf8_size(uint8_t b) {
  if (b < 0x80) return 1;
  if ((b & 0xE0) == 0xC0) return 2;
  if ((b & 0xF0) == 0xE0) return 3;
  if ((b & 0xF8) == 0xF0) return 4;
  return 1; // invalid, skip
}

// UTF-8 한 글자 디코딩. cp 반환, 소비 byte 수 리턴. BMP까지만.
static int utf8_decode(const char* s, uint32_t& cp) {
  uint8_t b0 = (uint8_t)s[0];
  if (b0 < 0x80) { cp = b0; return 1; }
  if ((b0 & 0xE0) == 0xC0) {
    cp = ((uint32_t)(b0 & 0x1F) << 6) | (s[1] & 0x3F);
    return 2;
  }
  if ((b0 & 0xF0) == 0xE0) {
    cp = ((uint32_t)(b0 & 0x0F) << 12) | ((uint32_t)(s[1] & 0x3F) << 6) | (s[2] & 0x3F);
    return 3;
  }
  if ((b0 & 0xF8) == 0xF0) {
    cp = ((uint32_t)(b0 & 0x07) << 18) | ((uint32_t)(s[1] & 0x3F) << 12) |
         ((uint32_t)(s[2] & 0x3F) << 6) | (s[3] & 0x3F);
    return 4;
  }
  cp = '?';
  return 1;
}

// ASCII 8px, 그 외(음절/자모)는 16px.
static int glyph_width_cp(uint32_t cp) {
  if (cp < 0x80) return 8;
  return 16;
}

// 라인 wrap 계산에선 leading byte만 보고 빠르게 판정.
// (자모는 0xE3 prefix지만 정확히 구분하려면 2byte까지 봄 — 여기선 단순화)
static int glyph_width(const char* p) {
  uint32_t cp;
  utf8_decode(p, cp);
  return glyph_width_cp(cp);
}

// 한 줄 안에서 byte offset에 해당하는 픽셀 x 위치 계산.
static int x_at_offset(const char* line_start, int line_len, int offset) {
  int x = 0;
  int i = 0;
  while (i < line_len && i < offset) {
    int sz = utf8_size((uint8_t)line_start[i]);
    if (i + sz > line_len) break;
    x += glyph_width(line_start + i);
    i += sz;
  }
  return x;
}

struct LineRange { size_t start; size_t end; };

// byte_offset 직전까지의 wrap line 개수만 계산 (wrap_text의 카운트만 버전).
// 새 메시지 도착 시 "이전 history의 라인 수" = 새 메시지 첫 라인 인덱스.
int display_wrap_lines_before(const String& s, size_t byte_offset) {
  if (byte_offset == 0) return 0;
  int count = 0;
  int x = 0;
  const size_t n = s.length();
  if (byte_offset > n) byte_offset = n;
  const char* p = s.c_str();
  size_t i = 0;

  while (i < byte_offset) {
    if (p[i] == '\n') {
      count++;
      x = 0;
      i++;
      continue;
    }
    int sz = utf8_size((uint8_t)p[i]);
    if (i + sz > n) break;
    int w = glyph_width(p + i);
    if (x + w > DISPLAY_W) {
      count++;
      x = 0;
      // 폭으로 wrap된 경우 — 이 글자는 다음 라인에서 다시 처리
      // (wrap_text와 동일한 의미가 되도록 i를 증가시키지 않고 x만 초기화 후 같은 iter 재진입은
      //  로직상 어색하므로, x=0 만 하고 곧장 이 글자 너비 추가)
    }
    x += w;
    i += sz;
  }
  return count;
}

// 텍스트를 디스플레이 폭에 맞춰 줄로 분할. \n 처리 포함.
static void wrap_text(const String& s, LineRange* out, int max_lines, int& out_count) {
  out_count = 0;
  size_t i = 0;
  size_t line_start = 0;
  int x = 0;
  const size_t n = s.length();
  const char* p = s.c_str();

  auto push_line = [&](size_t end) {
    if (out_count < max_lines) {
      out[out_count++] = {line_start, end};
    } else {
      // 가장 오래된 줄을 버리고 shift
      for (int k = 1; k < max_lines; k++) out[k - 1] = out[k];
      out[max_lines - 1] = {line_start, end};
    }
  };

  while (i < n) {
    if (p[i] == '\n') {
      push_line(i);
      line_start = i + 1;
      x = 0;
      i = i + 1;
      continue;
    }
    int sz = utf8_size((uint8_t)p[i]);
    if (i + sz > n) break;
    int w = glyph_width(p + i);
    if (x + w > DISPLAY_W) {
      push_line(i);
      line_start = i;
      x = 0;
    }
    x += w;
    i += sz;
  }
  push_line(n);
}

// 화면 중앙에 외곽 박스 + 텍스트.
static void draw_dialog() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_korean2);
  const char* t = s_dialog_text.c_str();
  int tw = u8g2.getUTF8Width(t);
  int box_w = tw + 16;
  int box_h = 28;
  if (box_w > DISPLAY_W - 4) box_w = DISPLAY_W - 4;
  int box_x = (DISPLAY_W - box_w) / 2;
  int box_y = (DISPLAY_H - box_h) / 2;
  u8g2.drawRFrame(box_x, box_y, box_w, box_h, 3);
  // 텍스트는 박스 중앙에 (unifont 16px, baseline 위 ~13)
  int tx = (DISPLAY_W - tw) / 2;
  int ty = box_y + box_h / 2 + 5;
  u8g2.drawUTF8(tx, ty, t);
}

void display_show_dialog(const char* text, uint32_t duration_ms) {
  s_dialog_text = text;
  s_dialog_end_ms = millis() + duration_ms;
}

bool display_dialog_active() {
  return s_dialog_end_ms > 0 && (int32_t)(millis() - s_dialog_end_ms) < 0;
}

bool display_dialog_consume_expiry() {
  if (s_dialog_end_ms == 0) return false;
  if ((int32_t)(millis() - s_dialog_end_ms) < 0) return false;
  s_dialog_end_ms = 0;
  return true;
}

// 한 라인을 글자 단위(glyph)로 그림. compat jamo는 비트맵, 나머지는 폰트.
static void draw_text_line(const char* base, size_t start, size_t end, int y) {
  int x = 0;
  size_t pos = start;
  char one[5];
  while (pos < end) {
    int sz = utf8_size((uint8_t)base[pos]);
    if (pos + sz > end) break;
    uint32_t cp;
    utf8_decode(base + pos, cp);
    const uint8_t* jamo_bmp = jamo_font_lookup(cp);
    if (jamo_bmp) {
      u8g2.drawBitmap(x, y - 14, 2, 16, jamo_bmp);
      x += 16;
    } else {
      memcpy(one, base + pos, sz);
      one[sz] = 0;
      u8g2.drawUTF8(x, y, one);
      x += glyph_width_cp(cp);
    }
    pos += sz;
  }
}

// HISTORY_BUF_BYTES(8KB) 가득 찼을 때 wrap 라인 최대 추정:
// - 영문(1B/char): 128px / 8px = 16 char/line → 8192/16 = 512 line
// - 한글(3B/char): 128px / 16px = 8 char/line → 8192/(8*3) = ~340 line
// 안전하게 512로. LineRange = 8 byte × 512 = 4 KB 정적 — stack 안 쓰고 BSS.
static LineRange s_history_lines[512];

int display_render_history(const String& text, int scroll_lines) {
  if (display_dialog_active()) {
    draw_dialog();
    u8g2.sendBuffer();
    return scroll_lines;
  }

  static constexpr int MAX_LINES = sizeof(s_history_lines) / sizeof(s_history_lines[0]);
  LineRange* lines = s_history_lines;
  int line_count = 0;
  wrap_text(text, lines, MAX_LINES, line_count);

  // 마지막 페이지의 시작 라인 (page-aligned). 예: line_count=10, TEXT_LINES=3
  //   → pages: [0-2][3-5][6-8][9]   total_pages=4, last_page_first=9.
  //   이전 코드는 max_first = 10-3 = 7로 잡고 (7/3)*3=6으로 정렬 → 라인 9 누락 버그.
  int last_page_first = (line_count > 0) ? ((line_count - 1) / TEXT_LINES) * TEXT_LINES : 0;
  int first;
  if (scroll_lines < 0 || scroll_lines > last_page_first) first = last_page_first;
  else                                                     first = scroll_lines;
  // 페이지 단위로 정렬 (위/아래 키가 페이지 단위라 자연스러움)
  first = (first / TEXT_LINES) * TEXT_LINES;
  if (first > last_page_first) first = last_page_first;
  if (first < 0) first = 0;

  // 페이지 라벨
  int total_pages = (line_count + TEXT_LINES - 1) / TEXT_LINES;
  if (total_pages == 0) total_pages = 1;
  int cur_page = first / TEXT_LINES + 1;
  char pl[10];
  snprintf(pl, sizeof(pl), "%d/%d", cur_page, total_pages);
  display_set_page_label(pl);

  u8g2.clearBuffer();
  draw_status_bar();

  int show = line_count - first;
  if (show > TEXT_LINES) show = TEXT_LINES;

  u8g2.setFont(u8g2_font_unifont_t_korean2);
  const char* base = text.c_str();
  for (int i = 0; i < show; i++) {
    const LineRange& lr = lines[first + i];
    int y = TEXT_AREA_Y + (i + 1) * TEXT_LINE_H - 3;
    draw_text_line(base, lr.start, lr.end, y);
  }
  u8g2.sendBuffer();
  return first;
}

void display_render_config(const char* prompt, const String& input) {
  if (display_dialog_active()) {
    draw_dialog();
    u8g2.sendBuffer();
    return;
  }
  display_set_page_label("");
  u8g2.clearBuffer();
  draw_status_bar();

  u8g2.setFont(u8g2_font_unifont_t_korean2);
  int y1 = TEXT_AREA_Y + TEXT_LINE_H - 3;
  u8g2.drawUTF8(0, y1, prompt);

  int y2 = y1 + TEXT_LINE_H;
  String shown = input;
  shown += "_";                       // 커서
  u8g2.drawUTF8(0, y2, shown.c_str());

  int y3 = y2 + TEXT_LINE_H;
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, y3, "Enter=save Esc=cancel");

  u8g2.sendBuffer();
}

void display_render(const String& committed, const String& composing) {
  // 다이얼로그 표시 중이면 본문 대신 다이얼로그만
  if (display_dialog_active()) {
    draw_dialog();
    u8g2.sendBuffer();
    return;
  }

  // TX 뷰는 페이지 라벨 없음
  display_set_page_label("");

  u8g2.clearBuffer();
  draw_status_bar();

  // 합쳐서 한 번에 wrap. composing 영역은 committed 끝부터 (committed.len .. total).
  String full = committed + composing;
  const size_t compose_start = committed.length();
  const size_t compose_end   = full.length();

  // 최대 보여줄 줄 수 + 여유분(앞쪽 자르기 위해)
  static constexpr int MAX_LINES = 32;
  LineRange lines[MAX_LINES];
  int line_count = 0;
  wrap_text(full, lines, MAX_LINES, line_count);

  // 마지막 TEXT_LINES 줄만 표시
  int first = line_count - TEXT_LINES;
  if (first < 0) first = 0;
  int show = line_count - first;

  u8g2.setFont(u8g2_font_unifont_t_korean2);
  const char* base = full.c_str();
  for (int i = 0; i < show; i++) {
    const LineRange& lr = lines[first + i];
    int y = TEXT_AREA_Y + (i + 1) * TEXT_LINE_H - 3;
    draw_text_line(base, lr.start, lr.end, y);

    // 조합 중 영역을 inverse video로 강조 (XOR draw로 픽셀 토글).
    // 글리프가 폰트에 없어 안 그려져도 박스는 보여서 위치 단서가 됨.
    if (compose_start < compose_end) {
      size_t line_a = lr.start, line_b = lr.end;
      size_t inter_a = max(line_a, compose_start);
      size_t inter_b = min(line_b, compose_end);
      if (inter_a < inter_b) {
        int x1 = x_at_offset(base + line_a, (int)(line_b - line_a),
                             (int)(inter_a - line_a));
        int x2 = x_at_offset(base + line_a, (int)(line_b - line_a),
                             (int)(inter_b - line_a));
        if (x2 > x1) {
          int y_top = y - TEXT_LINE_H + 3;          // 글자 셀 위쪽
          int box_h = TEXT_LINE_H;
          u8g2.setDrawColor(2);                     // XOR
          u8g2.drawBox(x1, y_top, x2 - x1, box_h);
          u8g2.setDrawColor(1);                     // 복원
        }
      }
    }
  }

  u8g2.sendBuffer();
}
