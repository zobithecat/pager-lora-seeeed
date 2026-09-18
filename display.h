#pragma once
#include <Arduino.h>
#include <stdint.h>

enum class BleVisualState : uint8_t {
  Init,
  Scanning,
  Connecting,
  Connected,
  Disconnected,
};

void display_begin();

// 상태바 갱신
void display_set_ble_state(BleVisualState s);
void display_set_battery_pct(int pct);             // -1 = 알 수 없음
void display_set_hangul_mode(bool hangul);
void display_set_lora_state(bool connected);       // LoRa link 상태
void display_set_view_label(const char* label);    // 상태바 우측 라벨 (TX/RX 등, ≤3자)

// 본문 영역 텍스트와 미리보기를 합쳐 그린다. 호출자가 화면 갱신 트리거.
// committed: 확정된 UTF-8 텍스트. composing: 조합 중인 음절 UTF-8.
void display_render(const String& committed, const String& composing);

// Config 화면: prompt(라벨) + 편집 중인 input. ASCII 가정.
void display_render_config(const char* prompt, const String& input);

// History 스크롤 렌더. scroll_lines = -1 또는 큰 수면 마지막 페이지로 clamp.
// 반환: 실제로 사용된 first line (clamp 결과). 호출자가 다음 스크롤 위해 저장.
// 내부에서 페이지 라벨도 갱신함.
int  display_render_history(const String& text, int scroll_lines);

// 특정 byte offset 직전까지의 wrap line 개수를 반환.
// 새 메시지 도착 시 "안 읽은 메시지 첫 줄" 으로 스크롤하기 위해 사용.
// (g_rx_scroll = display_wrap_lines_before(history, new_msg_byte_start))
int  display_wrap_lines_before(const String& text, size_t byte_offset);

// 상태바 우측 끝에 표시되는 라벨 (예: "2/5"). 비우면 안 그림.
void display_set_page_label(const char* label);

// 화면 중앙에 모달 다이얼로그 (텍스트 + 박스). duration_ms 후 자동 dismiss.
// 이 동안 display_render 호출하면 본문 대신 다이얼로그가 그려진다.
void display_show_dialog(const char* text, uint32_t duration_ms);
bool display_dialog_active();          // 표시 중인가
bool display_dialog_consume_expiry();  // 만료된 순간 한 번만 true (main loop이 redraw 트리거용)
