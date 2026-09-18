# pager-lora-seeeed — BLE HID 키보드 수신기 + LoRa 페이저 (한글 IME + SSD1306)

Seeed **XIAO ESP32S3**가 BLE Central로 동작하며 BLE HID 키보드(HOGP)에 연결되고,
입력을 한국어 두벌식 IME로 조합해 128x64 SSD1306 OLED에 표시한다. 작성한 메시지를
**Wio-SX1262 (SX1262 LoRa)** 로 송신하고, 다른 노드의 메시지를 수신한다. Left Alt로 한/영 토글.

> `../pager-lora-qwerty` (ESP32-C3 + DX-LR02 UART/AT 모듈) 코드를 이 키트로 이관한 버전.
> 애플리케이션 로직(BLE·IME·디스플레이·릴레이 프로토콜)은 그대로이고, **LoRa 전송 계층만
> DX-LR02 UART transparent → SX1262 SPI(RadioLib)** 로 교체했다. 프로토콜 한 줄 = LoRa 패킷 1개.
> 온에어 포맷이 동일해서 기존 DX-LR02 페이저 / T-Deck / Heltec relay와 그대로 통신된다.

> **프로토콜: mesh PROTOCOL.md v1.22** (정본: `../gopher-over-lora/lora/PROTOCOL.md`).
> 같은 LoRa 스택을 쓰는 두 번째 역할로 **차량 페이저(P01)** 가 [`pager-vehicle/`](pager-vehicle/README.md)에 있다
> — BME280 온습도, LoRa discovery, 주차 중 `!CAR` 상태 비콘, Bluefy 웹 대시보드(BLE).

## 하드웨어

- 보드: **Seeed XIAO ESP32S3** ([XIAO ESP32S3 & Wio-SX1262 Kit](https://www.seeedstudio.com/Wio-SX1262-with-XIAO-ESP32S3-p-5982.html))
- LoRa: **Wio-SX1262** (Semtech SX1262, SPI) — XIAO와 B2B 커넥터로 스택
- OLED: SSD1306 128x64 I2C (주소 0x3C) — **별도로 연결** (키트 미포함)
- BLE 키보드: HOGP (Service 0x1812) 광고하는 어떤 키보드든

### 핀 배선 (XIAO ESP32S3 ↔ Wio-SX1262, B2B 커넥터 고정)

LoRa는 B2B 커넥터로 자동 연결되므로 납땜 불필요. Meshtastic `seeed_xiao_s3` 변종과 동일한 매핑:

```
신호            GPIO      신호            GPIO
------------    ----      ------------    ----
SPI SCK          7        LoRa NSS/CS      41
SPI MISO         8        LoRa DIO1(IRQ)   39
SPI MOSI         9        LoRa BUSY        40
RF 스위치        DIO2(내부) LoRa RESET      42
TCXO            DIO3 @1.8V
```

### OLED 배선 (직접 연결)

GPIO 7/8/9는 이제 LoRa SPI가 쓰므로, OLED I2C는 XIAO 기본 I2C 핀으로 옮겼다:

```
XIAO ESP32S3       OLED
------------       ----
3V3           -->  VCC
GND           -->  GND
GPIO 5 (D4/SDA) <-> SDA
GPIO 6 (D5/SCL) <-> SCL
```

> 남는 유저 핀은 D0(GPIO1), D6(GPIO43), D7(GPIO44) 등. I2C(D4/D5)와 LoRa(SPI+제어핀)는
> 서로 충돌하지 않는다.

## Arduino IDE 셋업

1. 보드 매니저: **esp32 by Espressif Systems** (3.0 이상) 설치
2. 보드 선택: **`XIAO_ESP32S3`** (Tools ▸ Board ▸ ESP32 Arduino ▸ XIAO_ESP32S3)
3. 보드 옵션 (Tools 메뉴):
   - **USB CDC On Boot**: `Enabled`  ← native USB로 Serial 출력 (필수)
   - **Flash Size**: `8MB`
   - **Partition Scheme**: `8M with spiffs` (또는 기본값)
   - **PSRAM**: `OPI PSRAM` (XIAO ESP32S3는 8MB PSRAM 탑재)
   - **Upload Speed**: `921600`
4. 라이브러리 매니저로 설치:
   - **RadioLib** (jgromes) — 6.6 이상 (SX1262 드라이버)
   - **NimBLE-Arduino** (h2zero) — 2.x
   - **U8g2** (olikraus) — 2.34 이상
5. 스케치 폴더(`pager-lora-seeeed/`)를 열고 업로드

### 업로드 안 될 때

- XIAO ESP32S3: `BOOT` 버튼 누른 채 USB 연결 → 다운로드 모드 → 업로드 → 끝나면 `RST` 톡

## 사용법

1. 업로드 후 Serial Monitor (115200 baud) 열기 — 부팅 시 `[LORA] SX1262 begin OK` 확인
2. OLED 상태바에 `BLE: scanning...` 표시
3. 키보드 페어링 모드로 진입 → XIAO가 자동 연결
4. 키를 누르면 OLED에 글자가 표시됨
5. **Left Alt**로 한/영 토글 — 상태바 우측에 `[Kor]` 또는 `[Eng]`
6. **Cmd(GUI)+Enter** 로 LoRa 송신, **Cmd+.** 로 TX/RX 뷰 토글, **Ctrl+.** 로 Config(발신자 ID)

### Serial 디버그 입력
BLE 없이도 IME만 테스트 가능. Serial Monitor 입력창에 타이핑하면 그대로 IME로 들어간다.
- 백틱 `` ` ``: 한/영 토글 · Backspace/Enter/Space 동일 동작
- `X`/`H`: SX1262 RF 설정 + 핀 맵 dump
- `R`: 마지막 수신 패킷 RSSI/SNR
- `N`: 들린 노드 테이블(discovery) + dedup 링 통계 dump
- `~`: BLE 본드 전체 삭제 후 재스캔
- `Cnn`/`B`: SX1262에선 no-op (주파수는 `lora_rf.h` 고정, UART 브리지 없음)

## LoRa RF 파라미터

`lora_rf.h`가 세 리포(pager / T-Deck / Heltec relay)가 공유하는 단일 PHY 소스다.
기본값: 922.0 MHz, SF9, BW 125 kHz, CR 4/6, sync 0x12, preamble 8, **CRC on**(2026-09-14~,
explicit header가 CRC 유무를 싣기 때문에 노드별로 켜도 flag day가 아님), +22 dBm.
바꾸려면 `lora_rf.h`만 수정하고 **모든 노드를 재플래시**해야 서로 들린다. SX1262는
`radio.begin()`에서 이 값을 바로 설정하므로 DX-LR02 같은 AT 설정 단계가 없다.
`lora_rf.h`와 `relay.h`는 t-deck-os / pager-lora-qwerty와 **byte-identical**로 유지한다.

## 프로토콜 구현 범위 (v1.22 엔드포인트)

`lora.cpp`가 구현하는 것 — 키보드 페이저와 차량 페이저가 같은 파일을 쓴다:

- **§4 봉투**: `R|src|pktid|ttl|line`, grammar 검증(ttl ≤ 3, src = `[A-Z][0-9A-F]{2}`), 뒤에 붙은 CR/LF/NUL strip, dedup 링 256 + ghost 통계
- **§5 클래스**: L0(`HB`/`PING`/`PONG`) · L1(`!TYPE`, 모르는 타입은 조용히 drop) · L2(`[SOF]`…`[EOF]`)
  - `!!` escape — escape 1바이트를 자를 때 미리 예약(59 B)해서 ≤ 60 B 불변식 유지
  - 클래스 검사는 열린 프레임 검사 **뒤에** (프레임 안의 bare 라인은 정의상 사용자 텍스트)
  - 재조립은 **봉투 src별** 프레임, idle 20 s 만료 시 "잘림" 표시해 전달 (v1.19)
  - 자기 `[SOF]`…`[EOF]` 사이엔 아무것도 안 보냄 — TX 태스크가 프레임 내내 라디오 뮤텍스를 쥔다
- **§8 타이밍**: 응답은 받은 패킷의 4×ToA 뒤에, 청크 간 2×ToA+50 ms, 모든 TX 전 LBT(CAD, non-persistent)
- **§8a 비콘 양보**: `!GR`/`!BR`/`!VA` announce와 흐르는 스트림(`!GD`/`!BD`/0xC2 보이스)에 HB·비콘이 양보, 상한은 자기 주기 1회
- **§10 Range (v1.21)**: 브로드캐스트 `PING`엔 침묵, `<dst>`가 자기 id인 `PING`에만 `PONG`
- 뉴스/북/보이스 플레인은 소비하지 않는다 (프레임은 버리되 §8a 예약에는 반영)

하드웨어 없이 돌리는 검증: `tools/hosttest/run.sh` (목 Arduino/RadioLib 위에서 `lora.cpp` 58개 체크).

## 한글 표시 범위

U8g2 `u8g2_font_unifont_t_korean1` 폰트 사용. KS X 1001 완성형 약 2,350자 지원.
일상 한글은 모두 커버하지만 비표준 음절(예: "똠")은 □로 표시될 수 있다.

## 알려진 한계

- 첫 페어링 시 Just Works 방식. Passkey 요구하는 키보드는 미지원.
- HID boot keyboard 8바이트 report 포맷을 가정 (대부분의 BLE 키보드 호환).
  키가 안 잡히면 Serial 로그 raw 바이트 확인 후 `ble_hid_host.cpp`의 `parseReport()` 수정.
- CRC를 아직 안 켠 노드(DX-LR02)의 프레임은 PHY 검증 없이 올라온다 — `R|` grammar 검증이 일부만 걸러낸다 (PROTOCOL §9).
- 한자 변환, 자동완성 없음.

## 파일 구조

```
pager-lora-seeeed.ino   메인 (setup/loop, glue)
config.h                핀/상수 (XIAO ESP32S3 + SX1262)
lora_rf.h               공유 LoRa PHY 파라미터 (3개 리포 공통)
lora.h/.cpp             SX1262 RadioLib 드라이버 + PROTOCOL v1.22 엔드포인트 스택 (pager-vehicle/와 공유)
relay.h                 릴레이 계층 (R| 헤더 wrap/parse/검증/dedup) — 전 리포 공통
pager-vehicle/          차량 페이저(P01) 스케치 (별도 스케치 폴더)
docs/index.html         차량 페이저 Bluefy 웹 대시보드 — GitHub Pages: https://zobithecat.github.io/pager-lora-seeeed/
tools/hosttest/         lora.cpp 호스트 목 테스트
hid_keymap.h/.cpp       USB HID Usage → ASCII
keymap_dubeolsik.h      두벌식 QWERTY → 자모
hangul_ime.h/.cpp       자모 조합기 (cho/jung/jong 상태머신)
display.h/.cpp          U8g2 SSD1306 래퍼
ble_hid_host.h/.cpp     NimBLE BLE Central, HID Host
```
