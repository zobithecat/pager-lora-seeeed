# pager-vehicle — 차량 페이저 (P01)

차에 달아두는 LoRa 메시 노드. 화면도 키보드도 없고, UI는 폰의 **Bluefy 웹 대시보드**(BLE)다.

| 기능 | 내용 |
|------|------|
| 온습도 센싱 | I2C 환경 센서 자동 인식, 10초 주기 forced 측정. **BME280**(온도·습도·기압) / BMP280 / **BMP390·BMP388**(CJMCU-390, 온도·기압만 — 습도 없음) |
| LoRa discovery | 유효한 봉투를 하나라도 들은 노드 전부 — id / 이름(HB) / RSSI·SNR(직접 수신분만) / 홉 / 마지막 상태 프레임 |
| 주차 비콘 | 주차 중(= LiPo 구동) 5분마다 `!CAR` 상태 비콘. 포맷은 [PROTOCOL_CAR.md](PROTOCOL_CAR.md) |
| BLE 대시보드 | 폰이 Central. 상태 · 주변 노드 · 비콘 · 채팅(L2 송수신) · 재난경보(`!AL`) 표시 |
| 프로토콜 | mesh **PROTOCOL.md v1.22** 엔드포인트. routing id **`P01`** |

## 하드웨어

- Seeed **XIAO ESP32S3 + Wio-SX1262** 키트 (B2B 커넥터, 납땜 불필요)
- 환경 센서 I2C 모듈 → `3V3` / `GND` / `SDA=D4(GPIO5)` / **`SCL=D7(GPIO44)`**. 주소 0x76/0x77 자동 탐색
  - SCL이 XIAO 기본(D5)이 아닌 이유: 이 보드는 D5(GPIO6)·D6(GPIO43)이 무배선 상태에서도 Low로 잡혀 있었다(브리지/손상). 정상 보드면 `config.h`에서 6으로 되돌려도 된다
  - **CJMCU-390(실제 칩 BMP388, id 0x50)은 SPI로 읽는다** — 4선이 전부 GPIO라 I2C 모드 잠김 문제가 없다:
    `CSB`→**D1(GPIO2)**=CS, `SCL`→**D7(GPIO44)**=SCK, `SDA`→**D4(GPIO5)**=MOSI(SDI), `SDO`→**D3(GPIO4)**=MISO. `INT` 비움
  - 같은 선에 BME280을 I2C로 꽂아도 된다(probe가 I2C BME280 → SPI BMP3 순으로 시도)
  - 안 잡히면 Serial `I` — 버스 스캔 + SDA/SCL 선이 떠 있는지/Low로 잡혔는지 알려준다
- **LiPo** 1셀 → XIAO 뒷면 `BAT+`/`BAT-` 패드. USB가 꽂혀 있는 동안 XIAO가 충전한다
- 전원: 차량 USB 포트

> ⚠️ **LiPo + 여름철 주차 차량.** 실내는 70 °C를 넘기고 LiPo는 60 °C 위에서 위험하다.
> 직사광선이 닿는 대시보드 위는 피하고, 펌웨어는 60 °C 이상이면 비콘에 `H` 플래그를 세우고
> 대시보드에 경고를 띄운다 (`VEH_TEMP_WARN_C`). 경고일 뿐 보호 회로가 아니다.

### 주차 감지

추가 배선 없이 **USB 호스트가 이 장치를 열거했는가**로만 판단한다 (10초 디바운스).

- 호스트 있음 → **주행(D)**: 비콘 off, BLE 광고 빠르게, CPU 풀클럭
- 호스트 없음 → **주차(P)**: `!CAR` 비콘 on, BLE 광고 느리게, CPU 80 MHz

**데이터선이 없는 단순 USB 충전기(시거잭 어댑터 대부분)에서는 항상 "주차"로 보인다.**
헤드유닛의 데이터 USB 포트에 꽂거나, `5V` 핀을 100k/100k로 분압해 빈 GPIO에 넣고
`config.h`의 `VEH_VBUS_SENSE_PIN`에 그 핀 번호를 주면 전원 유무로 판단한다.

### 배터리 전압 / 잔량

XIAO ESP32S3는 BAT 패드가 ADC에 연결돼 있지 않다. 저항 2개로 분압해 **D0(A0, GPIO1)** 에 넣어야 읽힌다:

```
BAT+ ──[ R1 200k ]──┬──[ R2 200k ]── GND
                    └── D0 / A0 (GPIO1)        (선택) D0–GND 사이 100 nF
```

- R1 = R2면 아무 값이나 된다(100k–220k 권장, 상시 소모 ~10 µA). 비율이 다르면 `VEH_VBAT_DIVIDER = (R1+R2)/R2`
- 4.2 V → 핀에는 2.1 V. **BAT+를 GPIO에 직접 꽂으면 안 된다** (3.3 V 초과)
- 멀티미터로 잰 값과 다르면 `VEH_VBAT_CAL = 실측 / 표시값`
- 배선이 없으면 핀이 떠서 2.5–4.5 V 밖의 값이 읽히고, 펌웨어는 그걸 "측정 불가"(`-`)로 처리한다
- 잔량 %는 1셀 LiPo OCV 곡선 환산. USB가 꽂혀 있는 동안은 충전 전압이 읽혀 실제보다 높게 나온다
- 주차 중 15 % 이하(`VEH_BATT_LOW_PCT`)면 비콘 `st`에 `L` 플래그를 세우고 즉시 1발 보낸다

주차 중에도 LoRa는 계속 수신 상태다(discovery·채팅 수신 때문에 deep sleep을 안 쓴다).
대략 50–70 mA — 1000 mAh LiPo로 하루가 안 된다. 며칠씩 세워둘 거면 큰 셀을 쓸 것.

## 빌드

Arduino IDE / arduino-cli, 보드 `esp32:esp32:XIAO_ESP32S3` (esp32 core 3.x), PSRAM `OPI PSRAM`,
USB CDC On Boot `Enabled`. USB Mode는 둘 다 지원한다(기본 USB-OTG/TinyUSB, 또는 Hardware CDC).
라이브러리: **RadioLib** 7.x, **NimBLE-Arduino** 2.x. (BME280 드라이버는 내장 — 외부 라이브러리 없음)

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi pager-vehicle
```

Serial(115200): `S` 상태 · `I` I2C 진단 · `N` 노드 테이블+dedup 통계 · `X` RF 설정 · `R` 마지막 RSSI · `B` 비콘 즉시 · `M<text>⏎` 채팅 송신

## 대시보드 (Bluefy)

[`../docs/index.html`](../docs/index.html) 한 파일짜리 정적 페이지. Web Bluetooth는 **HTTPS에서만** 동작하므로
GitHub Pages(`main` 브랜치 `/docs`)로 서빙한다 → **https://zobithecat.github.io/pager-lora-seeeed/**
iOS **Bluefy** 앱(또는 Android/데스크톱 Chrome)에서 이 주소를 연다.
`연결` → `PAGER-P01` 선택. 끊기면 같은 장치로 조용히 재연결을 시도한다.

폰이 없는 동안 받은 채팅/경보는 최근 16개를 RAM에 보관했다가 연결 시 재생한다(재부팅하면 사라짐).

### BLE 링크 프로토콜 (Nordic UART Service)

- 장치→폰 (`6E400003…`, notify): JSON 한 줄 + `\n`. MTU에 맞춰 조각나 오므로 `\n`으로 재조립
  - `hello` 장치 정보 · `st` 상태(5초) · `nodes`+`node`×n 노드 목록 · `msg` 채팅 · `al` 경보 · `bcn` 보낸 비콘 · `txdone` · `err`
- 폰→장치 (`6E400002…`, write): 텍스트 한 줄 + `\n`
  - `GET` 전체 재전송 · `MSG <text>` 채팅(≤480 B) · `ID <name>` 표시 이름(NVS) · `BCN` 비콘 즉시

`VEH_BLE_PASSKEY`에 6자리 값을 주면 폰→장치 쓰기(= 이 노드 이름으로 메시에 송신할 권한)에
passkey 본딩을 요구한다. 기본은 0(열림) — 주차된 차 근처의 누구나 연결해 글을 쓸 수 있다는 뜻이니,
첫 연결이 확인되면 켜는 걸 권한다.

## 무선 규정 메모 (PROTOCOL §2)

실외·무인 노드라서 legal profile을 따른다: **14 dBm**(`LORA_TX_DBM`, 안테나 이득 포함 EIRP 기준 —
고이득 안테나면 더 내릴 것) + **모든 송신 전 LBT**(CAD, non-persistent 백오프).
주파수는 플릿과 들리려면 `lora_rf.h`의 922.0 MHz를 따를 수밖에 없다. 스펙의 legal 채널은
922.1 MHz이므로, 플릿 전체를 옮기는 flag day 전까지는 이 점이 남는 숙제다.

## 공유 파일

`lora.h` `lora.cpp` `relay.h` `lora_rf.h`는 상위 폴더(키보드 페이저)와 **byte-identical**이어야 한다.
역할 차이는 전부 `config.h` 매크로와 콜백으로 표현한다. 한쪽을 고쳤으면:

```bash
for f in lora.h lora.cpp relay.h lora_rf.h; do cmp $f pager-vehicle/$f; done
```

프로토콜 로직은 하드웨어 없이 `tools/hosttest/run.sh`로 검증할 수 있다.
