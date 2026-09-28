# P10 엣지라우터(edged.py) 에이전트 전언 — 차량 평면 `!CAR` 지원 (PROTOCOL v1.23)

보낸 곳: pager-lora-seeeed 세션 · 2026-09-28 · 대상 리포: `zobithecat/BLE_6_lora_combo` `edge-router/edged.py`
(P10 = Pi + ME25LS02 콤보 보드. 보드 펌웨어는 수정하지 않고 콘솔 로그 파싱 + `pager sysb64` 송신 구조)

## 배경 (검증된 사실)

- 새 노드 **P01 = 차량 페이저**(XIAO ESP32S3 + Wio-SX1262 + SHT35 + LiPo, 22 dBm). HB 표시 이름 기본 `CAR01`.
- 정본 **v1.23** 병합 완료 (`gopher-over-lora/lora/PROTOCOL.md`, PR #27):
  - §5 "Vehicle plane — `!CAR`": `!CAR\t<id>\t<st>\t<t_c>\t<rh>\t<hpa>\t<vbat_mv>\t<up_s>\t<nbr>`
    - **ttl=3로 출발(relay 탐)**, beacon-class(§8a). `st` 첫 글자 `P`(주차)/`D`(주행) + 플래그 `H`(실내 ≥60 °C) `L`(배터리 ≤15 %). 모르는 플래그는 무시.
    - 값이 `-`면 미측정이다(0이 아님). 지금 차량은 SHT35라 `hpa`는 항상 `-`. `vbat_mv`는 **2026-09-28부터 실측값이 실린다**(분압 보정 완료).
    - % 환산은 수신측이 한다. 1셀 LiPo OCV 표: 4200=100, 4150=95, 4110=90, 4080=85, 4020=80, 3980=75, 3950=70, 3910=65, 3870=60, 3850=55, 3840=50, 3820=45, 3800=40, 3790=35, 3770=30, 3750=25, 3730=20, 3710=15, 3690=10, 3610=5, 3270=0 (선형 보간). `st`가 `D`면 충전 전압이라 높게 나온다.
    - 주기: 각성+주차 300 s, **슬립 중 정확히 180 s**, 주행 중 off, 전환 시 1발.
  - §10 "An addressed PING is also a wake-up": 차는 주차 30분 뒤 딥슬립한다. **`PING\t<seq>\t<id>\tP01`(v1.21 `<dst>`)을 받으면 PONG + 30분 각성**. 브로드캐스트 PING으로는 안 깬다.
- **E01(집 엣지 라우터, `rpi3-lora-edge-router`)이 같은 기능을 이미 배포·검증했다**(`8bf357a`, `6048d07`). 참고 구현이니 동작·표시를 맞출 것:
  `state.cars`, 대시보드 🚗 카드, 깨우기 버튼(`POST /ping {"dst":"P01"}`), 입차/출차 기록(`car_in`/`car_out`), `H` 최초 발생 `car_hot` 이벤트.
- 실측(E01 경유, 2026-09-24): 주차 → 2분(테스트 빌드) 뒤 HB 끊김 → `!CAR` 정확히 180 s 간격 → E01 주소지정 PING → **2초 만에 PONG** → HB 재개.

## 작업

1. **`!CAR` 수신 파싱** — `State.apply()`의 `lora_message` L1 분기(지금 `!FS`를 처리하는 자리)에 추가.
   `self.cars[<id>] = {state, parked, hot, low, temp, hum, hpa, vbat_mv, pct, up_s, nbr, hops, rssi, snr, ts, beacons}`.
   `-`는 `None`. 채팅에 절대 안 섞임(이미 `!` 라인은 채팅 제외 — 유지).
2. **`snapshot()`에 `"cars"`** 추가, MQTT `edge/<anchor>/status`에도 실림. JSONL 이벤트로 `car`(수신마다), `car_hot`(H 최초), `car_in`/`car_out`(P↔D 전환 — **첫 목격은 이벤트 아님**, 시작 시점을 모르니까).
3. **`hop_label()`에 `!CAR` 추가** — ttl=3 고정 출발이라 `fixed3` 목록에 넣을 것. 지금은 `!`로 시작하는 목록 밖 타입이라 hops 라벨이 안 붙는다.
4. **깨우기**: `POST /ping {"dst":"P01"}` (또는 기존 라우트 패턴에 맞는 이름) → `PING\t<seq>\tP10\tP01` ttl 3 송신. 대시보드 차량 카드에 **깨우기** 버튼.
   - ⚠️ 송신 경로 확인 필요: `send_l1_line()`/`pager sysb64`가 **`!`로 시작하지 않는 L0 라인(`PING…`)도 그대로 내보내는지** 보드 펌웨어 동작을 확인할 것. 거부하거나 변형한다면 보드 콘솔에 PING 전용 명령이 있는지 찾아보고, 없으면 그 사실을 보고(펌웨어는 수정 금지 원칙).
5. **대시보드 🚗 카드** — 주차/주행, `H` 강조(실제 화재 위험), `L`, 온습도, 배터리 %(mV), 홉·RSSI, 마지막 수신, 비콘 횟수, 깨우기 버튼. E01 카드와 같은 문구 권장.
6. **테스트** — `test_parser.py` 스타일로: `!CAR` 파싱(`-`→None, 3905 mV→64 %, ttl 2 → 1홉), 채팅 미유입, 모르는 플래그 무시, 뒤에 붙은 10번째 필드 무시.

## 감사(audit) 항목

- [ ] **보드 펌웨어의 PONG 규칙(v1.21)**: 남에게 주소지정된 PING(`<dst>`가 P10 아님)에 **보드가 PONG하지 않는지**. E01은 이게 실제 결함이었다. P10은 PONG을 보드 펌웨어가 하므로 데몬에서 못 고칠 수 있다 — 그렇다면 콘솔 로그로 증상만 확인해 보고할 것(T-Deck/E01이 P01을 깨울 때마다 P10도 PONG하면 §8 응답 그리드 오염).
- [ ] **`!CAR`를 §8a 스트림 announce로 오인하지 않는지**: `nav_seconds()`/`request_meta()`/`overheard_key()`가 `!CAR`를 예약·보류 대상으로 잡으면 P10의 `!RB`/HB/CS가 불필요하게 밀린다. `!CAR`는 beacon-class다.
- [ ] **dedup**: 차 비콘은 ttl=3이라 직접+relay 사본이 온다. 보드 펌웨어 dedup(≥256, §7)이 거르는지, 데몬에서 중복 행이 안 생기는지.
- [ ] **`lora_neighbour`/`lora_beacon` 테이블**: P01이 HB로 이웃에 오르는지, `!CAR`만 오는 슬립 상태(HB 없음)에서도 "살아 있음"으로 보이게 `last_seen`을 `!CAR` 수신으로도 갱신할지 결정 — **권장: 갱신**. 안 그러면 슬립 중인 차가 이웃 목록에서 stale로 빠진다.
- [ ] **CS 텔레메트리 주기**: P10의 CS 보고(10 s)와 `!RB`/HB가 근거리의 슬립 중인 차를 계속 DIO1로 깨운다(차는 비주소 프레임이면 3 s 후 재슬립하도록 이미 줄였음). 고칠 대상은 아니지만 차량 배터리 추정에 영향 — 보고서에 P10 송신 주기만 적어줄 것.
- [ ] **`docs`/정본 사본**: 이 리포에 `PROTOCOL.md` 사본이 있으면 정본 v1.23과 `cmp` 동일하게.

## 합격 기준

1. P01의 `!CAR`가 `/status.cars.P01`과 대시보드 카드에 주차/주행·온습도·배터리 %로 표시된다(`hpa`는 `—`).
2. 깨우기 → 콘솔/프레임 로그에 `PING\t<seq>\tP10\tP01` ttl 3 송신 → `PONG\t<seq>\tCAR01` src P01 수신.
3. T-Deck이나 E01이 P01을 깨울 때 P10은 PONG하지 않는다(감사 1번) — 또는 펌웨어 한계로 못 막는다면 그 사실이 보고된다.
4. 차가 슬립 중(HB 없음, `!CAR`만 180 s)일 때도 대시보드에서 "살아 있음"으로 보인다.
5. 새 테스트 통과, 기존 `test_parser.py`·`test_dashboard_js.py` 회귀 없음.

## 참고

- 차량 펌웨어: `zobithecat/pager-lora-seeeed` `pager-vehicle/` (`config.h`: `VEH_SLEEP_WAKE_S 180`, `VEH_AWAKE_MS 30분`, `LORA_TX_DBM 22`, LBT on).
- E01 참고 구현: `zobithecat/rpi3-lora-edge-router` `edge_router/services.py` `_on_car()`, `lipo_pct()`, `state.cars`, `dashboard.html` 🚗 카드.
