# 엣지 라우터 에이전트 전언 — 차량 관측(car observe) 배포·감사 (PROTOCOL v1.23)

보낸 곳: pager-lora-seeeed 세션 · 2026-09-23 · 대상 리포: `rpi3-lora-edge-router` (브랜치 `feat/v1.11-router-plane`)

## 이미 들어간 것 (커밋 `8bf357a`, 전체 테스트 통과)

- `wire.py` `L1_TYPES["CAR"]` — 9필드, maxsplit 없음(뒤에 붙는 미래 필드는 무시).
- `state.cars[<id>]` + `/status` 스냅샷 `"cars"`: `state P|D`, `parked/hot/low`, `temp/hum/hpa`(`-`→`None`), `vbat_mv`, `pct`(1셀 OCV 환산, 차량과 같은 표),
  `up_s`, `nbr`, `hops`, `rssi/snr`, `ts`, `beacons`(누적). SSE 이벤트 `car`.
- `services._on_car()`: 관찰 전용. `H`/`L`이 **처음** 서면 `log.warning`, 주차↔주행 전환 `log.info`. 채팅에 안 섞임, 아무 동작도 트리거 안 함.
- `range_ping(dst)` / `POST /ping {"dst":"P01"}` → `PING\t<seq>\tE01\tP01` (v1.21 `<dst>`). **= 슬립 중인 차 깨우기**(§10 v1.23: PONG + 30분 각성).
- 대시보드 **🚗 차량** 카드 + 행별 **깨우기** 버튼.
- 테스트: `test_a_car_beacon_is_observed_not_chatted`, `test_an_addressed_ping_names_the_target`.

스펙: `gopher-over-lora/lora/PROTOCOL.md` v1.23 §5 "Vehicle plane — `!CAR`", §10 "An addressed PING is also a wake-up" (PR #27 병합됨).

## 작업

1. **배포**: Pi에서 `git pull` → `edged` 재시작. 대시보드에 🚗 카드가 보이는지, `/status`에 `cars`가 있는지.
2. **첫 검증**: 차량(P01)이 지금 USB 주행 상태라 비콘을 안 낸다. 대시보드 **깨우기** → Range 패널에 `PONG` src=P01이 찍히는 것으로 링크 확인.
   차량이 주차(USB 분리)로 바뀌면 `!CAR ... P ...`가 5분 간격(각성) / 3분 간격(슬립)으로 들어온다.
3. **`docs/PROTOCOL.md` 사본 동기화**: 현재 사본은 정본보다 오래됐다(v1.22 이전). 정본 v1.23과 **byte-identical**로 맞출 것.
4. **(선택) 알림 싱크**: `sinks.py`가 외부 알림(Telegram 등)을 지원한다면 `H`(과열) 최초 발생을 알림으로 올리는 것을 고려. 주차된 차 안 LiPo 과열은 실제 화재 위험이고 `!CAR`의 `H`가 유일한 경고다. `L`(배터리 부족)은 info 수준.
5. **(선택) 이벤트 싱크 로깅**: `!CAR` 수신을 event sink에 남겨 하루 단위 배터리 곡선(mV) 추적. 사용자가 분압 저항을 달면 `vbat_mv`가 채워진다(지금은 `-`).

## 감사(audit) 항목

- [ ] **주소지정 PING 무시 규칙(v1.21)**: `_rx_l0`의 PING 처리가 `<dst>`가 있고 우리(E01)가 아니면 **PONG하지 않는지** 확인. 지금 코드는 `f.get("seq")`만 보고 `schedule_pong`를 부르는 것처럼 보인다 — T-Deck이 P01을 깨우려고 쏜 PING에 라우터가 같이 PONG하면 §8 응답 그리드를 오염시킨다.
- [ ] **PONG 응답자 식별**: Range 패널이 `env.src`(P01)로 집계하는지(코드상 그렇다). PONG의 `<id>` 필드는 표시 이름 `CAR01`이다.
- [ ] **reply ttl(§8 v1.22)**: 주소지정 PING은 요청이 아니라 우리가 보내는 쪽이므로 해당 없음. 단 `range_ping(dst)`는 ttl=3으로 나가야 relay를 타고 주차장까지 간다 — 확인됨.
- [ ] **§8a 비콘 양보**: `!CAR`는 beacon-class다. 라우터가 `!CAR`를 스트림 announce로 오인해 자기 `!RB`를 미루는 경로가 없는지(`_hold`/예약 로직).
- [ ] **dedup 링 ≥ 256**(§7, 포워더): 차 비콘은 ttl=3이라 직접+relay 사본이 온다.
- [ ] **`nav`/`_nav_from`**: `!CAR`엔 위치 필드가 없다. `_nav_from(kind, f, env)`가 모르는 kind에서 조용히 지나가는지.
- [ ] **peers 테이블**: `_rx_l1`이 `note_peer(src, ..., ttl)`로 P01을 이웃에 올린다. ttl=3 출발이라 hops 산술 유효 — HB(ttl=1)와 달리 hops 표시가 맞는지.
- [ ] **라우터 트래픽이 차를 깨운다**: 슬립 중인 차는 **어떤 프레임에든** DIO1로 깬다. 라우터(E01) `!RB`+HB(~30 s)가 근거리에서 차를 분당 1회 깨운다. 라우터가 고칠 건 아니지만(§1 규칙대로 동작 중), 차량 배터리 수명 추정 시 이 사실을 반영할 것. 차량 펌웨어 쪽에서 비주소 웨이크 각성 창 단축 예정.

## 합격 기준

1. 배포 후 `/status.cars`에 P01이 나타나고 대시보드 카드에 주차/주행·온습도·배터리%가 맞게 표시된다(`-`는 `—`로).
2. 깨우기 버튼 → `PING\t<seq>\tE01\tP01` 송신 로그 → PONG src=P01 수신.
3. 감사 1번(주소지정 PING을 남에게 PONG하지 않음)이 테스트로 고정된다.
4. `docs/PROTOCOL.md`가 정본과 `cmp`로 동일하다.
