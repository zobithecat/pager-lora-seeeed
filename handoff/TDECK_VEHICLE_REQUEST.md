# T-Deck 에이전트 전언 — 차량 페이저(P01) 대응 (PROTOCOL v1.23)

보낸 곳: pager-lora-seeeed 세션 · 2026-09-23 · 대상 리포: `t-deck-os`

## 배경 (검증된 사실)

- 새 노드 **P01 = 차량 페이저** (XIAO ESP32S3 + Wio-SX1262 + SHT35 + LiPo). 화면 없음, 폰 BLE 대시보드만.
  표시 이름(HB의 id 필드) 기본값 `CAR01`. 코드: `zobithecat/pager-lora-seeeed` `pager-vehicle/`.
- 정본 스펙 **v1.23**으로 병합됨 (`gopher-over-lora/lora/PROTOCOL.md`, PR #27):
  - §5 "Vehicle plane — `!CAR`": `!CAR\t<id>\t<st>\t<t_c>\t<rh>\t<hpa>\t<vbat_mv>\t<up_s>\t<nbr>`,
    **ttl=3(relay 탐)**, beacon-class(§8a). `st` 첫 글자 `P`(주차)/`D`(주행) + 플래그 `H`(실내 ≥60 °C, LiPo 위험) `L`(배터리 ≤15 %).
    `-` = 미측정. `vbat_mv`만 싣고 %는 수신측 환산(1셀 OCV: 4200=100, 3840=50, 3690=10, 3270=0).
  - §10 "An addressed PING is also a wake-up": 주차 30분 뒤 차는 **딥슬립**하고 180 s마다 깨서 비콘 1발.
    슬립 중에도 SX1262 RX 듀티사이클로 듣고 있어 **어떤 프레임이든 DIO1로 깨어나** 처리하지만,
    **`PING\t<seq>\t<id>\t<dst=P01>`**(v1.21의 `<dst>`), L2 채팅, BLE 연결만 **30분 각성**을 만든다. 그 외 프레임은 25 s 뒤 재슬립.
- 실기기 검증됨: SHT35 값이 `!CAR`에 실려 나감 / 주소지정 PING 주입 → 4×ToA 뒤 `PONG\t<seq>\tCAR01` 송신 + 30분 각성 /
  딥슬립 → 메시 프레임에 DIO1 웨이크 → 깨운 프레임을 RX 버퍼에서 읽어 처리. **T-Deck이 실제 전파로 쏜 PING은 아직 미검증**(아래 합격 기준).

## 작업

1. **`!CAR` 수신 표시.** L1 디스패치에 `CAR` 추가. Discovery(노드 목록)의 P01 행에 상태를 보여줄 것:
   `주차/주행`, `H`→과열 강조(실제 화재 위험 — 눈에 띄게), `L`→배터리 부족, 온도/습도, 배터리 %(mV→OCV 환산), 홉 수(ttl=3 출발이라 §10 산술 유효), 마지막 수신 시각.
   채팅에 절대 섞지 말 것. 모르는 플래그 글자는 무시.
2. **Range 앱: "차 깨우기".** 대상 노드 id를 지정하는 주소지정 PING(`PING\t<seq>\t<id>\t<dst>`)이 이미 v1.21에 있으니
   P01을 대상으로 한 번에 보낼 수 있는 UI(예: 노드 목록에서 P01 선택 → "PING/깨우기"). PONG 수신 시 "깨어남(30분)" 표시.
3. **채팅 UX (권장).** P01에게 보낼 때 마지막 `!CAR`/HB가 3분 이상 전이거나 상태가 `P`면 "차가 자고 있을 수 있음 — 먼저 PING" 안내,
   또는 자동으로 주소지정 PING → PONG 확인 후 메시지 송신. 이유: 슬립 중인 차는 `[SOF]`에 깨어나 부팅(수백 ms)하는 동안
   **첫 청크를 놓칠 수 있다**(청크 간격 2×ToA+50 ms ≈ 0.75 s). 깨어 있는 상태(각성 30분)에서 보내면 문제없음.

## 감사(audit) 항목

- [ ] **PONG 응답자 식별**: `PONG\t<seq>\t<id>`의 `<id>`는 표시 이름(`CAR01`)이다. Range 집계는 **봉투 src(P01)** 기준이어야 한다(§10 "Why the second field of PONG is the responder"). id 필드로 매칭하면 차량이 안 잡힌다.
- [ ] **주소지정 PING 포맷**: 정확히 4필드 탭 구분, `<dst>` = 3자 routing id(`P01`). 표시 이름(`CAR01`)을 넣으면 차는 두 값 모두 받아주지만 다른 노드는 아니다 — routing id로 통일.
- [ ] **hops 해석**: `!CAR`는 ttl=3 출발 → `hops = 3 − ttl` 유효. HB/`!RB`(ttl=1)는 여전히 unknown 처리(§10).
- [ ] **`\n` 종단**: T-Deck이 모든 송신 줄에 `\n`을 붙이는 것으로 측정돼 있다(§4 v1.16). 차량 수신측은 strip하므로 동작엔 문제없지만, 60 B 예산에 1 B가 더 들어가는지(청커가 split 후 붙이면 61 B) 확인.
- [ ] **§8a 양보와의 관계**: `!CAR`는 beacon-class이므로 T-Deck이 스트림(`!VA`/`!GR`/`!BR`) 중일 때 차가 양보한다. T-Deck 쪽에서 `!CAR`를 스트림으로 오인해 자기 비콘을 미루는 코드가 없는지.
- [ ] **dedup**: 차 비콘은 relay 2홉을 타므로 직접+중계 사본이 온다. `(src,pktid)` dedup 링이 ≥128인지(§7).
- [ ] **알 수 없는 `!` 타입** drop이 조용한지(v1.3 forward-compat) — `!CAR` 구현 전 빌드에서 채팅에 새지 않는지 확인.

## 합격 기준

1. 차량이 각성 상태에서 5분마다 내는 `!CAR`가 Discovery에 상태·온습도·배터리%로 표시된다.
2. T-Deck에서 P01 주소지정 PING → 차의 `PONG` 수신 → Range에 응답자 P01 기록. (차량 시리얼 `S`에 `awake_for≈1800s (addressed PING)`)
3. USB 뽑고 30분 뒤 차가 슬립한 상태에서 같은 PING → PONG이 오고, 이어서 폰 Bluefy 접속이 된다(딥슬립 DIO1 웨이크 end-to-end).
4. 위 과정에서 채팅 창에 `!CAR`나 PONG이 나타나지 않는다.

## 참고

- 차량 펌웨어 설정: `pager-vehicle/config.h` (`VEH_SLEEP_WAKE_S 180`, `VEH_AWAKE_MS 30분`, `LORA_TX_DBM 14`, LBT on).
- 알려진 미해결(차량 쪽, T-Deck 책임 아님): 라우터 옆에서는 HB/`!RB`가 60 s마다 차를 깨워 슬립 효율이 떨어진다 — 차량 펌웨어에서 비주소 프레임 웨이크의 각성 창을 줄일 예정.
