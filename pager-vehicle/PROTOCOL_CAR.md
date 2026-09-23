# `!CAR` — 차량 상태 비콘

정본은 **`gopher-over-lora/lora/PROTOCOL.md` v1.23** 에 병합됐다 (§5 "Vehicle plane — `!CAR`",
§10 "An addressed `PING` is also a wake-up"). 프레임 포맷·플래그·주기·슬립/웨이크 규칙은 거기가
기준이고, 이 파일은 포인터만 남긴다. 구현은 `pager-vehicle/pager-vehicle.ino`(`build_beacon`,
`sleep_tick`, `stay_awake`).

요약:

```
!CAR\t<id>\t<st>\t<t_c>\t<rh>\t<hpa>\t<vbat_mv>\t<up_s>\t<nbr>      ttl=3, beacon-class
st = P|D + flags H(과열) L(배터리 부족)      vbat_mv → % 환산은 수신측 (1셀 LiPo OCV)
주기: 각성 주차 300 s · 슬립 중 180 s(타이머 wake마다) · 주행 off · 전환 시 1발
wake 신호(30분 각성): 주소지정 PING(<dst>=P01) · L2 채팅 · BLE 연결
```
