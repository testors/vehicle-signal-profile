# CAN Tools

BLE/Classic BT OBD 어댑터를 통한 CAN 버스 도구 모음. macOS에서 동작하며, ELM327/STN2255 호환 어댑터(OBDLink MX+, vLinker MC+ 등)를 지원한다.
PEAK PCAN-USB로 버스 전체를 수신 전용(listen-only)으로 받을 수도 있다(아래 "PCAN-USB" 절).

## 도구

실행파일은 `build/can_tools` 하나이고, 명령은 `can_tools <명령> [인자...]`로 고른다.

| 명령 | 설명 | CAN 송신 |
|------|------|----------|
| **scan** | BLE/Classic OBD 어댑터 스캔 (JSON), PCBUSB가 있으면 PCAN-USB 채널도 | 없음 |
| **monitor** | CAN 패시브 모니터 (silent mode, ACK 없음) | 없음 |
| **detect** | 버스 종류 감지 (3초 수신) | 없음 (`--probe`면 `0100` 송신) |
| **discover** | PT-CAN 시그널 자동 탐색 (대화형) | 없음 |
| **probe** | VSP 프로파일의 CAN/OBD 시그널을 목표 Hz로 읽을 수 있는지 검증 | `--plan`·`--mode can`은 없음, `--mode obd`는 진단 질의 송신, auto는 상시 송신 프레임이 안 들릴 때만 질의 |
| **replay** | 저장된 캡처를 차량 없이 다시 분석 | 없음 (어댑터 불필요) |
| **query** / **supported** / **raw** | OBD-II PID 조회, 지원 PID 목록, 원시 AT 명령 | 송신 (OBD 포트 전용) |
| **p992** | Porsche 992 legacy/debug command set | 송신 |
| **tui** | btop형 라이브 대시보드: 연결 화면, 시그널·Raw CAN·로그, discover, 녹화, replay | 없음 (TX 모드를 켤 때만 진단 질의 송신) |
| (인자 없음) | 터미널이면 `tui`(연결 화면), 아니면 사용법 출력 후 종료 코드 2 | 없음 |
| **menu** | 예전 대화형 메뉴 | 메뉴에 따라 |

`can_tools help`가 전체 명령과 송신 여부를 보여준다. 빌드하면 `build/` 안에 예전 도구 이름의 심볼릭 링크
`can_reader`, `can_discover`, `vsp_probe`, `discover_replay`가 함께 생기고, 이 이름으로 실행하면 예전 도구의
인자와 도움말을 그대로 쓴다(예: `./build/can_reader monitor vLinker 10`, `./build/vsp_probe <profile> --plan`).
`can_tools discover`는 `can_discover`처럼 장치를 생략할 수 있고, `can_reader discover`는 예전처럼 장치가 필요하다.

`discover`(`can_reader discover`, `can_discover`)는 모두 동일한 탐색 로직(`discover_session.m`, `can_discover.c`)을 공유한다.
어댑터에 연결하는 명령은 모두 같은 연결 경로(`common/obd_connect`)로 스캔·연결·초기화한다. BLE 준비를 최대 10초
기다리고, 8초 스캔해서 못 찾으면 한 번 더 스캔하며, BLE가 꺼져 있어도 페어링된 Classic 어댑터는 목록에 남는다.
장치 이름은 대소문자 없이 포함 여부로 고르고, `discover`에서 생략하면 첫 번째 어댑터를 쓴다. 수신 전용 명령(monitor, detect, discover, probe의 CAN phase)은
PASSIVE 초기화(`ATZ`, `ATE0`, `ATL0`, `STI`, `ATSP6`, `ATCSM1`, `ATS1`, `ATCAF0`, `ATH1`)를, 질의 명령(query, supported, raw)은
ACTIVE 초기화(ELM327 초기화 명령 + STN 옵션)를 쓴다. PASSIVE 초기화는 `ATCSM1` 응답에 `OK`가 있어야 성공한다: 응답이 없거나
`?`(미지원)면 ACK를 보내지 않는다고 확인할 수 없으므로 `[ERROR] silent mode not confirmed: ATCSM1 …`을 남기고 모니터를
시작하지 않는다(TUI 연결 화면에는 `adapter init failed: …`로 이유가 나온다).

## 빌드

vehicle-signal-profile 저장소의 `apps/can_tools` 폴더에서 빌드한다(이 문서의 경로는 모두 이 폴더 기준).
python3가 필요하다: 빌드가 저장소의 `profiles/`(`../../profiles`)에서 prior table을 생성한다(아래 "Prior table 생성").

```bash
./build.sh
```

또는:

```bash
mkdir -p build && cd build && cmake .. && make
```

빌드 결과는 `build/can_tools`와 별칭 심볼릭 링크 4개(`can_reader`, `can_discover`, `vsp_probe`, `discover_replay`)다.
`build/can_tools_fake`는 테스트 전용 빌드다(Bluetooth 대신 가짜 어댑터, 실제 장치에 닿지 않음). TUI는 ncurses를 쓴다(macOS SDK 기본 포함).

빌드 후 `build/libcan_discover_core.a`가 함께 생성된다. 다른 앱에서 탐색 엔진만 쓰고 싶으면
`common/can_discover.h`를 포함해 `disc_analyze_with_raw()`,
`disc_render_draft_dbc()`, `disc_render_draft_vsp()`를 직접 호출하면 된다.

## PCAN-USB (PEAK, 수신 전용)

Bluetooth 어댑터는 프레임이 텍스트로 와서 호스트가 도착 시각을 찍는다(측정: 평소 ~20 ms 지연, 50–130 ms 묶음,
ms 해상도). PEAK PCAN-USB는 USB로 버스 전체를 받고, CAN 컨트롤러가 찍은 µs 타임스탬프를 준다. 이 도구에서
PCAN은 **수신 전용**이다: ACK도 에러 프레임도 어떤 프레임도 보내지 않는다.

**설치.** macOS용 PEAK 드라이버는 없고, UV Software의 PCBUSB 라이브러리(PCAN-Basic 호환, 무료 바이너리)를 쓴다:
<https://github.com/mac-can/PCBUSB-Library>. 배포본의 `install.sh`가 `/usr/local/lib/libPCBUSB.dylib`(+버전 파일)과
`/usr/local/include/PCBUSB.h`를 설치한다(`/usr/local`에 쓰기 권한이 없으면 `sudo`가 필요할 수 있다). 빌드에는 필요 없다:
`can_tools`는 실행 중에 `dlopen`으로 `/usr/local/lib/libPCBUSB.dylib`, `/opt/homebrew/lib/libPCBUSB.dylib` 두 절대 경로만
찾고(현재 폴더나 `DYLD_*` 경로는 보지 않는다), 없으면 PCAN 명령만 설치 링크와 함께 실패하고 나머지는 그대로 동작한다.
**최소 버전 0.13.** 라이브러리를 읽으면 채널을 건드리기 전에 `PCAN_CHANNEL_VERSION`을 읽어 0.13 이상의 0.x만 쓴다
(검증한 버전이 0.13이다). 더 오래되었거나 major가 다르거나 버전을 읽을 수 없으면 찾은 문자열과 함께 거부하고 아무 채널도
초기화하지 않는다. 쓰는 버전은 연결 로그, monitor/detect JSON(`library_version`), TUI의 `session.json`에 남는다.
PCBUSB 0.13 기준 지원 장치: PCAN-USB(IPEH-002021/002022), PCAN-USB FD(IPEH-004022), PCAN-USB Pro FD(CAN1만).
Universal 바이너리라 Apple silicon에서도 동작한다.

**장치 이름.** `<device>` 자리에 `pcan`(첫 번째 사용 가능한 채널) 또는 `pcan1`..`pcan8`(대소문자 무시)을 쓴다.
`--bitrate RATE|auto`: `1M 800k 500k 250k 125k 100k 95k 83k 50k 47k 33k 20k 10k 5k`, 기본 500k. `auto`는
500k, 250k, 1M, 125k, 100k, 83k, 50k, 33k를 차례로 listen-only로 열어 각각 최대 0.5초 듣고, 2개 이상 ID에서 데이터
프레임 20개가 처음 들린 속도를 고른다(시도한 속도와 결과를 로그에 남긴다).

```bash
./build/can_tools scan                                   # BT 어댑터 + PCAN 채널 (transport "usb")
./build/can_tools monitor pcan 10 --out mon.json         # 버스 전체, 10초; ID를 주면 소프트웨어 필터
./build/can_tools monitor pcan1 10 --bitrate auto 0A5 1A0
./build/can_tools detect pcan                            # 3초 수신, 버스 분류 (ATRV 없음, --probe 불가)
./build/can_tools discover pcan                          # 가이드 탐색, phase 덤프는 6열 TSV
./build/can_tools probe <profile.vsp> --device pcan --mode can --seconds 10
./build/can_tools tui --device pcan --bitrate 500k       # 연결 화면에도 PCAN 채널이 "USB"로 보인다
```

monitor JSON에는 `"interface":"pcan"`, `"bitrate"`, `"mode":"listen-only"`, 프레임마다 `"timestamp_us"`(epoch µs)가
붙는다. TUI 헤더는 `pcan1 PCAN-USB · … · listen-only 500k`, 로그는 하드웨어 µs 타임스탬프임을 알린다. 연결 화면은
약 1초마다 채널 상태를 다시 읽어 꽂고 뽑는 것을 반영하고, `last:` 줄도 PCAN 채널을 기억한다. 사용 중 USB를 뽑으면
대시보드가 DISCONNECTED가 된다.

**listen-only 보장과 구현.**
- 라이브러리 함수 표(`common/pcan.h`의 `pcan_api_t`)에는 수신용 함수만 있다: `CAN_Initialize`, `CAN_Uninitialize`,
  `CAN_GetStatus`, `CAN_Read`, `CAN_GetValue`, `CAN_SetValue`, `CAN_GetErrorText`. `CAN_Write`/`CAN_WriteFD`는 선언도
  `dlsym`도 하지 않고, `CAN_Reset`도 쓰지 않는다(테스트가 바이너리 문자열까지 확인).
- 채널을 여는 곳은 `pcan_open()` 하나다: 초기화 전 채널에 `PCAN_LISTEN_ONLY = ON` → 아직 초기화 전에 다시 읽어 ON인지
  확인(읽기 실패나 ON이 아니면 초기화하지 않고 실패) → `CAN_Initialize` → `PCAN_LISTEN_ONLY`를 또 읽어 ON이 아니면 즉시
  `CAN_Uninitialize`하고 실패. 설정이 실패해도 초기화하지 않는다.
  `CAN_SetValue`는 이 한 곳에서 `PCAN_LISTEN_ONLY = ON`으로만 불린다. bitrate auto도 시도마다 이 순서를 그대로 밟고,
  listen-only를 확인하지 못하면 다른 속도로 재시도하지 않고 멈춘다.
- 송신하는 명령은 PCAN을 거부한다(어댑터 객체를 만들거나 라이브러리를 읽기 전에, 종료 코드 1):
  `query`, `supported`, `raw`, `p992`, `detect --probe`, `probe --mode obd`, `probe --mode auto`(질의로 넘어갈 수 있으므로)
  → `PCAN is listen-only: <명령> would transmit`. PCAN에는 `probe --mode can`을 쓴다. TUI의 TX 모드(`x`)는 모니터를 멈추지
  않고 바로 `TX unavailable: PCAN is listen-only`로 거부되며, PCAN 연결에는 송신 콜백 자체가 없다.
- PCAN 명령(monitor, detect, discover, probe)은 Bluetooth 객체를 만들지 않는다.

**처음 쓸 때: 차에 꽂기 전에 벤치에서 확인.** PCAN-USB(can_tools로 listen-only)와 다른 CAN 노드 하나만 벤치 버스에
연결하고, 양 끝에 120 Ω 종단을 제대로 단 다음 그 노드가 프레임을 보내게 한다. 아무도 ACK하지 않으므로 송신 노드는 ACK
오류를 낸다(STN/ELM 어댑터라면 `CAN ERROR`). 만약 PCAN이 ACK한다면 송신이 성공해서, ELM/STN은 OBD 요청에 `CAN ERROR`
대신 `NO DATA`로 답한다. 이때 can_tools는 재전송되는 같은 프레임을 계속 받아야 한다. 송신 쪽이 ACK 오류를 내고
can_tools가 프레임을 받으면 listen-only가 실제로 동작하는 것이다.

**타임스탬프와 6열 TSV.** 컨트롤러 시각(µs)은 첫 수신 묶음의 `min(호스트 수신 시각 − 컨트롤러 시각)`을 기준점으로
벽시계에 옮기고, 기준점은 그 뒤로 고정한다(단조 변환이라 ID별 주기가 정확히 유지된다). 호스트 시계와의 차이(drift)는
채널을 닫을 때 로그에 남는다. 하드웨어 타임스탬프 프레임은 `*.frames.tsv`(TUI 녹화, discover phase 덤프)에 6번째 열
`timestamp_us`(epoch µs)가 붙고 헤더에도 그 이름이 붙는다. 1열 `timestamp_ms`는 `floor(timestamp_us / 1000)`이라 5열만 읽는
도구도 같은 캡처로 본다. 리더는 5열·6열을 모두 받고, 6열이 1열과 같은 ms에 있지 않으면 그 행을 거부한다. 기존 5열 캡처는
그대로 읽힌다. RTR·상태·에러 프레임은 전달하지 않고 개수만 세며, 수신 큐 overrun과 읽기 오류도 세어 로그에 남긴다.

**제한.** classic CAN만 받는다(`CAN_Initialize`/`CAN_Read`): CAN FD 프레임은 받지 않는다. PCAN-USB Pro FD는 CAN1만 된다.
macOS 전용. PCBUSB 문서상 사용 중 뽑은 장치는 다시 꽂아도 같은 프로세스에서는 응답하지 않을 수 있다: 그때는 can_tools를
다시 시작한다. 실제 장치로는 아직 검증하지 않았다(테스트는 가짜 라이브러리 `tests/fake/fake_pcan.c`로 돈다).

## probe (구 vsp_probe)

VSP의 `signals[].canonical.id`와 `signals[].acquisition`을 기준으로 CAN frame 또는 diagnostic query를 선택하고, 지정한 목표 Hz를 만족하는지 JSON으로 출력한다.

기본 검증 대상은 rpm/gear/steering/throttle/brake 의미군이다:

```bash
# 하드웨어 없이 선택 계획만 확인
./build/can_tools probe ../../profiles/porsche__911_gt3__992__can_obd.vsp --plan

# CAN acquisition만 5Hz 목표로 10초 검증
./build/can_tools probe ../../profiles/porsche__911_gt3__992__can_obd.vsp \
  --device vLinker --mode can --hz 5 --seconds 10

# OBD/UDS acquisition만 5Hz 목표로 검증
./build/can_tools probe ../../profiles/porsche__911_gt3__992__can_obd.vsp \
  --device vLinker --mode obd --hz 5 --seconds 10
```

인자는 예전 `vsp_probe`와 같다(`./build/vsp_probe ...` 별칭도 동작).

프로필 프레임에 주기(`can_frames[].cycle_ms`)가 있으면 계획과 결과에 `frame_cycle_ms`·`frame_hz`가 붙고, 결과에는 관측/프레임 비율 `frame_rate_ratio`가 붙는다. 목표 Hz가 프레임보다 빠르고 프레임은 빠짐없이 들어왔으면 상태는 `under_target` 대신 `frame_limited`이고, 관측이 프레임 주기의 절반에도 못 미치면 `frame_rate_low`(프레임 유실, 게이트웨이 필터, 또는 이 차에서 주기가 다름)를 표시한다. mux 신호는 비교하지 않는다.

진단 응답은 VSP 규칙대로 CAN 프레임 하나씩(ISO-TP PCI 포함, 재조립 없음) 해석한다. `decode.mux`가 있으면 mux로 프레임을 고르고(멀티프레임 응답의 CF 순번, 첫 프레임의 `62 DID` 등), 없으면 단일/첫 프레임 앞의 긍정응답 표식(`41 PID`, `62 DID`, `61 LID`)으로 고른다. 수동 CAN도 mux와 `can_frames[].frame_id_mask`를 적용한다. DBC에서 변환한 `{"value": n}`만 있는 mux는 같은 프레임에서 `{"role": "multiplexor"}`인 시그널의 필드로 비교하고, 그런 시그널이 없으면 그 시그널은 디코드하지 않는다. 받은 프레임이 시그널이 읽는 바이트(필드, `sign_bit`, mux 필드)보다 짧으면 값을 갱신하지 않고 샘플로도 세지 않는다(없는 비트를 0으로 읽지 않음; CAN은 `short_frames`로 센다). 29-bit CAN ID는 아직 지원하지 않는다.

`--mode auto`는 CAN phase와 OBD phase를 순차 실행한다. 단, auto는 버스에 상시 송신 프레임이 들리면(PT-CAN 직결 가능성) OBD phase를 건너뛰고 아무것도 송신하지 않는다. OBD 포트에서 질의하려면 `--mode obd`를 명시한다. 차량 버스에 직결한 상태에서는 `--mode can`만 쓴다. `--signals`에는 `rpm,gear,steering,throttle,brake` 같은 의미군, canonical id, 또는 VSP signal name을 지정할 수 있다. STN/OBDLink 계열은 raw periodic scheduler를 사용한다. vLinker 계열은 제조사 확인상 `ATMA`/`STM`/`STMA` monitor mode에서 버스를 silent로 유지하며 periodic message(PM)도 송신하지 않으므로, auto에서는 poll 방식으로 fallback한다.

## scan / detect / monitor / query / supported / raw / p992 (구 can_reader)

D-CAN(OBD 포트) 액티브 쿼리와 PT-CAN 패시브 모니터링을 모두 지원한다. 인자는 예전 `can_reader`와 같고,
`./build/can_reader <명령> ...` 별칭도 그대로 동작한다.

### CLI 모드

```bash
# BLE 스캔
./build/can_tools scan

# 버스 종류 감지 (수신만 함; 상시 송신 프레임 유무)
./build/can_tools detect OBDLink
# OBD 포트에서만: 01 00 질의까지 보내 D-CAN 여부 확인 (직결 버스에서는 쓰지 말 것)
./build/can_tools detect OBDLink --probe

# 지원 PID 목록 (D-CAN)
./build/can_tools supported OBDLink

# PID 값 조회 (D-CAN)
./build/can_tools query OBDLink 0C 0D 11

# Porsche 992 legacy/debug profile commands
./build/can_tools p992 list

# Porsche 992 채널 1회 조회
./build/can_tools p992 read OBDLink RPM THROTTLE_POS STEER_ANGLE BRAKE_PRESS

# Porsche 992 채널 스트리밍 (NDJSON, 500ms 간격)
./build/can_tools p992 watch OBDLink --interval-ms 500 RPM STEER_ANGLE BRAKE_PRESS

# Porsche 992 라우트/원문 응답 진단
./build/can_tools p992 debug OBDLink
./build/can_tools p992 bin-probe OBDLink

# Porsche 992 DID 대상 ECU 스캔
./build/can_tools p992 scan-did OBDLink 5101
./build/can_tools p992 scan-did OBDLink 5101 --session 03

# Porsche 992 저수준 UDS DID 직접 읽기
./build/can_tools p992 uds-read OBDLink F190 --tx-id 7E4 --rx-id 7EC
./build/can_tools p992 uds-raw OBDLink 3E00 --tx-id 7E4 --rx-id 7EC

# CAN 패시브 모니터 (PT-CAN, 10초)
./build/can_tools monitor vLinker 10

# CAN 모니터 + ID 필터
./build/can_tools monitor vLinker 10 0A5 1A0 316

# PT-CAN 시그널 자동 탐색
./build/can_tools discover vLinker

# 원시 AT 명령
./build/can_tools raw OBDLink "ATRV"
```

- JSON 출력은 stdout, 로그는 stderr. `2>/dev/null`로 JSON만 파싱 가능.
- `<device>`: BLE 이름 prefix 매칭 (대소문자 무시)
- `p992 watch`는 stdout에 snapshot별 JSON object를 한 줄씩 출력한다(NDJSON).
- `p992`는 992 GT3 실차 검증 과정에서 만든 legacy/debug command set이다.

### 대화형 모드

예전 대화형 메뉴는 `menu`로 연다 (`./build/can_reader`를 인자 없이 실행해도 같다).
`can_tools`를 인자 없이 실행하면 이제 TUI가 열린다(아래 `tui` 절).

```bash
./build/can_tools menu
```

## tui (라이브 대시보드)

btop처럼 터미널 하나에서 어댑터 연결, 프로필 선택, 실시간 시그널, Raw CAN, 로그, discover를 다룬다.

```bash
./build/can_tools                         # 연결 화면 (= can_tools tui)
./build/can_tools tui --device OBDLink    # 이름에 OBDLink가 들어간 첫 어댑터에 바로 연결
./build/can_tools tui --device vLinker --profile ../../profiles/porsche__911_gt3__992__can_obd.vsp
./build/can_tools tui --replay captures/20261028_101500          # 저장된 세션을 차 없이 다시 보기
./build/can_tools tui --replay x.frames.tsv --snapshot-at 5 --size 120x40   # 화면을 텍스트로 출력 (터미널 불필요)
```

옵션: `--profile VSP|none`(없으면 연결 뒤 프로필 선택 창; replay는 아래 "세션 폴더"처럼 session.json의 프로필부터), `--profiles DIR`(프로필 폴더; 기본은 `VSP_PROFILES_DIR`,
`./profiles`, 저장소의 `profiles/`(빌드 때 절대 경로로 기록), 실행파일 폴더와 상위 폴더의 `profiles/` 순), `--no-tx`, `--no-rec`, `--captures DIR`(세션 폴더
위치, 기본 `captures/`), `--mono`(색 없음, `NO_COLOR`도 같음), `--ascii`(UTF-8이 아닌 터미널), replay 전용 `--speed X`(0.25~16).
`--device ""`는 거부한다.

**연결 화면**: 스캔으로 찾은 어댑터(BLE는 RSSI, Classic은 paired)가 계속 갱신되고 8초마다 다시 스캔한다. ↑↓로 고르고
⏎로 연결한다. 어댑터에는 ⏎를 누르거나 `--device`를 줄 때만 연결한다. `R` 저장된 캡처 목록에서 골라 replay로 열기
(목록이 열려 있는 동안은 연결하지 않고, 기다리던 어댑터는 `esc`로 목록을 닫은 뒤 연결),
`s` 목록 비우고 다시 스캔, `o` 캡처 경로를 직접 입력해 replay로 열기, `q` 종료(녹화 `r`은 연결한 뒤 대시보드에서). 마지막으로 쓴 어댑터와 프로필이 `last:` 줄에 나오고(처음부터 선택돼 있음) ⏎ 두 번이면
대시보드까지 간다: 그 이름의 어댑터가 스캔에 보이면 연결하고, 프로필 선택 창이 마지막 프로필에 맞춰 열린다.
연결하면 PASSIVE 초기화(`ATZ` … `ATCSM1` … `ATH1`, 필터 없음) 뒤 `STMA`(STN) 또는 `ATMA`로 모니터를 시작한다.

**대시보드**: 위는 Signals(값, 단위, Hz, age, 최근 10초 스파크라인 또는 좁으면 min/max, CAN ID), 아래는 Raw CAN(ID, Hz, dlc,
data; 1초 안에 바뀐 바이트는 밝게)과 Log. 80x24보다 작으면 탭으로 한 패널씩 보인다. 프로필이 없으면(또는 보여줄 시그널이
없으면) Gauges·Signals 자리에 `p`로 차량 프로필을 고르라는 4줄 안내 상자가 나오고, 탭 화면에서는 탭 줄 오른쪽에 같은 안내가 붙는다. 헤더는 어댑터, 버스 판정
(`detecting…` → `PT-CAN DIRECT` / `GATEWAY` / `SILENT (no frames)`), `silent` 또는 빨간 `TX ON`, 프로필, fr/s와 경과 시간,
`● REC`를 보여준다. 신호가 주기의 3배(최소 300 ms) 넘게 안 오면 회색에 Hz `0`, 10초가 넘으면 값이 `—`가 된다.
최근 10초 그래프도 같은 기준이다: 샘플 간격이 그보다 짧으면 이어 그리고(왼쪽 끝은 창 이전의 마지막 샘플로), 길면 넘는 구간을 비운다.
그래프는 창 안의 값 차이가 raw 2스텝(LSB) 이하면 평평한 선으로 그린다(6비트 이상 선형 정수 필드만; float·polynomial·5비트 이하는 그대로). 게이지의 스로틀·브레이크·RPM·속도는 0 미만을 0으로 보인다.
버스 판정은 모니터 시작 후 3초, 이후 5초마다 최근 3초로 discover와 같은 기준(20 Hz 이상 ID 2개 이상이면 DIRECT)을 쓴다.

| 키 | 동작 |
|----|------|
| `tab` | 다음 패널 (좁은 화면: 탭 전환) |
| `↑` `↓` `PgUp` `PgDn` | 포커스 패널 스크롤 (잘린 패널 아래 테두리에 `▼ n more`, `▲ n`) |
| `/` | 필터: 시그널 이름, Raw CAN ID 앞자리 (`esc` 해제) |
| `s` | Raw CAN 정렬: hz → id → 최근 변경 |
| `d` | discover 오버레이 |
| `p` | 프로필 선택 (입력으로 필터, ⏎ 불러오기, `Ctrl+N` 프로필 없이, `esc` 그대로) |
| `r` | 녹화 켜기/끄기 (라이브; replay에서는 로그 한 줄만) |
| `R` | replay 목록 (라이브, replay, 연결 화면) |
| `x` | TX 모드 켜기/끄기 (아래 안전 규칙) |
| `space` `[` `]` | replay 일시정지, 느리게, 빠르게 |
| `v` | 마지막 discover 결과 |
| `?` | 도움말 |
| `q` | 종료 (discover 진행 중이거나 TX 모드면 확인), `Ctrl+C`는 바로 종료 |
| `c` | 연결이 끊긴 뒤 연결 화면으로 |

**discover**: `d`가 대시보드 위에 탐색 창을 연다(뒤의 대시보드는 계속 갱신). ⏎로 다음 단계(Baseline, Steering, Throttle,
Brake, Gear, Wheel Speed)를 시작하고, `s` 건너뛰기(Baseline 제외), `a` 중단(캡처한 단계는 남고 manifest는 `interrupted`).
모니터는 단계 사이에도 멈추지 않는다. `live` 줄은 그 단계에 해당하는 프로필 시그널 값과, Baseline 동안 고정이었는데 지금 가장
많이 변하는 (ID, 바이트)와 스파크라인을 보여준다. 마지막 단계 뒤 결과 화면에서 ⏎를 누르면 초안 VSP를 바로 대시보드에
불러온다(프로필 없는 차 → discover → 값 확인). `v`는 result.json을 보여준다. Baseline이 DIRECT면 TX가 잠긴다.

**세션 폴더** (연결할 때 만든다; replay의 discover는 `captures/YYYYMMDD_HHMMSS_replay/`):

```
captures/YYYYMMDD_HHMMSS/
├── live.frames.tsv          # 녹화 (r로 다시 켜면 live_02.frames.tsv, ...); 형식은 discover dump와 같음
├── tui.log                  # TUI가 화면을 쓰는 동안의 stderr (Log 패널에도 표시)
├── session.json             # 어댑터, 전송 방식, STN, 모니터 명령, 프로필, 녹화 파일, 버스 판정 이력
├── 00_baseline.frames.tsv … # discover를 돌렸다면 단계별 dump
├── manifest.json, result.json, ptcan_discover_draft.dbc, ptcan_discover_draft.vsp
```

세션 폴더는 그대로 `can_tools tui --replay <폴더>`(프레임이 있는 live*.frames.tsv, 없으면 프레임이 있는 NN_*.frames.tsv를
이어 붙이고 1초 넘는 틈은 1초로 줄임)나 `can_tools replay <폴더>`로 다시 볼 수 있다. TUI의 replay 목록(연결 화면,
라이브 대시보드, replay 어디서나 `R`; `r`은 녹화)은 `--captures` 폴더 아래에서 프레임이 하나 이상 있는 세션 폴더와
`*.frames.tsv` 파일을 최신순으로 최대 512개 보여주고(헤더만 있는 녹화는 빠짐), ⏎로 고른 캡처를 바로 replay로 연다. 고른
캡처가 그사이 지워졌거나 비었으면 목록을 그대로 두고 이유를 보여주며, TX 모드 중에는 TX를 끄고 넘어갈지 먼저 묻는다. 마지막 어댑터·프로필은
`~/.can_tools/tui_last`에 저장한다. session.json과 tui_last의 프로필 경로는 절대 경로로 쓴다.

**replay의 프로필**: `--profile`을 주면 그 프로필(`none`이면 프로필 없이)을 쓴다. 주지 않았거나 목록(`R`)에서 고른 캡처는
캡처 폴더(파일이면 그 파일이 있는 폴더)의 session.json에 적힌 프로필을 불러온다. 그 경로에 파일이 없으면(저장소를 옮겼거나
다른 폴더에서 실행) 프로필 폴더에서 같은 파일 이름을 찾고, Log에 `session profile moved: <옛 경로> -> <새 경로>`를 남긴다.
session.json이 없거나 프로필이 null이거나 파일을 찾지 못하면 프로필 선택 창을 연다(tui_last의 마지막 프로필에 맞춰).
`esc`로 닫으면 프로필 없이 Raw CAN과 Log만 보이고, 언제든 `p`로 고를 수 있다.

**안전 규칙 (TX 모드)**: TUI는 기본적으로 아무것도 버스에 보내지 않는다(ATCSM1 silent mode, ACK도 없음). `x`로 TX 모드를
켜야만 프로필의 진단 시그널(OBD/UDS)을 질의하고, 다음을 모두 만족할 때만 켜진다: 라이브 연결(replay 아님), 버스 판정이
GATEWAY 또는 SILENT, 이 연결에서 한 번도 DIRECT로 판정된 적 없음(DIRECT는 연결이 끊길 때까지 잠김, discover Baseline 포함),
`--no-tx` 없음, 프로필에 진단 시그널이 1개 이상, 확인 창에서 `y`. DIRECT에서는 `x`가 로그 한 줄(`TX locked: DIRECT bus`)만 남긴다.
켜지면 모니터를 멈추고 poll 설정(`ATCSM0` 등, `vsp_probe` poll 모드와 같은 코드) 뒤 고유 질의를 돌아가며 초당 10개 이하로
보낸다. 끌 때(`x`, 종료, 끊김 정리)는 PASSIVE 초기화를 다시 실행해(ATZ로 헤더·주기 메시지 해제, ATCSM1 복구) 모니터를 다시
시작하거나 연결을 끊는다. 이 재초기화가 실패하면(ATCSM1이 `OK`가 아님) 어댑터가 아직 ACK를 보낼 수 있으므로 모니터를 다시
켜지 않고, 헤더가 빨간 `NOT SILENT`가 되며 Log에 `[ERROR] TX off: silent mode (ATCSM1) not confirmed …`가 남는다. `c`로 연결
화면에 돌아가 다시 연결하면 PASSIVE 초기화를 처음부터 다시 한다. **PT-CAN 직결 탭에서는 TX 모드를 켜지 말 것** — OBD 포트에서만 쓴다.

## discover (구 can_discover)

PT-CAN 시그널 자동 탐색. 인자는 예전 `can_discover`와 같다(`./build/can_discover ...` 별칭도 동작).

```bash
# 특정 어댑터 지정
./build/can_tools discover vLinker

# 첫 번째 어댑터에 자동 연결
./build/can_tools discover
```

### 동작 흐름

대화형 6단계 캡처로 모터스포츠 핵심 시그널을 자동 식별한다:

```
Phase 1: Baseline     (5초)  — 기본 트래픽 캡처
Phase 2: Steering     (8초)  — 스티어링 풀 좌/우 반복
Phase 3: Throttle     (8초)  — 액셀 끝까지 밟기/떼기 반복
Phase 4: Brake        (8초)  — 브레이크 끝까지 밟기/떼기 반복
Phase 5: Gear        (10초)  — P → R → N → D 시프트 [스킵 가능]
Phase 6: Wheel Speed (15초)  — 저속 주행 30+ km/h [스킵 가능]
```

각 Phase 후 사람이 CAN 후보를 고르지 않는다. 캡처가 끝나면 툴이 자동으로 추정한다:

1. phase마다 CAN ID별 raw frame을 baseline과 비교하고, counter/checksum/상수 바이트를 걸러낸다.
2. 신호별로 해당 phase에서 많이 변한 ID를 고르고, 변하는 바이트 묶음(region) 안에서 실제 bitfield
   (시작 비트, 길이, byte order, 부호)를 탐색한다. steering은 unsigned·2의 보수 외에 sign-magnitude
   (크기 + 별도 부호 비트, 예: Porsche/Audi 0x0A0 `32|12` + 부호 bit 44)도 시도한다.
3. 후보 필드마다 점수를 매긴다.
   - **evidence**: 해당 phase에서 그 필드가 baseline 대비 얼마나 변했는지
   - **specificity**: 그 신호가 움직이면 안 되는 phase에서 조용한지
     (예: 정차 phase의 wheel speed). 원래 같이 움직이는 phase(주행 중 RPM, 기어 변경 중 브레이크)는 감점하지 않는다.
   - **shape**: 비트 폭·주기·부호·바이트 순서가 신호 종류에 맞는지. 부호를 허용하는 신호(steering, brake)는
     부호(최상위) 비트가 바뀌는 순간 값이 이어지는지도 본다. 부호 방식을 잘못 읽으면 0 부근에서 범위
     절반만큼 튄다(2의 보수를 sign-magnitude로 읽거나 그 반대). 페달을 뗀 상태와 정차 상태는 값이
     하나이므로, throttle과 wheel speed는 baseline의 alive counter를 신호로 보지 않는다: counter 비트를
     포함한 필드는 감점하고, 필드 바로 옆이 counter면 필드가 잘렸다고 보지 않으며, counter 바로 위 비트도
     필드 LSB 후보로 시도한다. counter는 ID마다 baseline 프레임에서 찾는다: 거의 매 프레임(95%) 뒤집히는
     비트에서 시작해, 그 위 비트를 붙여도 묶음 값이 매 프레임 앞으로(범위 절반 미만) 또는 일정한 폭으로
     움직이는 동안(최대 8비트)이다. checksum 비트와 신호 자체의 떨림(예: idle의 스로틀 밸브 각도)은
     해당하지 않는다. 예: 992 GT3 `0x121`은 byte0 checksum, bit 8–11 counter, bit 12부터 페달.
   - **prior**: VSP corpus에 **같은 ID와 같은 layout**의 신호가 있는지 (ID만 같은 경우는 거의 점수 없음)
4. 필드 단위 최소 변화량을 넘지 못한 후보(예: 50→51 한 번 바뀐 바이트)는 버린다.
5. 모든 신호를 함께 배정해 두 신호가 같은 비트를 쓰지 않게 한다. 어떤 신호의 1순위 필드가 다른 신호에
   더 잘 맞으면 그 신호는 다음 후보로 넘어간다 (예: RPM과 throttle이 같은 phase에서 움직이는 경우).

`confidence`는 증거의 강도와 2순위 후보와의 차이로 정한다. 단독 후보라도 증거가 강하면 HIGH가 될 수 있고,
비슷한 후보가 남아 있으면 MEDIUM/LOW로 낮아진다. JSON의 `evidence` 객체와 VSP의 `discovery` 블록에서
점수 구성을 확인할 수 있다.

완료 시 JSON 결과와 함께 작업 디렉터리에 draft DBC와 draft VSP를 생성한다.
DBC는 raw 값 기준의 호환 초안이고, VSP는 정보가 없는 차량의 profile 시작점이다.
두 파일은 같은 최종 필드 목록에서 만들어지므로 내용이 서로 같고, JSON에 보고된 layout도 그대로 들어간다.
prior는 관측한 bitfield와 layout·부호가 맞을 때만 이름/scale/unit/canonical id를 채운다.
(prior가 관측 필드를 포함하고 남는 비트가 캡처 내내 변하지 않으면 prior layout을 채택한다. 맞는 것이
없으면 마지막으로, LSB가 같고 관측 필드가 위쪽으로만 더 넓은 prior도 그 위쪽 비트가 캡처 내내 변하지 않으면
채택한다: 캡처로는 두 폭을 구분할 수 없으므로 탐색 폭·바이트 경계 가점보다 corpus를 따른다. 여럿이면
support가 큰 것, 같으면 더 넓은 것.)
scale도 관측값으로 검사한다: 관측 raw 범위가 prior의 min/max 안에 들어야 하고, 브레이크·페달을 뗀 상태,
정차 상태(baseline)가 물리값 범위의 아래쪽 1/4에 와야 한다. idle은 0보다 위에 있고 작은 블립 뒤 언더슈트도
있으므로 RPM은 아래쪽 절반이면 된다. 맞지 않는 prior는 쓰지 않는다.
DLC는 항상 관측값을 쓴다. wheel speed는 가능한 경우 바퀴별 4개 신호로 분해해 출력한다.
discover 실행 시에는 기본적으로 `captures/YYYYMMDD_HHMMSS/` 세션 폴더를 만들고,
각 phase raw dump(`*.frames.tsv`), `manifest.json`, `result.json`,
`ptcan_discover_draft.dbc`, `ptcan_discover_draft.vsp`를 함께 저장한다.
원하면 `--dump-dir <DIR>`로 저장 위치를 덮어쓸 수 있다.
캡처 버퍼가 넘쳐 버린 frame이 있으면 콘솔에 경고하고 manifest의 `capture_dropped_frames`에 기록하며,
Hz는 실제로 저장된 frame이 덮는 시간으로 계산한다.
그래도 LOW confidence 결과, checksum/counter, 나머지 bit-packed field는 사람이 검토해 보정해야 한다.

### 저장된 캡처 다시 분석 (replay, 구 discover_replay)

차량 없이 저장된 세션 폴더로 탐색을 다시 돌릴 수 있다. 엔진을 고친 뒤 실제 캡처로 검증할 때 쓴다:

```bash
./build/can_tools replay captures/20260408_153012            # 결과는 <세션>/replay/ 에 저장
./build/can_tools replay captures/20260408_153012 --out-dir /tmp/replay
```

phase 길이는 manifest의 `analyzed_duration_s` → `actual_duration_s` → frame timestamp 순으로 정한다.
캡처 버퍼가 고정 크기였던 예전 세션은 frame이 앞부분만 남아 있으므로, frame이 덮는 시간이 manifest 시간의
80% 미만이면 그 시간으로 Hz를 계산하고 경고한다.

### Prior table 생성

prior table(`can_discover_priors.h`)은 빌드할 때 `tools/gen_can_discover_priors.py`가 저장소의
`profiles/*.vsp`(`../../profiles`)에서 `build/generated/`에 생성한다. 헤더는 커밋하지 않으며, 프로필이나 생성기가
바뀌면 다음 빌드에서 다시 만든다. 런타임에는 프로필 폴더가 없어도 된다.
헤더 상단에 `profiles/`를 마지막으로 바꾼 commit, 파일 수, 입력 digest가 기록된다. DLC가 8을 넘거나 필드가 frame을
벗어나는 신호, multiplexed 신호는 제외한다. VSP `decode.sign_bit`(sign-magnitude) 신호는 부호 `'S'`와
부호 비트 위치로 들어가며, 같은 비트를 다른 부호 방식으로 읽은 후보도 부호 전환이 더 매끄러울 때만 이 prior를 채택한다.

### 회귀 테스트

합성 캡처로 탐색 엔진을 검증한다 (RPM/throttle 분리, 같은 메시지의 두 신호, 저속 gateway,
잡음만 있는 캡처, skip된 phase, 묶음 timestamp, counter 바로 옆 페달과 작은 RPM 블립(992 GT3 실차 캡처 패턴),
같은 바이트에서 counter 위에 놓인 Motorola 페달, idle에서 하위 비트가 떨리는 스로틀,
DBC/VSP 일치, cycle time, 512 KB 스레드 스택 등):

```bash
cd build && ctest --output-on-failure      # 또는 ./build/discover_regression
```

합성 데이터는 회귀를 막기 위한 것이고 실제 차량 정확도를 보장하지 않는다. 실제 캡처는 `discover_replay`로 확인한다.

### D-CAN / PT-CAN 자동 감지

Baseline 캡처의 CAN ID별 Hz로 버스 접속 모드를 자동 판별한다:

| 모드 | 판정 조건 | "빠른 메시지" 기준 | 경로 |
|------|----------|-------------------|------|
| **DIRECT** | 20 Hz 이상인 ID가 2개 이상 | ID Hz 75번째 백분위 × 0.8 (5~30 Hz) | 어댑터 → CAN 직결 |
| **GATEWAY** | 그 외 | 1.0 Hz | 어댑터 → OBD 포트 → 게이트웨이(ZGW 등) → PT-CAN |

게이트웨이 경유 시에는 일부 메시지만 저속으로 중계되므로 가장 빠른 ID도 느리다(0.1~5 Hz).
직결 버스에는 느린 차체 메시지가 많아도 20~100 Hz 파워트레인/섀시 메시지가 항상 있으므로, median이 아니라
빠른 ID가 있는지로 판정한다(예: 992 GT3 직결은 median 4 Hz지만 25~50 Hz ID가 6개 있다).
"빠른 메시지" 기준은 RPM·wheel speed 후보의 주기 가점에 쓰인다. 직결 버스는 그 버스의 빠른 쪽 주기에 맞춰
정하므로 일반적인 PT-CAN은 30 Hz, 엔진 메시지가 10 Hz인 버스(992 GT3)는 약 8 Hz가 된다.

```
[INFO] Bus mode: GATEWAY (D-CAN)    ← 가장 빠른 ID도 ~5 Hz
[INFO] Bus mode: DIRECT (PT-CAN)    ← 20 Hz 이상 ID가 2개 이상
```

JSON 출력에도 `"bus_mode"` 필드가 포함된다.

### 시그널 특성화

캡처 중 풀 레인지 입력으로 데이터 타입도 함께 분석:

- **Layout**: `start_bit` / `bit_length` / `byte_order`(0=big, 1=little) — DBC 표기 그대로
- **byte / byte2**: 필드가 걸친 가장 상위 바이트와 그다음 하위 바이트
- **Endianness**: Big/Little-endian (여러 바이트에 걸친 시그널)
- **Signedness**: Signed/Unsigned/Sign-magnitude. sign-magnitude는 `sign_char` `'S'`와 `sign_bit`로 보고하고,
  JSON에는 `"signed":false,"sign_bit":N`, draft VSP에는 `"sign_bit"`, draft DBC에는 크기 신호와 1비트
  `<Name>_Sign` 신호로 쓴다(vehicle-signal-profile의 `dbc2vsp`/`vsp2dbc`와 같은 규칙).
- **Raw Range**: 관측 최솟값/최댓값
- **Confidence**: HIGH/MEDIUM/LOW

### 출력 예시

```json
{
  "bus_mode":"direct",
  "dbc_path":"ptcan_discover_draft_20260408_153012.dbc",
  "vsp_path":"ptcan_discover_draft_20260408_153012.vsp",
  "steering":{"can_id":"0x0A5","dlc":8,"hz":100.0,"byte":3,"byte2":2,"start_bit":16,"bit_length":16,"byte_order":1,"score":104.0,"confidence":"HIGH","evidence":{"phase":60.0,"specificity":20.0,"shape":24.0,"prior":0.0,"prior_support":0,"ambiguity_margin":104.0},"endian":"little","signed":true,"raw_min":-4500,"raw_max":4500},
  "rpm":{"can_id":"0x0F3","dlc":8,"hz":50.0,"byte":4,"byte2":5,"start_bit":39,"bit_length":16,"byte_order":0,"score":117.0,"confidence":"HIGH","evidence":{"phase":50.0,"specificity":20.0,"shape":47.0,"prior":0.0,"prior_support":0,"ambiguity_margin":27.0},"endian":"big","signed":false,"raw_min":2988,"raw_max":23800},
  "throttle":{"can_id":"0x0D9","dlc":6,"hz":20.0,"byte":3,"byte2":2,"start_bit":16,"bit_length":12,"byte_order":1,"score":123.0,"confidence":"HIGH","evidence":{"phase":56.0,"specificity":20.0,"shape":31.0,"prior":16.0,"prior_support":7,"ambiguity_margin":44.0},"endian":"little","signed":false,"raw_min":0,"raw_max":4000}
}
```

## 프로젝트 구조

```
apps/can_tools/
├── can_tools/           # 단일 실행파일 can_tools (명령별 파일)
│   ├── main.m           # 명령 분기, 도움말, argv[0] 별칭(can_reader 등)
│   ├── cli_common.m/h   # 명령 공용 상태, 연결 래퍼
│   ├── cli_pcan.c/h     # PCAN 장치 열기(라이브러리, 채널, bitrate), 송신 명령 거부 메시지
│   ├── cmd_scan.m       # scan
│   ├── cmd_detect.m     # detect
│   ├── cmd_monitor.m    # monitor
│   ├── cmd_query.m      # query / supported / raw
│   ├── cmd_discover.m   # discover
│   ├── cmd_probe.m      # probe (구 vsp_probe)
│   ├── cmd_replay.c     # replay (구 discover_replay, 순수 C)
│   ├── cmd_p992.m       # p992 legacy/debug
│   ├── interactive.m    # 예전 대화형 메뉴 (can_tools menu)
│   ├── commands.h       # 명령 진입점
│   └── tui/             # 라이브 대시보드
│       ├── tui_main.m   # tui 옵션, replay 루프, snapshot 모드
│       ├── tui_live.m   # 연결 화면, PASSIVE 연결, 라이브 루프, TX host
│       ├── tui_app.c    # 앱 상태, 오버레이(discover, 결과, 프로필, 도움말, 확인), 키, 녹화, TX 가드
│       ├── tui_model.c  # Raw CAN 표, 시그널 통계·이력, 버스 판정, 로그, 숫자 형식 (순수 C)
│       ├── tui_render.c / tui_screens.c / tui_grid.c  # 셀 그리드에 그리기 (순수 C, ncurses 없음)
│       ├── tui_term.c   # ncurses 백엔드
│       ├── tui_profiles.c, tui_replay.c, tui_recorder.c, tui_capture.c
├── common/              # 공유 라이브러리
│   ├── elm327.c/h       # ELM327 프로토콜 핸들러
│   ├── stn2255.c/h      # STN2255 확장 (배치 쿼리, STMA 등)
│   ├── obd_adapter.m/h  # BLE/Classic BT OBD 어댑터 연결
│   ├── obd_connect.m/h  # 공용 스캔·연결·초기화 경로 (PASSIVE/ACTIVE 프로파일)
│   ├── obd_poll.m/h     # poll 방식 OBD 질의 (probe poll 모드, TUI TX 모드; 송신)
│   ├── frame_source.c/h # CAN frame 소스 인터페이스, replay 소스, 캡처 버퍼
│   ├── frame_source_live.m/h # 어댑터 monitor를 frame 소스로 감싼 live 소스
│   ├── pcan.c/h         # PCAN-USB(PCBUSB, dlopen): 수신 전용 함수 표, listen-only 열기, bitrate auto, 시각 변환
│   ├── frame_source_pcan.c/h # 열린 PCAN 채널의 frame 소스 (하드웨어 µs 타임스탬프)
│   ├── frame_tsv.c/h    # 캡처 dump(*.frames.tsv) 읽기·쓰기 (5열, 하드웨어 타임스탬프는 6열)
│   ├── discover_session.m/h # 탐색 세션(phase 캡처→분석→파일 저장) + 대화형 CLI
│   ├── vsp_engine.m/h   # VSP 프로파일 로드, 시그널 선택, frame 디코드 (C API)
│   ├── cli_util.c/h     # CLI 로그, JSON 문자열 출력
│   └── can_discover.c/h # PT-CAN 시그널 자동 탐색 엔진
├── tools/               # prior 생성기(빌드 때 실행, build/generated/can_discover_priors.h), ref2dbc
├── tests/               # 탐색 엔진 회귀, VSP 디코더, replay, 탐색 세션, 어댑터 수명, TUI 테스트 (하드웨어 불필요)
│   ├── fake/            # 가짜 어댑터, 가짜 PCBUSB (can_tools_fake)
│   ├── tui_fake/        # 가짜 어댑터로 TUI 라이브 경로의 AT 명령 기록 검증
│   ├── pcan_fake/       # 가짜 PCBUSB로 송신 명령 거부, listen-only 호출 순서, PCAN TUI 검증
│   ├── pcan_test.c      # PCAN 열기 순서, bitrate auto, 시각 변환, 6열 TSV (단위 테스트)
│   ├── golden/tui/      # TUI 화면 스냅샷 골든 (snapshots.sh update로 갱신)
│   └── fixtures/tui/    # 합성 스트림, 테스트 프로필, discover 세션 (gen_fixtures.py로 생성)
├── CMakeLists.txt
└── build.sh
```

## 주의사항

- macOS Bluetooth 권한 필요 (시스템 설정 > 개인정보 보호 및 보안)
- PT-CAN 시그널 탐색은 PT-CAN 직결 및 D-CAN(OBD 포트) 경유 모두 지원 — 버스 모드 자동 감지
- 어댑터는 패시브 모드에서 Silent Mode(`ATCSM1`)로 동작 — CAN ACK 미전송. `ATCSM1`에 `OK`로 답하지 않는 어댑터(일부 저가
  복제품)는 수신 전용 명령에 쓸 수 없다
- `ctest`(build 폴더)는 하드웨어 없이 돈다: TUI 테스트는 replay와 가짜 어댑터 빌드만 쓴다
- IGN ON 상태에서 실행해야 함
