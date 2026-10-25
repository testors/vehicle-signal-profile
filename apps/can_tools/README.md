# CAN Tools

BLE/Classic BT OBD 어댑터를 통한 CAN 버스 도구 모음. macOS에서 동작하며, ELM327/STN2255 호환 어댑터(OBDLink MX+, vLinker MC+ 등)를 지원한다.

## 도구

| 도구 | 설명 |
|------|------|
| **can_reader** | OBD-II PID 조회, CAN 모니터링, 버스 감지, 시그널 탐색 |
| **can_discover** | PT-CAN 시그널 자동 탐색 전용 (경량, 대화형) |
| **vsp_probe** | VSP 프로파일의 CAN/OBD 시그널을 목표 Hz로 읽을 수 있는지 검증 |

`can_reader`는 D-CAN/PT-CAN 양쪽을 지원하는 다목적 도구이고, `can_discover`는 PT-CAN 시그널 탐색에 특화된 단일 목적 도구이다. `vsp_probe`는 `../vehicle-signal-profile` 같은 VSP 프로파일을 런타임 검증하는 별도 실행파일이다. `can_reader discover` 명령과 `can_discover`는 동일한 탐색 로직(`can_discover.c`)을 공유한다.

## 빌드

```bash
./build.sh
```

또는:

```bash
mkdir -p build && cd build && cmake .. && make
```

빌드 후 `build/libcan_discover_core.a`가 함께 생성된다. 다른 앱에서 탐색 엔진만 쓰고 싶으면
`common/can_discover.h`를 포함해 `disc_analyze_with_raw()`,
`disc_render_draft_dbc()`, `disc_render_draft_vsp()`를 직접 호출하면 된다.

## vsp_probe

VSP의 `signals[].canonical.id`와 `signals[].acquisition`을 기준으로 CAN frame 또는 diagnostic query를 선택하고, 지정한 목표 Hz를 만족하는지 JSON으로 출력한다.

기본 검증 대상은 rpm/gear/steering/throttle/brake 의미군이다:

```bash
# 하드웨어 없이 선택 계획만 확인
./build/vsp_probe ../vehicle-signal-profile/profiles/porsche__911_gt3__992.vsp --plan

# CAN acquisition만 5Hz 목표로 10초 검증
./build/vsp_probe ../vehicle-signal-profile/profiles/porsche__911_gt3__992.vsp \
  --device vLinker --mode can --hz 5 --seconds 10

# OBD/UDS acquisition만 5Hz 목표로 검증
./build/vsp_probe ../vehicle-signal-profile/profiles/porsche__911_gt3__992.vsp \
  --device vLinker --mode obd --hz 5 --seconds 10
```

`--mode auto`는 CAN phase와 OBD phase를 순차 실행한다. `--signals`에는 `rpm,gear,steering,throttle,brake` 같은 의미군, canonical id, 또는 VSP signal name을 지정할 수 있다. STN/OBDLink 계열은 raw periodic scheduler를 사용한다. vLinker 계열은 제조사 확인상 `ATMA`/`STM`/`STMA` monitor mode에서 버스를 silent로 유지하며 periodic message(PM)도 송신하지 않으므로, auto에서는 poll 방식으로 fallback한다.

## can_reader

D-CAN(OBD 포트) 액티브 쿼리와 PT-CAN 패시브 모니터링을 모두 지원한다.

### CLI 모드

```bash
# BLE 스캔
./build/can_reader scan

# D-CAN vs PT-CAN 자동 감지
./build/can_reader detect OBDLink

# 지원 PID 목록 (D-CAN)
./build/can_reader supported OBDLink

# PID 값 조회 (D-CAN)
./build/can_reader query OBDLink 0C 0D 11

# Porsche 992 legacy/debug profile commands
./build/can_reader p992 list

# Porsche 992 채널 1회 조회
./build/can_reader p992 read OBDLink RPM THROTTLE_POS STEER_ANGLE BRAKE_PRESS

# Porsche 992 채널 스트리밍 (NDJSON, 500ms 간격)
./build/can_reader p992 watch OBDLink --interval-ms 500 RPM STEER_ANGLE BRAKE_PRESS

# Porsche 992 라우트/원문 응답 진단
./build/can_reader p992 debug OBDLink
./build/can_reader p992 bin-probe OBDLink

# Porsche 992 DID 대상 ECU 스캔
./build/can_reader p992 scan-did OBDLink 5101
./build/can_reader p992 scan-did OBDLink 5101 --session 03

# Porsche 992 저수준 UDS DID 직접 읽기
./build/can_reader p992 uds-read OBDLink F190 --tx-id 7E4 --rx-id 7EC
./build/can_reader p992 uds-raw OBDLink 3E00 --tx-id 7E4 --rx-id 7EC

# CAN 패시브 모니터 (PT-CAN, 10초)
./build/can_reader monitor vLinker 10

# CAN 모니터 + ID 필터
./build/can_reader monitor vLinker 10 0A5 1A0 316

# PT-CAN 시그널 자동 탐색
./build/can_reader discover vLinker

# 원시 AT 명령
./build/can_reader raw OBDLink "ATRV"
```

- JSON 출력은 stdout, 로그는 stderr. `2>/dev/null`로 JSON만 파싱 가능.
- `<device>`: BLE 이름 prefix 매칭 (대소문자 무시)
- `p992 watch`는 stdout에 snapshot별 JSON object를 한 줄씩 출력한다(NDJSON).
- `p992`는 992 GT3 실차 검증 과정에서 만든 legacy/debug command set이다.

### 대화형 모드

인자 없이 실행하면 대화형 메뉴 모드로 진입:

```bash
./build/can_reader
```

## can_discover

PT-CAN 시그널 자동 탐색 전용 도구.

```bash
# 특정 어댑터 지정
./build/can_discover vLinker

# 첫 번째 어댑터에 자동 연결
./build/can_discover
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
     절반만큼 튄다(2의 보수를 sign-magnitude로 읽거나 그 반대).
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
(prior가 관측 필드를 포함하고 남는 비트가 캡처 내내 변하지 않으면 prior layout을 채택한다.)
scale도 관측값으로 검사한다: 관측 raw 범위가 prior의 min/max 안에 들어야 하고, 브레이크·페달을 뗀 상태,
idle, 정차 상태(baseline)가 물리값 범위의 아래쪽에 와야 한다. 맞지 않는 prior는 쓰지 않는다.
DLC는 항상 관측값을 쓴다. wheel speed는 가능한 경우 바퀴별 4개 신호로 분해해 출력한다.
discover 실행 시에는 기본적으로 `captures/YYYYMMDD_HHMMSS/` 세션 폴더를 만들고,
각 phase raw dump(`*.frames.tsv`), `manifest.json`, `result.json`,
`ptcan_discover_draft.dbc`, `ptcan_discover_draft.vsp`를 함께 저장한다.
원하면 `--dump-dir <DIR>`로 저장 위치를 덮어쓸 수 있다.
캡처 버퍼가 넘쳐 버린 frame이 있으면 콘솔에 경고하고 manifest의 `capture_dropped_frames`에 기록하며,
Hz는 실제로 저장된 frame이 덮는 시간으로 계산한다.
그래도 LOW confidence 결과, checksum/counter, 나머지 bit-packed field는 사람이 검토해 보정해야 한다.

### 저장된 캡처 다시 분석 (discover_replay)

차량 없이 저장된 세션 폴더로 탐색을 다시 돌릴 수 있다. 엔진을 고친 뒤 실제 캡처로 검증할 때 쓴다:

```bash
./build/discover_replay captures/20260408_153012            # 결과는 <세션>/replay/ 에 저장
./build/discover_replay captures/20260408_153012 --out-dir /tmp/replay
```

phase 길이는 manifest의 `analyzed_duration_s` → `actual_duration_s` → frame timestamp 순으로 정한다.
캡처 버퍼가 고정 크기였던 예전 세션은 frame이 앞부분만 남아 있으므로, frame이 덮는 시간이 manifest 시간의
80% 미만이면 그 시간으로 Hz를 계산하고 경고한다.

### Prior table 생성

prior table(`common/can_discover_priors.h`)은 `tools/gen_can_discover_priors.py`로
`../vehicle-signal-profile/profiles/*.vsp`에서 생성하며, 런타임에는 VSP 저장소가 없어도 된다.
헤더 상단에 원본 저장소 commit, 파일 수, 입력 digest가 기록된다. DLC가 8을 넘거나 필드가 frame을
벗어나는 신호, multiplexed 신호는 제외한다. VSP `decode.sign_bit`(sign-magnitude) 신호는 부호 `'S'`와
부호 비트 위치로 들어가며, 같은 비트를 다른 부호 방식으로 읽은 후보도 부호 전환이 더 매끄러울 때만 이 prior를 채택한다.

```bash
python3 tools/gen_can_discover_priors.py            # 재생성
python3 tools/gen_can_discover_priors.py --check    # 현재 헤더가 원본과 일치하는지 확인
```

### 회귀 테스트

합성 캡처로 탐색 엔진을 검증한다 (RPM/throttle 분리, 같은 메시지의 두 신호, 저속 gateway,
잡음만 있는 캡처, skip된 phase, 묶음 timestamp, DBC/VSP 일치, cycle time, 512 KB 스레드 스택 등):

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
can_tools/
├── common/              # 공유 라이브러리
│   ├── elm327.c/h       # ELM327 프로토콜 핸들러
│   ├── stn2255.c/h      # STN2255 확장 (배치 쿼리, STMA 등)
│   ├── obd_adapter.m/h  # BLE/Classic BT OBD 어댑터 연결
│   ├── can_discover.c/h # PT-CAN 시그널 자동 탐색 엔진
│   └── can_discover_priors.h # VSP corpus prior table (생성 파일)
├── can_reader/          # 다목적 CAN 도구
│   └── main.m
├── can_discover/        # 시그널 탐색 전용 도구
│   └── main.m
├── vsp_probe/           # VSP acquisition Hz 검증 도구
│   └── main.m
├── tools/               # prior 생성기, discover_replay, ref2dbc
├── tests/               # 탐색 엔진 회귀 테스트 (합성 캡처)
├── CMakeLists.txt
└── build.sh
```

## 주의사항

- macOS Bluetooth 권한 필요 (시스템 설정 > 개인정보 보호 및 보안)
- PT-CAN 시그널 탐색은 PT-CAN 직결 및 D-CAN(OBD 포트) 경유 모두 지원 — 버스 모드 자동 감지
- 어댑터는 패시브 모드에서 Silent Mode(`ATCSM1`)로 동작 — CAN ACK 미전송
- IGN ON 상태에서 실행해야 함
