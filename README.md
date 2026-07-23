# YOLO Autonomous Drone

Gazebo Sim, ArduPilot, Mission Planner, YOLOv5를 연결해 waypoint 비행 중 바구니를 탐지하고 `GUIDED` 제어로 접근하는 드론 프로젝트입니다.

마지막 검증: **2026-07-23, Windows 11 + WSL2 Ubuntu 24.04**

## 역할과 범위

이 저장소가 관리하는 범위:

- Gazebo 월드, 근사 드론 형상, 표적 바구니와 하향 카메라
- ArduPilot SITL 파라미터와 MAVProxy 포트 구성
- YOLOv5 추론, `AUTO -> GUIDED` 인계, 표적 중심 접근
- RadioMaster USB joystick의 MAVLink RC override bridge
- 새 WSL 환경용 설치, 실행, 검증 스크립트

팀 전체 계획과 실기체 작업 기록은 [drone_log](https://github.com/woosun2006-cmyk/drone_log)를 원본으로 사용합니다. 두 저장소에 같은 계획을 복사하지 않습니다.

## 시스템 구조

```text
Gazebo <--- JSON/FDM ---> ArduPilot SITL ---> MAVProxy hub
   |                              |              | UDP 14550 -> Mission Planner
   | downward camera              |              | TCP 5772 -> YOLO controller
   v                              |              ` TCP 5773 -> joystick bridge
YOLOv5 ---------------------------'
```

MAVProxy는 메시지 허브이며 제어권 중재기는 아닙니다. YOLO는 `GUIDED`에서만 속도 setpoint를 보내고, 조종기가 다른 비행 모드로 전환하면 자동 명령을 중단합니다.

상세 설계: [ARCHITECTURE_OVERVIEW.md](ARCHITECTURE_OVERVIEW.md)

## 빠른 설치

새 WSL 컴퓨터에서:

```bash
cd ~
git clone https://github.com/jin04151/yolo-autonomous-drone.git
cd yolo-autonomous-drone
bash scripts/setup_gazebo_sitl.sh
source ~/.bashrc
```

설치기는 Gazebo Jetty, ArduPilot SITL, `ardupilot_gazebo`, YOLOv5와 Python 환경을 준비합니다. NVIDIA GPU가 보이면 CUDA 12.4용 PyTorch를 설치하고, 없으면 CPU 빌드를 설치합니다.

학습 모델은 Git에 포함하지 않습니다. 승인된 모델을 다음 위치에 둡니다.

```text
weights/best_v5.pt
```

모델 체크섬과 외부 경로 사용법: [weights/README.md](weights/README.md)

전체 설치 안내: [docs/GAZEBO_SITL_SETUP.md](docs/GAZEBO_SITL_SETUP.md)

## 실행

터미널 1, Gazebo와 YOLO:

```bash
gazebo-my-drone
```

터미널 2, ArduPilot SITL과 MAVProxy:

```bash
sitl-my-drone
```

Mission Planner는 UDP `14550`에 연결합니다. GUI의 YOLO 패널은 `/yolo/annotated` topic을 표시합니다.

자동 이륙만 시험하려면:

```bash
drone-takeoff 2
```

조종기 연결 후 RC bridge를 실행하려면:

```bash
joystick-bridge
```

## 자동 임무 흐름

```text
DISARMED
  -> Mission Planner에서 mission write
  -> ARM + AUTO
  -> waypoint flight at 0.5 m/s
  -> 유효한 target을 5 frame 연속 탐지
  -> GUIDED 요청
  -> 가장 confidence가 높은 유효 target 중심으로 이동
  -> 중앙에서 1초 유지
  -> 0.2 m/s로 하강
  -> 상대고도 0.8m에서 정지
```

현재 코드는 바구니 위에서 호버하며 자동 착륙하지 않습니다. 표적을 잃으면 수평·수직 속도 `0`을 보내고, `GUIDED`를 벗어나면 `PILOT_OVERRIDE` 상태로 들어갑니다.

화면 가장자리에 닿거나 전체 면적의 20%를 넘는 detection은 제어 대상에서 제외합니다. 이는 단색 바닥이나 화면 경계를 `white_box`로 잘못 인식했을 때 자동 인계되는 것을 막는 안전 필터입니다.

## 프로젝트 구조

```text
config/                  SITL 파라미터와 upstream revision
docs/                    설치 및 운용 문서
scripts/                 설치기, 검증기, 실행 명령
sim/gazebo/models/       드론과 바구니 모델
sim/gazebo/worlds/       짙은 녹색 시험 월드
src/mavlink/             MAVLink 이륙 도구
src/vision/              YOLO, 자동 인계, joystick bridge
tests/                   자동 인계 단위 테스트
weights/                 로컬 YOLO checkpoint 위치
```

## 주요 포트

| 포트 | 연결 | 용도 |
|---|---|---|
| UDP 9002 | Gazebo <-> SITL | JSON/FDM 센서와 모터 데이터 |
| TCP 5760 | SITL -> MAVProxy | SITL 기본 MAVLink master |
| TCP 5772 | YOLO -> MAVProxy | 자동제어 전용 연결 |
| TCP 5773 | joystick -> MAVProxy | RC bridge 전용 연결 |
| UDP 14550 | MAVProxy -> Mission Planner | GCS telemetry와 명령 |
| UDP 5600 | Gazebo -> YOLO | H.264 하향 카메라 영상 |

`5772/5773`은 WSL의 `127.0.0.1`에만 열립니다. Windows Mission Planner를 사용할 때만 Windows host IP로 `14550/UDP` 출력을 추가합니다.

## 시뮬레이션 한계

- 질량 약 `1.183kg`, 2212 920KV 모터 4개, 30A SimonK ESC, 3S LiPo 가정
- 프로펠러 직경 약 24cm, 피치 4.5inch와 최대 추력 550g/모터는 잠정값
- 하향 카메라 640x480, 10Hz, 수평 화각 약 157.6deg
- 공개 기본 외형과 충돌 형상, 관성은 모두 근사값

따라서 SITL은 영상 처리, MAVLink, 임무 상태 전환 검증용입니다. 실제 PID, 호버 스로틀, 배터리, 추력 검증을 대신하지 않습니다. 시뮬레이션 파라미터를 Pixhawk에 그대로 쓰면 안 됩니다.

## 검증

```bash
bash scripts/validate_setup.sh
```

이 명령은 필수 실행 파일, plugin, SDF, Python import와 자동 인계 단위 테스트를 검사합니다. 실행형 smoke test는 설치 안내서의 절차를 따릅니다.

## Upstream

- [Third-party notices](THIRD_PARTY_NOTICES.md)
- [Gazebo Jetty](https://gazebosim.org/docs/jetty/install_ubuntu/)
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
- [YOLOv5](https://github.com/ultralytics/yolov5)
