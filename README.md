# YOLO Autonomous Drone

Gazebo Sim, ArduPilot SITL, Mission Planner, YOLOv5, MAVLink를 연결해 표적을 탐지하고 접근하는 드론 프로젝트입니다. 시뮬레이션에서 임무 흐름을 검증한 뒤 Jetson Nano와 Pixhawk 기반 실기체로 옮기는 것을 목표로 합니다.

마지막 로컬 검증일: **2026-07-23**

## 시스템 구조

```text
Mission Planner ─────── MAVLink ───────┐
                                       v
Gazebo Sim <── JSON/FDM ──> ArduPilot SITL ──> MAVProxy hub
    │                                           │
    │ downward camera                           │ TCP 5772
    v                                           v
YOLOv5 detector ──> target controller ──────────┘

RadioMaster joystick ──> RC bridge ── TCP 5773 ──> MAVProxy hub
```

- **Gazebo Sim**: 드론 동역학, 잔디 월드, 표적 바구니, 하향 카메라를 시뮬레이션합니다.
- **ArduPilot SITL**: 자세 안정화, 모터 출력, 비행 모드, waypoint 임무를 담당합니다.
- **Mission Planner**: 비행 상태 확인, 미션 작성, ARM 및 모드 변경에 사용합니다.
- **YOLOv5 제어기**: AUTO 비행 중 표적을 탐지하고 GUIDED 모드로 전환해 표적 중심으로 접근합니다.
- **조종기**: 비상 시 LOITER, ALT_HOLD 또는 STABILIZE로 전환해 자동제어를 중단합니다.

## 팀 문서의 역할

- [drone_log](https://github.com/woosun2006-cmyk/drone_log): 팀 전체 계획, Jetson/Pixhawk 실기체 작업, 비행 준비 기록의 원본
- 이 저장소: Gazebo/SITL/YOLO 구현, 재현 가능한 설치 절차와 시뮬레이션 시험의 원본

같은 내용을 두 저장소에 복사해 관리하지 않습니다. 공통 통신 규약은 MAVProxy를 단일 허브로 사용한다는 원칙을 따르고, 변경 시 두 문서가 서로 링크하는 인터페이스 요약만 함께 갱신합니다.

## 현재 구현된 흐름

```text
DISARMED
  -> Mission upload
  -> ARM
  -> AUTO waypoint flight (0.5 m/s)
  -> target detected for 5 consecutive frames
  -> GUIDED handoff
  -> highest-confidence target centering
  -> centered for 1 second
  -> descend at 0.2 m/s
  -> hold at 0.8 m relative altitude
```

타깃을 놓치거나 조종기 모드 스위치로 GUIDED를 벗어나면 자동 이동과 하강을 중단합니다. 현재 로직은 착륙하지 않고 표적 위 0.8m에서 호버합니다.

## 검증 환경

| 항목 | 현재 로컬 환경 |
|---|---|
| Host | Windows + WSL2 + WSLg |
| Linux | Ubuntu 24.04.4 LTS |
| Gazebo | Jetty, Gazebo Sim 10.4.0 |
| ArduPilot | 4.8.0-dev, commit `ceb710c` |
| ardupilot_gazebo | commit `082a0fe` 기반 로컬 수정 |
| Python | 3.12.3 |
| YOLO | YOLOv5 v7.0, commit `915bbf29` |
| PyTorch | 2.6.0+cu124 |
| GPU | NVIDIA RTX 4050 Laptop GPU through WSL CUDA |
| Weights | `best_v5.pt` |

Gazebo Jetty는 Ubuntu 24.04용 `gz-jetty` 패키지이며 Gazebo Sim 10을 포함합니다. 다른 Gazebo 세대를 사용하면 개발 라이브러리 이름과 플러그인 빌드 설정이 달라집니다.

## 처음 설치하기

새 WSL 컴퓨터에서는 저장소 clone만으로 Gazebo와 SITL이 설치되지 않습니다. 다음 문서의 순서대로 진행하십시오.

1. [Gazebo, SITL, Mission Planner, YOLO 전체 설치 및 실행 안내서](docs/GAZEBO_SITL_SETUP.md)
2. Gazebo와 ArduPilot 기본 예제를 먼저 검증합니다.
3. 프로젝트 월드와 제어 코드를 설치합니다.
4. Mission Planner에서 waypoint 임무를 업로드합니다.
5. AUTO 비행 중 YOLO의 GUIDED 인계를 확인합니다.

## 현재 저장소 이식 상태

이 저장소는 아직 프로젝트 자산 이관 중입니다. 현재 로컬에서 동작하는 다음 항목은 커밋 전 상태이므로 **현 시점의 fresh clone은 완전 실행형이 아닙니다**.

- `ensamb_iris_runway.sdf` 월드
- `ensamb_with_standoffs`, `ensamb_with_gimbal`, `target_basket` 모델과 mesh
- 수정된 `GstCameraPlugin`
- `gazebo_yolo.py`, `basket_handoff.py`, joystick bridge와 테스트
- 실행기 `gazebo-my-drone`, `sitl-my-drone`, `yolo-basket-gazebo`
- YOLO 가중치 `best_v5.pt`

위 파일은 라이선스와 배포 방식을 확인한 뒤 `sim/`, `src/`, `scripts/`, `weights/`로 옮겨야 합니다. 특히 학습 가중치는 일반 Git, Git LFS 또는 GitHub Release 중 하나를 명시적으로 선택해야 합니다.

## 시뮬레이션 정확도 주의

현재 모델은 실제 외형과 측정 질량을 반영했지만, 추력 모델은 실측 추력표가 없는 잠정값입니다.

- 총 시뮬레이션 질량: 약 `1.183kg`
- 모터: 2212 920KV, 4개
- ESC: 30A SimonK, 4개
- 배터리 가정: 3S LiPo
- 프로펠러: 직경 약 24cm, 피치 잠정 4.5inch
- 모터 최대 추력 가정: 약 550g/개
- 하향 카메라: 640x480, 10Hz, 수평 화각 약 157.6deg

따라서 SITL은 **미션 로직, 영상 처리, MAVLink 연결, 상태 전환 검증용**입니다. 실제 기체의 PID, 호버 스로틀, 배터리 거동과 최대 추력을 검증하는 장비로 사용하면 안 됩니다. 실제 비행 전에는 프로펠러 규격, 스로틀별 추력/RPM/전류, 전체 무게중심과 관성값을 측정해야 합니다.

## 주요 포트

현재 실행기를 사용하면 아래 포트는 자동 구성되므로 매번 직접 열 필요가 없습니다. 같은 WSL 안에서만 사용할 포트를 외부 인터페이스에 공개하지 마십시오.

| 포트 | 필요 여부 | 연결 | 용도 |
|---|---|---|---|
| UDP 9002 | 필수 | Gazebo <-> SITL | ArduPilot JSON/FDM 센서·모터 데이터 |
| TCP 5760 | 필수 | SITL -> MAVProxy | SITL 기본 MAVLink master; 앱 직접 연결 금지 |
| TCP 5772 | 필수 | YOLO -> MAVProxy | GUIDED controller 전용 hub output |
| TCP 5773 | 선택 | RC bridge -> MAVProxy | 조종기 bridge 전용 hub output |
| UDP 14550 | 필수 | MAVProxy -> Mission Planner | GCS telemetry와 명령 |
| UDP 5600 | 필수 | Gazebo -> YOLO | 현재 하향 카메라 H.264 stream |

`14550`은 Mission Planner가 Windows에서 실행될 때만 Windows host IP로 전달하면 됩니다. `5772/5773`은 Jetson 또는 WSL 내부의 `127.0.0.1`에만 bind합니다. SITL의 기본 `5762/5763` 포트는 프로젝트 애플리케이션에 사용하지 않습니다.

## 기준 좌표와 테스트 월드

- Home: `-35.363262, 149.165237`, elevation `584m`
- 드론 시작점: Gazebo `(0, 0)`
- 표적 바구니: Gazebo `(0, 4)`
- 10m 북쪽 waypoint 예시: `-35.363172, 149.165237`
- AUTO waypoint 속도: `WP_SPD=0.5m/s` (ArduPilot 4.8-dev)

ArduPilot 안정 버전에서는 같은 파라미터가 `WPNAV_SPEED`이고 단위가 cm/s일 수 있습니다. 반드시 Mission Planner의 실제 파라미터 목록에서 이름과 단위를 확인하십시오.

## Upstream 문서

- [Gazebo Jetty Ubuntu 설치](https://gazebosim.org/docs/jetty/install_ubuntu/)
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [ArduPilot SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [ArduPilot Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [YOLOv5](https://github.com/ultralytics/yolov5)
