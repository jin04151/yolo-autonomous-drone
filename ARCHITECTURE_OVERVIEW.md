# Architecture Overview

## 1. 목적

이 프로젝트는 `AUTO` waypoint 비행과 영상 기반 표적 접근을 하나의 ArduPilot 비행 상태 안에서 검증합니다. Python이 개별 모터 PWM을 계산하지 않습니다. Python은 MAVLink 속도 setpoint를 보내고, 자세 안정화와 motor mixing은 ArduPilot이 담당합니다.

## 2. 구성요소

| 구성요소 | 책임 |
|---|---|
| Gazebo Sim | 물리, 충돌, IMU/GPS, 모터 반력, 하향 카메라 |
| ArduPilot SITL | EKF, 비행 모드, waypoint, 자세·고도 제어, motor mixing |
| MAVProxy | 하나의 vehicle link를 여러 client에 전달 |
| Mission Planner | mission 작성, telemetry, ARM, mode 전환 |
| YOLO controller | 영상 추론, `AUTO -> GUIDED`, body-frame velocity 전송 |
| joystick bridge | Linux joystick axis를 RC1~RC4 override로 변환 |

## 3. 통신 경계

```text
Gazebo UDP 9002 <--JSON--> SITL TCP 5760 <--MAVLink--> MAVProxy
                                                       |-- UDP 14550 GCS
                                                       |-- TCP 5772 YOLO
                                                       `-- TCP 5773 joystick
```

SITL의 `5762/5763`에 애플리케이션을 직접 붙이지 않습니다. 모든 client는 MAVProxy hub output을 사용합니다. 실제 Pixhawk에서는 MAVProxy의 master만 serial device로 바뀌며 client 계약은 유지할 수 있습니다.

## 4. 제어권 규칙

1. Mission Planner가 mission을 업로드하고 `AUTO`를 시작합니다.
2. YOLO는 vehicle이 armed이고 `AUTO`일 때만 연속 탐지를 셉니다.
3. 확인 조건을 만족하면 YOLO가 `GUIDED`를 요청합니다.
4. `GUIDED` 확인 후 `MAV_FRAME_BODY_NED` 속도를 주기적으로 전송합니다.
5. target loss 시 `(0, 0, 0)`을 보내며 위치를 유지합니다.
6. 조종기가 `LOITER`, `ALT_HOLD`, `STABILIZE` 등으로 바꾸면 YOLO는 명령을 중단합니다.

MAVProxy 자체에는 누가 우선인지 판단하는 기능이 없습니다. mode가 제어권 계약입니다.

## 5. 좌표계

- Gazebo world: ENU 기반 표현
- ArduPilot local command: NED
- YOLO command: `MAV_FRAME_BODY_NED`
- image right -> body right `+vy`
- image up -> body forward `+vx`
- NED 하강 -> `+vz`

카메라 장착 방향이나 image rotation이 바뀌면 이 매핑을 다시 검증해야 합니다.

## 6. 상태 흐름

```text
WAIT_AUTO -> CONFIRMING -> REQUESTING_GUIDED -> GUIDED_TRACKING
     ^                                                |
     `---------------- target/mode reset -------------'

GUIDED_TRACKING -> PILOT_OVERRIDE  (mode leaves GUIDED)
```

접근 중에는 target 중심 오차로 XY 속도를 만들고, deadband 안에서 1초 유지한 뒤 하강합니다. 상대고도 0.8m 이하에서는 하강을 멈춥니다.

## 7. SITL과 실기체

```text
SITL:    controller -> MAVProxy -> ArduPilot SITL -> Gazebo
Vehicle: controller -> MAVProxy -> serial -> Pixhawk -> ESC/motors
```

공유 가능한 부분은 detection, mission state, MAVLink message입니다. 공유하면 안 되는 부분은 PID, motor/propeller model, battery calibration, failsafe와 sensor calibration입니다.

실기체 전환 전 필수 조건:

- 프로펠러 제거 상태에서 motor order와 direction 확인
- RC, compass, accelerometer, battery monitor 교정
- geofence, RC/GCS/battery failsafe 설정
- manual override mode switch 실측
- 낮은 고도 tether 또는 안전구역 단계 시험

## 8. 저장소 경계

- 이 저장소: 실행 가능한 시뮬레이션과 vision/MAVLink 구현
- [drone_log](https://github.com/woosun2006-cmyk/drone_log): 팀 계획, 하드웨어 결정, 현장 로그

하드웨어 결정은 `drone_log`에서 기록하고, 그 결정 때문에 바뀐 software interface만 이 문서에 반영합니다.
