# Gazebo, ArduPilot SITL, Mission Planner, YOLO Setup

이 문서는 Windows의 WSL2 Ubuntu 24.04 환경에서 프로젝트 시뮬레이션 스택을 재구성하기 위한 안내서입니다. 명령은 특별한 표시가 없으면 **Ubuntu WSL 터미널**에서 실행합니다.

마지막 로컬 검증일: **2026-07-23**

## 1. 구성 요소의 역할

| 구성 요소 | 역할 |
|---|---|
| Gazebo Sim | 물리 환경, 드론 모델, IMU/GPS/카메라 센서 생성 |
| ardupilot_gazebo | Gazebo 센서와 ArduPilot JSON/FDM 인터페이스 연결 |
| ArduPilot SITL | Pixhawk 대신 PC에서 실행되는 비행제어기 |
| MAVProxy | SITL MAVLink를 Mission Planner, YOLO, 조종기로 분배 |
| Mission Planner | 미션 작성, 상태 확인, 파라미터 및 모드 제어 |
| YOLOv5 controller | 카메라 탐지 결과를 GUIDED 속도 명령으로 변환 |

ROS/ROS 2는 이 구성에 필요하지 않습니다.

### 사용 포트

실행기가 대부분 자동으로 구성합니다. 모든 포트를 Windows 방화벽이나 공유기에서 외부에 개방하는 작업은 필요하지 않습니다.

| 포트 | 필요 여부 | 연결 | 용도 |
|---|---|---|---|
| UDP 9002 | 필수 | Gazebo <-> SITL | JSON/FDM 센서 입력과 actuator 출력 |
| TCP 5760 | 필수 | SITL -> MAVProxy | MAVProxy master 연결 |
| TCP 5763 | 필수 | YOLO -> SITL | GUIDED velocity setpoint 전송 |
| UDP 14550 | 필수 | MAVProxy -> Mission Planner | GCS 연결 |
| UDP 5600 | 필수 | Gazebo -> YOLO | H.264 하향 카메라 영상 |
| UDP 14551 | 선택 | MAVProxy -> RC bridge/보조 client | 조종기 bridge 사용 시 |
| TCP 5762 | 선택 | client -> SITL | MAVLink 시험과 디버깅 |

현재 WSL 내부 연결은 `127.0.0.1`을 사용합니다. Windows에서 Mission Planner를 실행하는 경우에만 MAVProxy의 `14550` output을 Windows host IP로 추가합니다. `tcpin:0.0.0.0`처럼 모든 인터페이스에 listen하는 설정은 외부 장치 연결이 필요할 때만 사용하십시오.

## 2. 사전 조건

### Windows

- Windows 11 권장
- WSL2와 WSLg 활성화
- Ubuntu 24.04 배포판
- NVIDIA GPU 사용 시 최신 Windows NVIDIA 드라이버
- USB 조종기를 WSL로 넘길 경우 `usbipd-win`

PowerShell에서 WSL 상태를 확인합니다.

```powershell
wsl --status
wsl --list --verbose
```

Ubuntu에서 GUI와 GPU를 확인합니다.

```bash
echo "$DISPLAY"
nvidia-smi          # NVIDIA GPU가 없으면 생략
```

## 3. 프로젝트 clone

```bash
cd ~
git clone https://github.com/jin04151/yolo-autonomous-drone.git
cd ~/yolo-autonomous-drone
```

현재 저장소는 이관 중이므로 README의 **현재 저장소 이식 상태**를 먼저 확인하십시오. 아래 설치는 upstream 프로그램을 준비하는 과정이며, 아직 커밋되지 않은 프로젝트 자산을 자동 생성하지 않습니다.

## 4. Gazebo Jetty 설치

현재 검증 환경은 Gazebo Jetty의 Gazebo Sim 10입니다. Ubuntu 24.04에서:

```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo apt-get update
sudo apt-get install -y gz-jetty libgz-sim10-dev rapidjson-dev \
  libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

설치를 확인합니다.

```bash
gz sim --versions
env -u WAYLAND_DISPLAY gz sim -v4 shapes.sdf
```

WSL에서 `libEGL`, ZINK 또는 `currentGLContext` 오류가 발생하면 이 프로젝트에서는 Wayland 대신 X11 경로로 실행합니다.

```bash
env -u WAYLAND_DISPLAY gz sim <world.sdf>
```

이 우회는 렌더링 경로 문제를 피하기 위한 것이며 Gazebo 설치 실패를 의미하지는 않습니다.

## 5. ArduPilot SITL 설치

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
git submodule update --init --recursive
```

새 터미널을 연 뒤 기본 SITL 빌드를 확인합니다.

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

`sim_vehicle.py`는 ArduPilot을 빌드하고 실행한 뒤 MAVProxy도 함께 시작합니다.

## 6. ardupilot_gazebo 빌드

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ~/ardupilot_gazebo

export GZ_VERSION=jetty
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build . -j"$(nproc)"
```

Gazebo가 플러그인, 모델, 월드를 찾도록 `~/.bashrc`에 추가합니다.

```bash
cat >> ~/.bashrc <<'EOF'
export ARDUPILOT_GAZEBO_HOME="$HOME/ardupilot_gazebo"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
EOF

source ~/.bashrc
```

### Upstream 기본 연결 시험

터미널 1:

```bash
env -u WAYLAND_DISPLAY gz sim -v4 -r iris_runway.sdf
```

터미널 2:

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f gazebo-iris --model JSON --console --map
```

MAVProxy에서:

```text
mode guided
arm throttle
takeoff 2
mode land
```

이 기본 시험이 실패하면 프로젝트 커스텀 모델을 추가하기 전에 Gazebo와 SITL 설치부터 해결해야 합니다.

## 7. 프로젝트 파일 배치 계약

최종적으로 저장소가 clone-only 실행형이 되려면 다음 구조를 가져야 합니다.

```text
yolo-autonomous-drone/
├── config/
│   └── my-drone.parm
├── sim/gazebo/
│   ├── models/
│   │   ├── ensamb_with_standoffs/
│   │   ├── ensamb_with_gimbal/
│   │   └── target_basket/
│   └── worlds/
│       └── ensamb_iris_runway.sdf
├── src/vision/
│   ├── gazebo_yolo.py
│   └── basket_handoff.py
├── scripts/
│   ├── gazebo-my-drone
│   ├── sitl-my-drone
│   └── yolo-basket-gazebo
└── weights/
    └── best_v5.pt
```

현재는 이 파일 일부가 로컬 `~/ardupilot_gazebo`, `~/yolov5`, `~/.local/bin`에만 있습니다. 이관이 끝나기 전에는 다른 PC에서 동일한 커스텀 월드를 실행할 수 없습니다.

## 8. 현재 로컬 실행 순서

현재 개발 PC에는 다음 명령이 `~/.local/bin` 실행기로 설치되어 있습니다.

### 터미널 1: Gazebo + YOLO

```bash
gazebo-my-drone
```

이 실행기는:

1. `ensamb_iris_runway.sdf`를 실행합니다.
2. 하향 카메라의 streaming topic이 생길 때까지 기다립니다.
3. `yolo-basket-gazebo`를 자동 실행합니다.
4. `/yolo/annotated` 영상을 Gazebo GUI 패널에 게시합니다.

### 터미널 2: ArduPilot SITL + MAVProxy

```bash
sitl-my-drone
```

실제 핵심 명령은 다음과 같습니다.

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --model JSON \
  --add-param-file="$HOME/ardupilot_gazebo/config/my-drone.parm" \
  --out=127.0.0.1:14550 \
  --out=127.0.0.1:14551 \
  --mavproxy-args=--daemon
```

현재 ArduPilot 4.8-dev 설정:

```text
WP_SPD 0.5
```

이는 AUTO waypoint 수평 속도 `0.5m/s`입니다. ArduPilot 버전에 따라 파라미터 이름과 단위가 다를 수 있습니다.

### 터미널 3 또는 GUI: Mission Planner

현재 개발 PC는 WSL 안에서 Mono로 Mission Planner를 실행합니다.

```bash
cd ~/MissionPlanner
env -u WAYLAND_DISPLAY mono MissionPlanner.exe
```

Mission Planner에서:

```text
Connection type: UDP
Port: 14550
```

WSL Mono 실행은 플러그인과 조이스틱 호환성이 제한될 수 있습니다. 새 PC에서는 Windows용 Mission Planner를 Windows에서 실행하고 MAVProxy가 Windows IP로 UDP를 전달하는 구성이 더 안정적입니다.

Windows host IP를 WSL에서 확인:

```bash
WINDOWS_GCS_IP="$(ip route | awk '/default/ {print $3; exit}')"
echo "$WINDOWS_GCS_IP"
```

MAVProxy output 예시:

```text
--out=<WINDOWS_GCS_IP>:14550
```

## 9. Mission Planner waypoint 시험

현재 월드 좌표:

| 대상 | 위치 |
|---|---|
| Home | `-35.363262, 149.165237`, MSL 584m |
| Basket | Gazebo `(0, 4, 0)`, home에서 북쪽 4m |
| Test waypoint | `-35.363172, 149.165237`, home에서 북쪽 약 10m |

Mission Planner Plan 화면에서:

1. Home이 유효한 위치인지 확인합니다.
2. `TAKEOFF`, 상대고도 2~3m를 추가합니다.
3. 위 10m waypoint를 추가합니다.
4. `Write`로 SITL에 업로드합니다.
5. Flight Data에서 ARM 후 `AUTO`로 전환합니다.

권장 경로:

```text
start (0m) -> basket (4m) -> waypoint (10m)
```

하향 카메라 화각이 매우 넓으므로 높은 고도에서는 출발 직후 바구니가 보일 수 있습니다. AUTO 이동 중 탐지 인계를 명확히 시험하려면 바구니를 더 멀리 배치하거나 탐지 ROI/최소 box 크기 조건을 추가해야 합니다.

## 10. YOLO 자동제어 상태 흐름

```text
WAIT_AUTO
  -> CONFIRMING
  -> REQUESTING_GUIDED
  -> GUIDED_TRACKING
  -> target centered for 1 second
  -> DESCEND
  -> HOLD at 0.8m
```

현재 기본 제어값:

| 설정 | 값 |
|---|---:|
| GUIDED 인계 확인 | 연속 5 frames |
| 중앙 deadband | 정규화 오차 0.15 |
| XY gain | 0.6 |
| 최대 수평 속도 | 0.4m/s |
| 중앙 유지 시간 | 1.0s |
| 하강 속도 | 0.2m/s |
| 최소 접근 고도 | 0.8m |
| target loss timeout | 1.0s |

선택 대상은 지정 class가 없으면 basket 이름이 포함된 class를 우선하고, 그렇지 않으면 신뢰도가 가장 높은 detection입니다. 현재 모델 class가 `white_box`이면 다른 흰 물체를 오탐할 수 있으므로 실제 비행 전에 class, ROI, box 크기, 연속 탐지 조건을 강화해야 합니다.

중요: Jetson/YOLO 코드는 모터 PWM을 직접 제어하지 않습니다. MAVLink `SET_POSITION_TARGET_LOCAL_NED` 속도 setpoint를 보내고, 실제 자세 안정화와 모터 믹싱은 ArduPilot이 담당합니다.

## 11. 조종기 연결과 수동 인계

RadioMaster Pocket USB Joystick을 WSL에 연결하는 예입니다.

관리자 PowerShell:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

Ubuntu:

```bash
sudo apt-get install -y usbutils joystick
lsusb
jstest /dev/input/js0
```

현재 개발 PC에서는:

```bash
joystick-bridge
```

스틱을 움직이는 것만으로 자동제어권이 확실히 넘어오는 구조는 아닙니다. 비상 전환은 조종기의 mode switch를 사용합니다.

- `LOITER`: 현재 위치 유지 후 수동 입력
- `ALT_HOLD`: 고도 유지 수동조종
- `STABILIZE`: 자세 기반 수동조종

YOLO controller는 GUIDED가 아닌 모드를 감지하면 `PILOT_OVERRIDE`로 들어가 자동 setpoint 전송을 중단합니다.

## 12. 실제 Pixhawk + Jetson으로 전환

시뮬레이션과 실기체의 상위 제어 구조는 비슷하지만 연결 대상이 달라집니다.

```text
Simulation: Python/YOLO -> TCP/UDP MAVLink -> ArduPilot SITL -> Gazebo
Real:       Python/YOLO -> serial MAVLink  -> Pixhawk       -> ESC/motors
```

현재 확인한 실제 장비 설정:

- Pixhawk: PH4-mini, ArduCopter 4.6.3
- Jetson device: `/dev/ttyACM0` 또는 안정적인 `/dev/serial/by-id/...`
- USB 시험 baud: 115200
- `SERIAL2_PROTOCOL=2` (MAVLink2)
- 확인 당시 `SERIAL2_BAUD=57`, 즉 57600
- `ARMING_CHECK=1`
- `FRAME_CLASS=1`, `FRAME_TYPE=1` (Quad/X)
- `BATT_MONITOR=0`은 실비행 전에 반드시 배터리 모니터 설정 필요
- pre-arm 경고로 RC 미검출 및 compass 미보정 확인됨

읽기 중심 연결 시험:

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
```

Mission Planner에 전달하려면 Jetson에서:

```bash
mavproxy.py \
  --master=/dev/ttyACM0 \
  --baudrate=115200 \
  --out=tcpin:0.0.0.0:5760
```

실기체에서 ARM 또는 모터 시험을 하기 전에는 프로펠러를 제거하고 RC, compass, battery monitor, failsafe, flight mode, motor order/direction을 검증해야 합니다. 시뮬레이션 파라미터를 실기체에 그대로 복사하면 안 됩니다.

## 13. 문제 해결

### Gazebo GUI가 EGL/ZINK 오류로 종료됨

```bash
env -u WAYLAND_DISPLAY gz sim <world.sdf>
```

### 카메라 topic 확인

```bash
gz topic -l | grep -E 'image|camera'
```

현재 topic:

```text
/world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image
```

stream 활성화:

```bash
gz topic -t \
  /world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image/enable_streaming \
  -m gz.msgs.Boolean -p 'data: 1'
```

### YOLO가 GPU를 사용하는지 확인

```bash
nvidia-smi
```

YOLO 시작 로그에 다음 형태가 보여야 합니다.

```text
torch-2.6.0+cu124 CUDA:0
```

### Mission Planner가 UDP 14550에 연결되지 않음

```bash
ss -lunp | grep 14550
pgrep -af mavproxy.py
```

Windows Mission Planner라면 WSL의 `127.0.0.1`이 아니라 Windows host IP로 MAVProxy output이 생성됐는지 확인합니다.

### ARM 실패

MAVProxy의 `STATUSTEXT` 또는 Mission Planner Messages를 확인합니다. frame class/type, EKF, compass, RC, throttle neutral, battery/failsafe 경고를 무시하지 마십시오. `ARMING_CHECK=0`은 제한된 SITL 시험 외에는 사용하지 않습니다.

### GUIDED로 전환됐지만 움직이지 않음

탐지 중심 오차가 deadband 안이면 `vx=0`, `vy=0`이 정상입니다. 중앙 유지 후 하강 로그가 나타나는지, 상대고도 telemetry가 들어오는지 확인합니다.

## 14. 종료

각 실행 터미널에서 `Ctrl+C`를 누릅니다. 남은 프로세스 확인:

```bash
pgrep -af 'gz-sim|arducopter|sim_vehicle.py|mavproxy.py|gazebo_yolo.py|MissionPlanner.exe'
```

## 15. 재현성 체크리스트

- [ ] Ubuntu 및 Gazebo 버전 기록
- [ ] ArduPilot commit 고정
- [ ] ardupilot_gazebo commit과 로컬 patch 저장
- [ ] 프로젝트 world/model/mesh 저장
- [ ] YOLOv5 commit과 Python requirements 고정
- [ ] weights 배포 위치 및 SHA256 기록
- [ ] launcher가 `$HOME` 하드코딩 대신 저장소 경로 사용
- [ ] Mission Planner 연결 주소와 port 기록
- [ ] SITL unit test 및 짧은 smoke mission 자동화
- [ ] 실제 기체 파라미터와 SITL 파라미터 분리

## 16. Upstream 참고자료

- [Gazebo Jetty binary installation](https://gazebosim.org/docs/jetty/install_ubuntu/)
- [ArduPilot: Using SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [ArduPilot: Setting up SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [ArduPilot Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [YOLOv5](https://github.com/ultralytics/yolov5)
