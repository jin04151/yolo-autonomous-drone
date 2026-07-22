# Gazebo SITL 설치 및 운용 안내서

이 문서는 새 Windows/WSL 컴퓨터에서 저장소를 설치하고 Gazebo, ArduPilot SITL, Mission Planner, YOLO, 조종기를 연결하는 절차입니다.

검증 기준: 2026-07-23, Windows 11, WSL2 Ubuntu 24.04, Gazebo Jetty.

## 1. Windows 준비

관리자 PowerShell에서 WSL을 확인합니다.

```powershell
wsl --status
wsl --list --verbose
```

Ubuntu 24.04가 없으면:

```powershell
wsl --install -d Ubuntu-24.04
```

NVIDIA GPU를 쓸 경우 Windows NVIDIA driver를 설치한 뒤 Ubuntu에서 확인합니다.

```bash
nvidia-smi
```

WSL에 별도 Linux CUDA driver를 설치하지 않습니다. Windows driver가 WSL GPU를 제공합니다.

## 2. 저장소와 전체 설치

Ubuntu 터미널에서:

```bash
cd ~
git clone https://github.com/jin04151/yolo-autonomous-drone.git
cd yolo-autonomous-drone
bash scripts/setup_gazebo_sitl.sh
source ~/.bashrc
```

설치기는 다음 작업을 수행합니다.

1. Gazebo Jetty와 C++/GStreamer/OpenCV 의존성 설치
2. `~/ardupilot`, `~/ardupilot_gazebo`, `~/yolov5` clone
3. 검증된 commit checkout
4. ArduCopter SITL과 Gazebo plugin build
5. `~/venv-yolov5` 생성과 PyTorch 설치
6. 실행 명령을 `~/.local/bin`에 symlink
7. SDF, Python import와 단위 테스트 검증

기존 upstream checkout은 기본적으로 수정하지 않습니다. 명시적으로 최신 remote object를 받아 검증 commit으로 맞추려면:

```bash
UPDATE_EXISTING=1 bash scripts/setup_gazebo_sitl.sh
```

GPU 없이 CPU PyTorch를 설치하려면:

```bash
INSTALL_CUDA_TORCH=0 bash scripts/setup_gazebo_sitl.sh
```

경로를 바꿀 수도 있습니다.

```bash
ARDUPILOT_DIR="$HOME/src/ardupilot" \
ARDUPILOT_GAZEBO_DIR="$HOME/src/ardupilot_gazebo" \
YOLOV5_DIR="$HOME/src/yolov5" \
bash scripts/setup_gazebo_sitl.sh
```

## 3. YOLO checkpoint 배치

학습 모델은 저장소에 포함되지 않습니다. 승인된 `best_v5.pt`를 다음에 둡니다.

```bash
cp /mnt/c/Users/<WINDOWS_USER>/Downloads/best_v5.pt \
  ~/yolo-autonomous-drone/weights/best_v5.pt
```

현재 개발 파일 확인값:

```text
size:   3,856,751 bytes
sha256: 462f6d8bf76a1a9fcbace5b4f0b4930b0750acb9d399420446350532185ce988
```

확인:

```bash
sha256sum weights/best_v5.pt
bash scripts/validate_setup.sh
```

다른 위치의 모델은 환경 변수로 지정합니다.

```bash
export YOLO_WEIGHTS="$HOME/models/best_v5.pt"
```

## 4. 기본 실행

### 터미널 1: Gazebo + YOLO

```bash
gazebo-my-drone
```

이 명령은 잔디 월드와 하향 카메라를 열고 카메라가 준비되면 YOLO를 자동 시작합니다. WSLg의 EGL/ZINK 문제를 피하기 위해 내부적으로 `WAYLAND_DISPLAY`를 해제합니다.

### 터미널 2: SITL + MAVProxy

```bash
sitl-my-drone
```

MAVProxy prompt와 ArduPilot heartbeat가 나타나야 합니다. 실행기는 다음 client 출력을 자동 생성합니다.

```text
127.0.0.1:14550 UDP       Mission Planner
127.0.0.1:5772 TCP listen YOLO/MAVLink tools
127.0.0.1:5773 TCP listen joystick bridge
```

Gazebo와 SITL 중 어느 쪽을 먼저 시작해도 되지만, 비행 명령은 두 프로그램의 연결과 EKF 초기화 후에 보냅니다.

### Headless Gazebo

GUI와 YOLO 없이 물리 연결만 확인할 때:

```bash
gazebo-my-drone-headless
```

## 5. Mission Planner

권장 구성은 Mission Planner를 Windows에 설치해 실행하는 것입니다. WSL용 Mono 실행은 일부 GUI와 joystick 기능이 불안정할 수 있습니다.

같은 WSL 안의 Mission Planner를 쓸 경우:

```bash
cd ~/MissionPlanner
env -u WAYLAND_DISPLAY mono MissionPlanner.exe
```

연결 설정:

```text
Connection: UDP
Port: 14550
Baud: UDP에서는 사용되지 않음
```

Windows Mission Planner가 WSL의 UDP를 받지 못하면 Windows host IP를 확인합니다.

```bash
ip route | awk '/default/ {print $3; exit}'
```

그 주소가 예를 들어 `172.25.64.1`이면 `sitl-my-drone`에 output을 추가합니다.

```bash
sitl-my-drone --out=172.25.64.1:14550
```

Windows 방화벽에서 UDP 14550 허용이 필요할 수 있습니다.

## 6. 수동 명령 시험

MAVProxy prompt에서:

```text
mode guided
arm throttle
takeoff 2
mode land
```

같은 동작 중 GUIDED, ARM, 2m 이륙을 한 번에 실행하려면:

```bash
drone-takeoff 2
```

`ARM` 실패 시 ArduPilot `STATUSTEXT`를 확인합니다. frame, EKF, throttle, RC, compass 경고를 해결해야 하며 실기체에서 `ARMING_CHECK=0`으로 우회하면 안 됩니다.

## 7. AUTO mission과 YOLO 인계

현재 월드 기준:

| 대상 | 좌표 |
|---|---|
| Home | `-35.363262, 149.165237`, elevation 584m |
| 드론 | Gazebo `(0, 0)` |
| Basket | Gazebo `(0, 4)` |
| 약 10m 북쪽 waypoint | `-35.363172, 149.165237` |

Mission Planner Plan에서:

1. `TAKEOFF`, 상대고도 2~3m를 추가합니다.
2. 북쪽 waypoint를 추가합니다.
3. `Write`를 눌러 SITL에 mission을 업로드합니다.
4. Flight Data에서 ARM합니다.
5. `AUTO`로 전환합니다.

SITL 파라미터 `WP_SPD=0.5`로 AUTO 수평 속도를 0.5m/s로 제한합니다. ArduPilot 안정 버전은 `WPNAV_SPEED`와 cm/s 단위를 사용할 수 있으므로 실제 parameter 목록을 확인합니다.

YOLO controller는 다음 조건에서만 인계합니다.

- armed
- mode `AUTO`
- 같은 target 5 frame 연속 탐지

인계 후 `GUIDED`에서 target 중심으로 최대 0.4m/s 이동하고, 중앙 1초 유지 후 0.2m/s로 하강합니다. 상대고도 0.8m에서 하강을 멈춥니다.

## 8. Gazebo GUI의 YOLO 화면

원본 카메라 topic:

```text
/world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image
```

YOLO annotated topic:

```text
/yolo/annotated
```

월드의 `ImageDisplay` plugin이 `/yolo/annotated`를 구독하므로 별도 OpenCV 창 없이 Gazebo GUI에서 box를 봅니다. topic 확인:

```bash
gz topic -l | grep -E 'image|yolo'
```

YOLO 시작 로그에서 GPU 사용을 확인합니다.

```text
CUDA:0 (NVIDIA ...)
```

## 9. RadioMaster joystick

조종기에서 USB mode를 `USB Joystick`으로 선택합니다. 관리자 PowerShell에서 매번 현재 BUSID를 확인합니다.

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

`bind`는 최초 1회 또는 Shared가 아닐 때만 필요합니다. BUSID는 재연결 후 달라질 수 있습니다.

Ubuntu에서:

```bash
lsusb
jstest /dev/input/js0
joystick-bridge --dry-run
joystick-bridge
```

현재 기본 axis:

```text
roll=0, pitch=1, throttle=2, yaw=3
```

장치마다 방향이 다르면 실제 `--dry-run` 결과를 기준으로 변경합니다.

```bash
joystick-bridge --reverse pitch,throttle
```

비상 수동 인계는 스틱 움직임 자체가 아니라 flight mode switch로 `LOITER`, `ALT_HOLD` 또는 `STABILIZE`로 전환해 수행합니다. mode가 `GUIDED`를 벗어나면 YOLO가 자동 명령을 멈춥니다.

## 10. 포트 계약

| 포트 | 필수 | 방향 | 설명 |
|---|---|---|---|
| UDP 9002 | 예 | Gazebo <-> SITL | JSON/FDM |
| TCP 5760 | 예 | SITL -> MAVProxy | master link |
| TCP 5772 | 예 | client -> MAVProxy | YOLO와 이륙 도구 |
| TCP 5773 | 조종 시 | client -> MAVProxy | joystick bridge |
| UDP 14550 | GCS 사용 시 | MAVProxy -> GCS | Mission Planner |
| UDP 5600 | YOLO 사용 시 | Gazebo -> detector | H.264 camera |

확인 명령:

```bash
ss -lntup | grep -E '5760|5772|5773|14550|5600|9002'
pgrep -af 'gz-sim|arducopter|sim_vehicle.py|mavproxy.py|gazebo_yolo.py'
```

## 11. 실제 Pixhawk + Jetson 전환

실기체에서는 Gazebo와 SITL 대신 Pixhawk serial link가 MAVProxy master가 됩니다.

```text
Simulation: MAVProxy --master=tcp:127.0.0.1:5760
Vehicle:    MAVProxy --master=/dev/serial/by-id/<PIXHAWK> --baudrate=115200
```

확인된 장비 상태:

- PH4-mini, ArduCopter 4.6.3
- USB device `/dev/ttyACM0` 또는 `/dev/serial/by-id/usb-ArduPilot_PH4-mini_...`
- `SERIAL2_PROTOCOL=2`
- 확인 당시 `SERIAL2_BAUD=57` 즉 57600
- `FRAME_CLASS=1`, `FRAME_TYPE=1` Quad/X
- `ARMING_CHECK=1`
- `BATT_MONITOR=0`은 실비행 전에 설정 필요
- RC 미검출과 compass 미보정 pre-arm 경고가 확인됨

읽기 중심 연결:

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
```

client와 Mission Planner까지 전달:

```bash
mavproxy.py \
  --master=/dev/ttyACM0 \
  --baudrate=115200 \
  --out=tcpin:127.0.0.1:5772 \
  --out=tcpin:127.0.0.1:5773 \
  --out=udpout:<MISSION_PLANNER_IP>:14550
```

USB baud는 USB ACM 전송 속도를 의미하지 않을 수 있지만 도구 인자로는 일치시킵니다. TELEM2 UART를 사용한다면 실제 `SERIAL2_BAUD`와 물리 연결 baud를 맞춰야 합니다.

실기체 ARM 전에 프로펠러를 제거하고 RC, compass, accelerometer, battery monitor, failsafe, motor order/direction을 확인합니다.

## 12. 문제 해결

### Gazebo가 EGL/ZINK 오류로 종료

프로젝트 실행기는 이미 아래 방식으로 실행합니다.

```bash
env -u WAYLAND_DISPLAY gz sim ...
```

### YOLO 창 또는 GUI 영상이 없음

```bash
gz topic -l | grep image
pgrep -af gazebo_yolo.py
gst-inspect-1.0 x264enc
```

`x264enc`가 없으면 `gstreamer1.0-plugins-ugly` 설치를 확인합니다.

### Mission Planner 14550 연결 실패

```bash
pgrep -af mavproxy.py
ss -lunp | grep 14550
```

Windows에서 실행 중이면 loopback이 아닌 Windows host IP output이 있는지 확인합니다.

### `mode alt_hold` 직후 하강

mode 전환 시 현재 RC3 값이 throttle 입력으로 적용됩니다. RC3가 1000이면 ALT_HOLD에서 하강 명령입니다. 전환 전 실제 joystick bridge가 RC3 중립 약 1500을 보내는지 확인합니다.

### 조종기 중립인데 기체가 이동

`joystick-bridge --dry-run`으로 axis와 방향을 확인하고, Mission Planner/MAVProxy의 `RC_CHANNELS`에서 RC1~RC4가 중립인지 확인합니다. RC override와 다른 client 명령을 동시에 보내지 않습니다.

## 13. 종료와 검증

각 실행 터미널에서 `Ctrl+C`를 누릅니다. 남은 프로세스 확인:

```bash
pgrep -af 'gz-sim|arducopter|sim_vehicle.py|mavproxy.py|gazebo_yolo.py|MissionPlanner.exe'
```

정적 검증:

```bash
bash scripts/validate_setup.sh
```

headless smoke 순서:

```bash
# terminal 1
gazebo-my-drone-headless

# terminal 2
sitl-my-drone

# terminal 3
ss -lntp | grep -E '5772|5773'
drone-takeoff 2
```

검증이 끝나면 MAVProxy에서 `mode land` 후 프로세스를 종료합니다.

## 14. Upstream 문서

- [Gazebo Jetty Ubuntu installation](https://gazebosim.org/docs/jetty/install_ubuntu/)
- [ArduPilot SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
- [YOLOv5](https://github.com/ultralytics/yolov5)
