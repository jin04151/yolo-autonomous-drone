# Astro Drone Gazebo Assets

마지막 검증: 2026-07-23, Gazebo Jetty / Gazebo Sim 10.

## 월드

`worlds/ensamb_iris_runway.sdf`는 다음을 포함합니다.

- 1500m x 1500m 무광 짙은 녹색 ground plane
- home `-35.363262, 149.165237`, elevation 584m
- 드론 시작점 `(0, 0, 0.195)`, yaw 90deg
- 흰색 basket target `(0, 4, 0)`
- Gazebo GUI의 `/yolo/annotated` image panel

## 모델

| 경로 | 역할 |
|---|---|
| `models/ensamb_with_standoffs` | 기본 형상, 충돌체, IMU, 하향 카메라, rotor link |
| `models/ensamb_with_gimbal` | ArduPilot plugin, motor/lift-drag systems를 포함하는 wrapper |
| `models/target_basket` | 흰색 box 표적과 동일 크기 collision |

프로펠러 mesh는 `ardupilot_gazebo`의 `iris_with_standoffs` 모델을 참조합니다. 실행 시 해당 upstream model directory가 `GZ_SIM_RESOURCE_PATH`에 있어야 하며 프로젝트 실행기가 자동 설정합니다. 바닥은 영상 오탐을 줄이기 위해 texture와 반사를 사용하지 않습니다.

## 카메라

```text
topic: /world/ensamb_iris_runway/model/ensamb_with_gimbal/model/ensamb_with_standoffs/link/down_camera_link/sensor/down_camera/image
resolution: 640x480
rate: 10 Hz
horizontal_fov: 2.7507 rad
UDP H.264 port: 5600
```

실제 IMX219 계열 센서와 동일한 제품 모델을 시뮬레이션한 것이 아니라, 해상도·방향·화각을 근사한 Gazebo pinhole camera입니다.

## 물리 정확도

공개 저장소에는 소유권이 확인되지 않은 프로젝트 STL을 포함하지 않습니다. 기본 외형, collision, mass, inertia와 thrust coefficients는 SDF 근사값이며 실제 비행 controller tuning의 근거로 사용하지 않습니다.
