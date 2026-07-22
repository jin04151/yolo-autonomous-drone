# Third-Party Notices

This repository interoperates with or derives configuration from the following
upstream projects. Their licenses apply to their respective components.

## ardupilot_gazebo

- Source: <https://github.com/ArduPilot/ardupilot_gazebo>
- Pinned revision: `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- License: GNU Lesser General Public License v3.0

The Gazebo vehicle SDF files under `sim/gazebo/models/` were adapted from the
ArduPilot Iris model structure. Propeller meshes and grass textures are not
copied into this repository; they are resolved from a separately installed
`ardupilot_gazebo` checkout at runtime.

## ArduPilot

- Source: <https://github.com/ArduPilot/ardupilot>
- Pinned revision: `ceb710cc7557ef4bd226c7601a7c5eb7fedb4ea2`
- License: GNU General Public License v3.0 or later

ArduPilot is cloned and built separately by the setup script.

## YOLOv5

- Source: <https://github.com/ultralytics/yolov5>
- Pinned revision: `915bbf294bb74c859f0b41f1c23bc395014ea679`
- License at the pinned revision: GNU General Public License v3.0

YOLOv5 is cloned separately by the setup script. Trained checkpoints are not
distributed by this repository.

## Private Project Meshes

`ensambFinal.STL` and `basket.stl` are not distributed in this repository.
Their ownership and redistribution terms have not been confirmed. Do not add
them to Git history without written permission from the rights holder.
