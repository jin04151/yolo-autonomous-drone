#!/usr/bin/env python3
"""Run YOLOv5 on the Gazebo gimbal camera UDP stream."""
import argparse
import os
import pathlib
import queue
import signal
import subprocess
import sys
import threading
import time
import types
from pathlib import Path

import cv2
import numpy as np
import torch
from gz.msgs import image_pb2
from gz.msgs.image_pb2 import Image
from gz.transport import Node

FILE = Path(__file__).resolve()
ROOT = FILE.parent
REPO_ROOT = FILE.parents[2]
YOLOV5_DIR = Path(os.environ.get("YOLOV5_DIR", Path.home() / "yolov5")).expanduser().resolve()
if not YOLOV5_DIR.is_dir():
    raise SystemExit(
        f"YOLOv5 checkout not found: {YOLOV5_DIR}. "
        "Set YOLOV5_DIR or run scripts/setup_gazebo_sitl.sh."
    )
for module_path in (ROOT, YOLOV5_DIR):
    if str(module_path) not in sys.path:
        sys.path.insert(0, str(module_path))


def patch_yolov5_checkpoint_compatibility():
    """Keep an unmodified YOLOv5 v7 checkout working on Python 3.12 / Torch 2.6."""
    if "pathlib._local" not in sys.modules:
        pathlib_local = types.ModuleType("pathlib._local")
        pathlib_local.Path = pathlib.Path
        pathlib_local.PosixPath = pathlib.PosixPath
        pathlib_local.WindowsPath = pathlib.PosixPath
        pathlib_local.PurePath = pathlib.PurePath
        pathlib_local.PurePosixPath = pathlib.PurePosixPath
        pathlib_local.PureWindowsPath = pathlib.PureWindowsPath
        sys.modules["pathlib._local"] = pathlib_local

    original_torch_load = torch.load

    def compatible_torch_load(*args, **kwargs):
        kwargs.setdefault("weights_only", False)
        return original_torch_load(*args, **kwargs)

    torch.load = compatible_torch_load


patch_yolov5_checkpoint_compatibility()

from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, smart_inference_mode
from basket_handoff import (
    AutoGuidedHandoff,
    HandoffConfig,
    VehicleState,
    downward_camera_velocity,
    select_control_target,
)


def gazebo_pipeline(port):
    return (
        f"udpsrc port={port} "
        "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 "
        "! rtph264depay "
        "! avdec_h264 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink drop=true sync=false"
    )


def handle_sigint(_signum, _frame):
    raise SystemExit(130)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights",
        default=os.environ.get("YOLO_WEIGHTS", str(REPO_ROOT / "weights" / "best_v5.pt")),
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--enable-topic", default="auto")
    parser.add_argument("--no-enable-streaming", action="store_true")
    parser.add_argument("--open-timeout", type=float, default=10.0)
    parser.add_argument("--img", "--imgsz", dest="imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--no-view", action="store_true")
    parser.add_argument("--gui-topic", default="/yolo/annotated")
    parser.add_argument("--no-gui-publish", action="store_true")
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--control", action="store_true", help="track the target while the vehicle is already in GUIDED")
    parser.add_argument("--handoff-on-detect", action="store_true", help="switch from AUTO to GUIDED after stable target detection")
    parser.add_argument("--mavlink", default="tcp:127.0.0.1:5772", help="MAVProxy hub connection for control mode")
    parser.add_argument("--mavlink-retry", type=float, default=2.0, help="seconds between MAVLink connection attempts")
    parser.add_argument("--target-class", default="auto", help="class name to track, or auto to prefer names containing basket")
    parser.add_argument("--control-min-area-ratio", type=float, default=0.0002, help="minimum frame area ratio for a control target")
    parser.add_argument("--control-max-area-ratio", type=float, default=0.20, help="maximum frame area ratio for a control target")
    parser.add_argument("--control-edge-margin", type=int, default=2, help="reject control targets touching this many edge pixels")
    parser.add_argument("--control-rate", type=float, default=5.0, help="MAVLink command rate in Hz")
    parser.add_argument("--confirm-frames", type=int, default=5, help="consecutive detections required before GUIDED handoff")
    parser.add_argument("--xy-gain", type=float, default=0.6, help="horizontal velocity gain from normalized image error")
    parser.add_argument("--max-xy-speed", type=float, default=0.4, help="maximum horizontal tracking speed in m/s")
    parser.add_argument("--center-deadband", type=float, default=0.15, help="normalized image error considered centered")
    parser.add_argument("--target-lost-timeout", type=float, default=1.0, help="seconds before reporting a lost target while holding position")
    parser.add_argument("--center-hold-sec", type=float, default=1.0, help="seconds the target must remain centered before descending")
    parser.add_argument("--descend-speed", type=float, default=0.2, help="target approach descent speed in m/s")
    parser.add_argument("--min-approach-alt", type=float, default=0.8, help="minimum relative altitude for target approach")
    parser.add_argument("--auto-start", action="store_true", help="switch to GUIDED, arm, and take off before starting control")
    parser.add_argument("--takeoff-alt", type=float, default=2.0, help="takeoff altitude in meters for --auto-start")
    parser.add_argument("--takeoff-timeout", type=float, default=25.0, help="seconds to wait for takeoff altitude")
    return parser.parse_args()


class GazeboImagePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.node = Node()
        self.publisher = self.node.advertise(topic, Image)
        if not self.publisher.valid():
            raise SystemExit(f"Could not advertise Gazebo image topic: {topic}")
        print(f"Publishing annotated video to Gazebo GUI: {topic}", flush=True)

    def publish(self, frame):
        height, width = frame.shape[:2]
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        message = Image()
        message.width = width
        message.height = height
        message.step = width * 3
        message.pixel_format_type = image_pb2.RGB_INT8
        message.data = np.ascontiguousarray(rgb_frame).tobytes()
        self.publisher.publish(message)


def load_mavutil():
    try:
        from pymavlink import mavutil
        return mavutil
    except ModuleNotFoundError:
        ardupilot_site = (
            Path.home()
            / "venv-ardupilot"
            / f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
        )
        if ardupilot_site.exists():
            sys.path.append(str(ardupilot_site))
            from pymavlink import mavutil
            return mavutil
        raise SystemExit(
            "pymavlink is not installed in venv-yolov5 and could not be found in ~/venv-ardupilot. "
            "Install it with: source ~/venv-yolov5/bin/activate && pip install pymavlink"
        )


class MavlinkVelocityController:
    def __init__(self, connection_string, rate_hz, heartbeat_timeout=1.0):
        self.mavutil = load_mavutil()
        self.connection_string = connection_string
        self.period = 1.0 / max(rate_hz, 0.1)
        self.last_send = 0.0
        print(f"Connecting MAVLink control: {connection_string}", flush=True)
        try:
            self.master = self.mavutil.mavlink_connection(
                connection_string,
                autoreconnect=True,
                source_system=250,
            )
            heartbeat = self.master.wait_heartbeat(timeout=heartbeat_timeout)
        except Exception as exc:
            master = getattr(self, "master", None)
            if master is not None:
                master.close()
            raise ConnectionError(f"MAVLink connection failed: {exc}") from exc

        if heartbeat is None:
            self.master.close()
            raise ConnectionError(f"No MAVLink heartbeat on {connection_string}")

        self.mode = self.mavutil.mode_string_v10(heartbeat)
        self.armed = bool(
            heartbeat.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        self.relative_alt_m = None
        print(
            f"MAVLink heartbeat received: system={self.master.target_system} "
            f"component={self.master.target_component} mode={self.mode} armed={self.armed}",
            flush=True,
        )

        self.type_mask = (
            self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

    def send_velocity(self, vx, vy, vz, yaw_rate, force=False):
        now = time.time()
        if not force and now - self.last_send < self.period:
            return
        self.last_send = now
        self.master.mav.set_position_target_local_ned_send(
            int(now * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            self.mavutil.mavlink.MAV_FRAME_BODY_NED,
            self.type_mask,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0,
            0,
            0,
            0,
            yaw_rate,
        )

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0, 0.0, force=True)

    def poll_state(self):
        while True:
            msg = self.master.recv_match(
                type=["HEARTBEAT", "GLOBAL_POSITION_INT"], blocking=False
            )
            if msg is None:
                break
            if msg.get_srcSystem() != self.master.target_system:
                continue
            if msg.get_type() == "HEARTBEAT":
                self.mode = self.mavutil.mode_string_v10(msg)
                self.armed = bool(
                    msg.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
            elif msg.get_type() == "GLOBAL_POSITION_INT":
                self.relative_alt_m = msg.relative_alt / 1000.0
        return VehicleState(self.mode, self.armed, self.relative_alt_m)

    def set_mode(self, mode, timeout=8.0):
        mode_mapping = self.master.mode_mapping()
        if mode not in mode_mapping:
            print(f"Flight mode {mode} is not available", flush=True)
            return False

        mode_id = mode_mapping[mode]
        print(f"Setting mode {mode}...", flush=True)
        self.master.mav.set_mode_send(
            self.master.target_system,
            self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg:
                self.mode = self.mavutil.mode_string_v10(msg)
                self.armed = bool(
                    msg.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
            if msg and self.mode == mode:
                print(f"Mode {mode}", flush=True)
                return True

        print(f"Timed out waiting for mode {mode}", flush=True)
        return False

    def arm(self, timeout=10.0):
        print("Arming...", flush=True)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and msg.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Armed", flush=True)
                return

        raise SystemExit("Timed out waiting for arm")

    def takeoff(self, altitude_m):
        print(f"Taking off to {altitude_m:.1f} m...", flush=True)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
        )

    def wait_altitude(self, altitude_m, timeout):
        target_alt = max(altitude_m * 0.8, altitude_m - 0.5)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
            if msg is None:
                continue
            relative_alt_m = msg.relative_alt / 1000.0
            print(f"altitude {relative_alt_m:.2f} m", flush=True)
            if relative_alt_m >= target_alt:
                print("Takeoff altitude reached", flush=True)
                return

        raise SystemExit(f"Timed out waiting for takeoff altitude {altitude_m:.1f} m")

    def auto_start(self, altitude_m, takeoff_timeout):
        if not self.set_mode("GUIDED"):
            raise SystemExit("Could not enter GUIDED for auto-start")
        self.arm()
        self.takeoff(altitude_m)
        self.wait_altitude(altitude_m, takeoff_timeout)

    def close(self):
        self.master.close()


class FrameReader:
    def __init__(self, pipeline, open_timeout):
        self.pipeline = pipeline
        self.frames = queue.Queue(maxsize=1)
        self.open_result = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

        try:
            ok, message = self.open_result.get(timeout=open_timeout)
        except queue.Empty:
            raise SystemExit(
                "Timed out waiting for Gazebo UDP camera stream. "
                "Check that gazebo-my-drone is running and camera streaming is enabled."
            )

        if not ok:
            raise SystemExit(message)

    def _run(self):
        cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.open_result.put((
                False,
                "Could not open Gazebo UDP camera stream. Check that gazebo-my-drone is running.",
            ))
            return

        self.open_result.put((True, ""))
        while not self.stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.02)
                continue

            try:
                if self.frames.full():
                    self.frames.get_nowait()
                self.frames.put_nowait(frame)
            except queue.Empty:
                pass
            except queue.Full:
                pass

        cap.release()

    def read(self, timeout=0.1):
        try:
            return True, self.frames.get(timeout=timeout)
        except queue.Empty:
            return False, None

    def release(self):
        self.stop_event.set()


def enable_gazebo_streaming(enable_topic):
    if enable_topic == "auto":
        try:
            topics = subprocess.run(
                ["gz", "topic", "-l"],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            ).stdout.splitlines()
        except Exception as exc:
            print(f"Could not list Gazebo topics: {exc}", flush=True)
            return

        candidates = [topic.strip() for topic in topics if topic.strip().endswith("/enable_streaming")]
        if not candidates:
            print("No Gazebo camera enable_streaming topic found. Start gazebo-my-drone first.", flush=True)
            return

        enable_topic = next(
            (topic for topic in candidates if "/sensor/camera/image/enable_streaming" in topic),
            candidates[0],
        )

    print(f"Enabling Gazebo camera stream: {enable_topic}", flush=True)
    try:
        subprocess.run(
            ["gz", "topic", "-t", enable_topic, "-m", "gz.msgs.Boolean", "-p", "data: 1"],
            check=True,
            timeout=5,
        )
    except Exception as exc:
        print(f"Could not enable Gazebo camera stream: {exc}", flush=True)


@smart_inference_mode()
def main():
    signal.signal(signal.SIGINT, handle_sigint)
    args = parse_args()

    if not args.no_enable_streaming:
        enable_gazebo_streaming(args.enable_topic)

    pipeline = gazebo_pipeline(args.port)
    print(f"Opening Gazebo camera UDP {args.port}...", flush=True)
    reader = FrameReader(pipeline, args.open_timeout)

    device = select_device(args.device)
    model = DetectMultiBackend(args.weights, device=device, fp16=args.half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size((args.imgsz, args.imgsz), s=stride)
    model.warmup(imgsz=(1, 3, *imgsz))

    gui_publisher = None
    if not args.no_gui_publish:
        gui_publisher = GazeboImagePublisher(args.gui_topic)

    print(f"Classes: {names}", flush=True)
    print(f"YOLO device: {model.device}", flush=True)
    if torch.cuda.is_available() and str(model.device).startswith("cuda"):
        print(f"GPU: {torch.cuda.get_device_name(0)}", flush=True)
    print(f"Listening to Gazebo camera UDP {args.port}. Press q or Esc to quit.", flush=True)
    if args.handoff_on_detect and args.auto_start:
        raise SystemExit("--handoff-on-detect cannot be combined with --auto-start")

    control_enabled = args.control or args.handoff_on_detect
    control_active = control_enabled
    controller = None
    next_mavlink_attempt = 0.0
    auto_start_done = False
    handoff_config = HandoffConfig(
        confirm_frames=args.confirm_frames,
        center_deadband=args.center_deadband,
        xy_gain=args.xy_gain,
        max_xy_speed=args.max_xy_speed,
        target_lost_timeout=args.target_lost_timeout,
        center_hold_sec=args.center_hold_sec,
        descend_speed=args.descend_speed,
        min_approach_alt=args.min_approach_alt,
    )
    handoff = AutoGuidedHandoff(handoff_config) if args.handoff_on_detect else None
    if args.handoff_on_detect:
        print(
            "AUTO-to-GUIDED handoff enabled. The controller will only take over "
            "after stable detection while AUTO and armed.",
            flush=True,
        )
    elif args.control:
        print(
            "Direct GUIDED target tracking enabled. Keep Mission Planner ready "
            "to switch LOITER/LAND.",
            flush=True,
        )

    last_print = 0.0
    frame_count = 0
    try:
        while True:
            ok, frame = reader.read(timeout=0.1)
            if not ok:
                if not args.no_view:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break
                continue

            frame_count += 1
            im = letterbox(frame, imgsz, stride=stride, auto=pt)[0]
            im = im.transpose((2, 0, 1))[::-1]
            im = np.ascontiguousarray(im)
            im = torch.from_numpy(im).to(model.device)
            im = im.half() if model.fp16 else im.float()
            im /= 255
            if im.ndimension() == 3:
                im = im[None]

            pred = model(im)
            pred = non_max_suppression(pred, args.conf, args.iou, max_det=100)
            annotator = Annotator(frame, line_width=2, example=str(names))
            detections = []

            for det in pred:
                if len(det):
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], frame.shape).round()
                    for *xyxy, conf, cls in reversed(det):
                        c = int(cls)
                        label = f"{names[c]} {conf:.2f}"
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        x1, y1, x2, y2 = [int(v) for v in xyxy]
                        detections.append({
                            "name": names[c],
                            "conf": float(conf),
                            "xyxy": (x1, y1, x2, y2),
                        })

            now = time.time()
            target = select_control_target(
                detections,
                args.target_class,
                frame.shape,
                args.control_min_area_ratio,
                args.control_max_area_ratio,
                args.control_edge_margin,
            )
            control_status = "detection only"

            if control_enabled and controller is None and now >= next_mavlink_attempt:
                next_mavlink_attempt = now + max(0.5, args.mavlink_retry)
                try:
                    controller = MavlinkVelocityController(
                        args.mavlink,
                        args.control_rate,
                        heartbeat_timeout=1.0,
                    )
                except ConnectionError as exc:
                    print(f"MAVLink not ready; retrying: {exc}", flush=True)

            if controller is None and control_enabled:
                control_status = f"waiting for MAVLink: {args.mavlink}"
            elif controller is not None and control_active:
                vehicle = controller.poll_state()
                if args.auto_start and not auto_start_done:
                    controller.auto_start(args.takeoff_alt, args.takeoff_timeout)
                    auto_start_done = True
                    vehicle = controller.poll_state()

                if handoff is not None:
                    output = handoff.update(target, frame.shape, vehicle, now)
                    control_status = output.status
                    if output.send_stop and vehicle.mode == handoff_config.guided_mode:
                        controller.stop()
                    if output.request_mode is not None:
                        controller.stop()
                        success = controller.set_mode(output.request_mode)
                        handoff.finish_mode_request(success, time.time())
                        control_status = (
                            "GUIDED handoff complete" if success else "GUIDED handoff failed"
                        )
                    elif output.command is not None:
                        command = output.command
                        controller.send_velocity(
                            command.vx,
                            command.vy,
                            command.vz,
                            command.yaw_rate,
                        )
                        control_status = (
                            f"{output.status} vx={command.vx:.2f} vy={command.vy:.2f} "
                            f"ex={output.error_x:.2f} ey={output.error_y:.2f}"
                        )
                elif vehicle.mode != "GUIDED" or not vehicle.armed:
                    control_status = (
                        f"waiting for GUIDED + armed: mode={vehicle.mode} armed={vehicle.armed}"
                    )
                elif target is None:
                    controller.stop()
                    control_status = "GUIDED no target: holding"
                else:
                    command, error_x, error_y = downward_camera_velocity(
                        target, frame.shape, handoff_config
                    )
                    controller.send_velocity(
                        command.vx,
                        command.vy,
                        command.vz,
                        command.yaw_rate,
                    )
                    control_status = (
                        f"GUIDED tracking vx={command.vx:.2f} vy={command.vy:.2f} "
                        f"ex={error_x:.2f} ey={error_y:.2f}"
                    )

                if target is not None:
                    tx1, ty1, tx2, ty2 = target["xyxy"]
                    cv2.circle(
                        frame,
                        (int((tx1 + tx2) * 0.5), int((ty1 + ty2) * 0.5)),
                        5,
                        (0, 255, 255),
                        -1,
                    )
            elif controller is not None:
                control_status = "automatic control paused"

            if control_enabled:
                cv2.putText(
                    frame,
                    control_status,
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255) if control_active else (180, 180, 180),
                    2,
                    cv2.LINE_AA,
                )

            if now - last_print >= args.print_every:
                last_print = now
                if detections:
                    msg = ", ".join(
                        f"{det['name']} {det['conf']:.2f} "
                        f"[{det['xyxy'][0]},{det['xyxy'][1]},{det['xyxy'][2]},{det['xyxy'][3]}]"
                        for det in detections
                    )
                    print(f"frame {frame_count}: {msg} | {control_status}", flush=True)
                else:
                    print(f"frame {frame_count}: no detections | {control_status}", flush=True)

            annotated_frame = annotator.result()
            if gui_publisher is not None:
                gui_publisher.publish(annotated_frame)

            if not args.no_view:
                cv2.imshow("gazebo_yolo", annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if controller is not None and key == ord("c"):
                    control_active = not control_active
                    controller.stop()
                    state = "ON" if control_active else "OFF"
                    print(f"Auto control toggled {state}", flush=True)
                if controller is not None and key == ord(" "):
                    controller.stop()
                    print("Stop command sent", flush=True)
    finally:
        if controller is not None:
            controller.stop()
            controller.close()
        reader.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
