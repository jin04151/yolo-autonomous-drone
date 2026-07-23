"""Portable AUTO-to-GUIDED handoff logic for a downward-facing camera."""
from dataclasses import dataclass
from enum import Enum
from typing import Optional


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class VehicleState:
    mode: str
    armed: bool
    relative_alt_m: Optional[float] = None


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    vz: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class HandoffConfig:
    auto_mode: str = "AUTO"
    guided_mode: str = "GUIDED"
    confirm_frames: int = 5
    center_deadband: float = 0.12
    xy_gain: float = 0.6
    max_xy_speed: float = 0.4
    target_lost_timeout: float = 1.0
    center_hold_sec: float = 1.0
    descend_speed: float = 0.2
    min_approach_alt: float = 0.8


class HandoffState(Enum):
    WAIT_AUTO = "wait_auto"
    CONFIRMING = "confirming"
    REQUESTING_GUIDED = "requesting_guided"
    GUIDED_TRACKING = "guided_tracking"
    PILOT_OVERRIDE = "pilot_override"


@dataclass(frozen=True)
class HandoffOutput:
    status: str
    command: Optional[VelocityCommand] = None
    request_mode: Optional[str] = None
    send_stop: bool = False
    error_x: float = 0.0
    error_y: float = 0.0


def downward_camera_velocity(target, frame_shape, config):
    """Map image error to MAV_FRAME_BODY_NED horizontal velocity.

    The Gazebo camera optical axis points down. Image right maps to body right
    (+vy), while image up maps to body forward (+vx).
    """
    height, width = frame_shape[:2]
    x1, y1, x2, y2 = target["xyxy"]
    center_x = (x1 + x2) * 0.5
    center_y = (y1 + y2) * 0.5
    error_x = (center_x - width * 0.5) / (width * 0.5)
    error_y = (center_y - height * 0.5) / (height * 0.5)

    active_x = 0.0 if abs(error_x) <= config.center_deadband else error_x
    active_y = 0.0 if abs(error_y) <= config.center_deadband else error_y
    vx = clamp(-config.xy_gain * active_y, -config.max_xy_speed, config.max_xy_speed)
    vy = clamp(config.xy_gain * active_x, -config.max_xy_speed, config.max_xy_speed)
    return VelocityCommand(vx=vx, vy=vy), error_x, error_y


def filter_control_detections(
    detections,
    frame_shape,
    min_area_ratio=0.0002,
    max_area_ratio=0.20,
    edge_margin=2,
):
    """Select a plausible target while rejecting frame-wide and edge artifacts."""
    height, width = frame_shape[:2]
    frame_area = width * height
    candidates = []
    for detection in detections:
        x1, y1, x2, y2 = detection["xyxy"]
        area_ratio = max(0, x2 - x1) * max(0, y2 - y1) / frame_area
        touches_edge = (
            x1 <= edge_margin
            or y1 <= edge_margin
            or x2 >= width - edge_margin
            or y2 >= height - edge_margin
        )
        if not touches_edge and min_area_ratio <= area_ratio <= max_area_ratio:
            candidates.append(detection)
    return candidates


def select_control_target(
    detections,
    target_class,
    frame_shape,
    min_area_ratio=0.0002,
    max_area_ratio=0.20,
    edge_margin=2,
):
    """Select the highest-confidence valid target for automatic control."""
    candidates = filter_control_detections(
        detections,
        frame_shape,
        min_area_ratio,
        max_area_ratio,
        edge_margin,
    )

    if target_class != "auto":
        candidates = [
            detection
            for detection in candidates
            if detection["name"] == target_class
        ]
    else:
        basket_candidates = [
            detection
            for detection in candidates
            if "basket" in detection["name"].lower()
        ]
        if basket_candidates:
            candidates = basket_candidates

    return max(candidates, key=lambda detection: detection["conf"], default=None)


class AutoGuidedHandoff:
    def __init__(self, config):
        self.config = config
        self.state = HandoffState.WAIT_AUTO
        self.confirm_count = 0
        self.last_seen_at = None
        self.centered_since = None

    def finish_mode_request(self, success, now):
        if success:
            self.state = HandoffState.GUIDED_TRACKING
            self.last_seen_at = now
        else:
            self.state = HandoffState.WAIT_AUTO
        self.confirm_count = 0
        self.centered_since = None

    def update(self, target, frame_shape, vehicle, now):
        if self.state == HandoffState.REQUESTING_GUIDED:
            return HandoffOutput("waiting for GUIDED acknowledgement", send_stop=True)

        if self.state == HandoffState.GUIDED_TRACKING:
            if vehicle.mode != self.config.guided_mode:
                self.state = HandoffState.PILOT_OVERRIDE
                self.confirm_count = 0
                self.centered_since = None
                return HandoffOutput(
                    f"pilot override: mode={vehicle.mode}",
                    send_stop=True,
                )

            if target is not None:
                self.last_seen_at = now
                command, error_x, error_y = downward_camera_velocity(
                    target, frame_shape, self.config
                )
                centered = command.vx == 0.0 and command.vy == 0.0
                if not centered:
                    self.centered_since = None
                    status = "GUIDED tracking target"
                else:
                    if self.centered_since is None:
                        self.centered_since = now
                    centered_for = now - self.centered_since
                    altitude = vehicle.relative_alt_m
                    can_descend = (
                        centered_for >= self.config.center_hold_sec
                        and altitude is not None
                        and altitude > self.config.min_approach_alt
                    )
                    if can_descend:
                        command = VelocityCommand(
                            command.vx,
                            command.vy,
                            self.config.descend_speed,
                        )
                        status = f"GUIDED approaching target alt={altitude:.2f}m"
                    elif altitude is not None and altitude <= self.config.min_approach_alt:
                        status = f"GUIDED approach complete alt={altitude:.2f}m: holding"
                    elif altitude is None:
                        status = "GUIDED target centered: waiting for altitude"
                    else:
                        status = f"GUIDED target centered {centered_for:.1f}s: holding"
                return HandoffOutput(
                    status,
                    command=command,
                    error_x=error_x,
                    error_y=error_y,
                )

            lost_for = 0.0 if self.last_seen_at is None else now - self.last_seen_at
            self.centered_since = None
            if lost_for >= self.config.target_lost_timeout:
                return HandoffOutput(
                    f"GUIDED target lost {lost_for:.1f}s: holding",
                    command=VelocityCommand(0.0, 0.0),
                )
            return HandoffOutput(
                "GUIDED target briefly lost: holding",
                command=VelocityCommand(0.0, 0.0),
            )

        if self.state == HandoffState.PILOT_OVERRIDE:
            if vehicle.mode == self.config.auto_mode and vehicle.armed:
                self.state = HandoffState.WAIT_AUTO
            else:
                return HandoffOutput(f"pilot override active: mode={vehicle.mode}")

        if vehicle.mode != self.config.auto_mode or not vehicle.armed:
            self.state = HandoffState.WAIT_AUTO
            self.confirm_count = 0
            return HandoffOutput(
                f"monitoring: mode={vehicle.mode} armed={vehicle.armed}"
            )

        if target is None:
            self.state = HandoffState.WAIT_AUTO
            self.confirm_count = 0
            return HandoffOutput("AUTO: searching for target")

        self.state = HandoffState.CONFIRMING
        self.confirm_count += 1
        if self.confirm_count < max(1, self.config.confirm_frames):
            return HandoffOutput(
                f"AUTO: confirming target {self.confirm_count}/{self.config.confirm_frames}"
            )

        self.state = HandoffState.REQUESTING_GUIDED
        self.confirm_count = 0
        return HandoffOutput(
            "target confirmed: requesting GUIDED",
            request_mode=self.config.guided_mode,
            send_stop=True,
        )
