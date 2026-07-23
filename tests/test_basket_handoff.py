import unittest
import sys
from pathlib import Path

VISION_SRC = Path(__file__).resolve().parents[1] / "src" / "vision"
sys.path.insert(0, str(VISION_SRC))

from basket_handoff import (
    AutoGuidedHandoff,
    HandoffConfig,
    HandoffState,
    VehicleState,
    downward_camera_velocity,
    select_control_target,
)


FRAME = (480, 640, 3)


def target(cx, cy, size=40):
    half = size // 2
    return {"xyxy": (cx - half, cy - half, cx + half, cy + half), "conf": 0.9}


class DownwardCameraControlTest(unittest.TestCase):
    def setUp(self):
        self.config = HandoffConfig(center_deadband=0.1, xy_gain=1.0, max_xy_speed=0.5)

    def test_center_means_hold(self):
        command, error_x, error_y = downward_camera_velocity(
            target(320, 240), FRAME, self.config
        )
        self.assertEqual((command.vx, command.vy), (0.0, 0.0))
        self.assertEqual((error_x, error_y), (0.0, 0.0))

    def test_image_up_and_right_means_body_forward_and_right(self):
        command, _, _ = downward_camera_velocity(target(560, 80), FRAME, self.config)
        self.assertGreater(command.vx, 0.0)
        self.assertGreater(command.vy, 0.0)
        self.assertLessEqual(command.vx, self.config.max_xy_speed)
        self.assertLessEqual(command.vy, self.config.max_xy_speed)


class ControlTargetSelectionTest(unittest.TestCase):
    def test_rejects_frame_wide_and_edge_detections(self):
        detections = [
            {"name": "white_box", "conf": 0.99, "xyxy": (0, 0, 615, 480)},
            {"name": "white_box", "conf": 0.90, "xyxy": (0, 10, 100, 80)},
        ]
        self.assertIsNone(select_control_target(detections, "auto", FRAME))

    def test_keeps_plausible_interior_target(self):
        expected = {
            "name": "white_box",
            "conf": 0.70,
            "xyxy": (250, 200, 300, 230),
        }
        detections = [
            {"name": "white_box", "conf": 0.99, "xyxy": (0, 0, 615, 480)},
            expected,
        ]
        self.assertEqual(
            select_control_target(detections, "auto", FRAME),
            expected,
        )


class AutoGuidedHandoffTest(unittest.TestCase):
    def setUp(self):
        self.config = HandoffConfig(confirm_frames=3, target_lost_timeout=0.5)
        self.handoff = AutoGuidedHandoff(self.config)

    def test_only_confirms_while_auto_and_armed(self):
        detected = target(400, 200)
        output = self.handoff.update(
            detected, FRAME, VehicleState("AUTO", False), now=1.0
        )
        self.assertIsNone(output.request_mode)
        self.assertEqual(self.handoff.confirm_count, 0)

        output = self.handoff.update(
            detected, FRAME, VehicleState("LOITER", True), now=2.0
        )
        self.assertIsNone(output.request_mode)
        self.assertEqual(self.handoff.confirm_count, 0)

    def test_confirmed_target_requests_guided(self):
        vehicle = VehicleState("AUTO", True)
        detected = target(400, 200)
        self.handoff.update(detected, FRAME, vehicle, now=1.0)
        self.handoff.update(detected, FRAME, vehicle, now=1.1)
        output = self.handoff.update(detected, FRAME, vehicle, now=1.2)
        self.assertEqual(output.request_mode, "GUIDED")
        self.assertEqual(self.handoff.state, HandoffState.REQUESTING_GUIDED)

    def test_guided_tracking_and_pilot_override(self):
        self.handoff.state = HandoffState.REQUESTING_GUIDED
        self.handoff.finish_mode_request(True, now=2.0)
        output = self.handoff.update(
            target(500, 100), FRAME, VehicleState("GUIDED", True), now=2.1
        )
        self.assertIsNotNone(output.command)

        output = self.handoff.update(
            target(500, 100), FRAME, VehicleState("LOITER", True), now=2.2
        )
        self.assertTrue(output.send_stop)
        self.assertEqual(self.handoff.state, HandoffState.PILOT_OVERRIDE)

        output = self.handoff.update(
            target(500, 100), FRAME, VehicleState("LOITER", True), now=2.3
        )
        self.assertIsNone(output.request_mode)
        self.assertEqual(self.handoff.confirm_count, 0)

    def test_target_loss_commands_hold(self):
        self.handoff.state = HandoffState.REQUESTING_GUIDED
        self.handoff.finish_mode_request(True, now=3.0)
        output = self.handoff.update(
            None, FRAME, VehicleState("GUIDED", True), now=4.0
        )
        self.assertEqual((output.command.vx, output.command.vy), (0.0, 0.0))
        self.assertEqual(output.command.vz, 0.0)
        self.assertIn("holding", output.status)

    def test_centered_target_descends_after_hold_time(self):
        self.handoff.state = HandoffState.REQUESTING_GUIDED
        self.handoff.finish_mode_request(True, now=3.0)
        vehicle = VehicleState("GUIDED", True, relative_alt_m=2.0)

        first = self.handoff.update(target(320, 240), FRAME, vehicle, now=3.1)
        later = self.handoff.update(target(320, 240), FRAME, vehicle, now=4.2)

        self.assertEqual(first.command.vz, 0.0)
        self.assertGreater(later.command.vz, 0.0)
        self.assertIn("approaching", later.status)

    def test_approach_stops_at_minimum_altitude(self):
        self.handoff.state = HandoffState.REQUESTING_GUIDED
        self.handoff.finish_mode_request(True, now=3.0)
        vehicle = VehicleState("GUIDED", True, relative_alt_m=0.7)

        self.handoff.update(target(320, 240), FRAME, vehicle, now=3.1)
        output = self.handoff.update(target(320, 240), FRAME, vehicle, now=4.2)

        self.assertEqual(output.command.vz, 0.0)
        self.assertIn("approach complete", output.status)


if __name__ == "__main__":
    unittest.main()
