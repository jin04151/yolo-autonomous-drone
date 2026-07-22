#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path


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
        raise SystemExit("pymavlink not found. Start from the existing WSL setup or install pymavlink.")


def parse_args():
    parser = argparse.ArgumentParser(description="Set GUIDED, arm, and take off through MAVLink.")
    parser.add_argument("--mavlink", default="tcp:127.0.0.1:5772")
    parser.add_argument("--alt", type=float, default=2.0)
    parser.add_argument("--mode-timeout", type=float, default=8.0)
    parser.add_argument("--arm-timeout", type=float, default=10.0)
    parser.add_argument("--takeoff-timeout", type=float, default=25.0)
    return parser.parse_args()


class TakeoffClient:
    def __init__(self, connection_string):
        self.mavutil = load_mavutil()
        print(f"Connecting MAVLink: {connection_string}", flush=True)
        self.master = self.mavutil.mavlink_connection(
            connection_string,
            autoreconnect=True,
            source_system=251,
        )
        heartbeat = self.master.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise SystemExit(
                "No MAVLink heartbeat. Start sitl-my-drone first, or run in MAVProxy: "
                "output add tcpin:127.0.0.1:5772"
            )
        print(
            f"Heartbeat: system={self.master.target_system} "
            f"component={self.master.target_component} "
            f"mode={self.mavutil.mode_string_v10(heartbeat)}",
            flush=True,
        )

    def set_mode(self, mode, timeout):
        mode_mapping = self.master.mode_mapping()
        if mode not in mode_mapping:
            raise SystemExit(f"Flight mode {mode} is not available")

        print(f"Setting mode {mode}...", flush=True)
        self.master.mav.set_mode_send(
            self.master.target_system,
            self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode],
        )

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and self.mavutil.mode_string_v10(msg) == mode:
                print(f"Mode {mode}", flush=True)
                return
        raise SystemExit(f"Timed out waiting for mode {mode}")

    def arm(self, timeout):
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


def main():
    args = parse_args()
    client = TakeoffClient(args.mavlink)
    client.set_mode("GUIDED", args.mode_timeout)
    client.arm(args.arm_timeout)
    client.takeoff(args.alt)
    client.wait_altitude(args.alt, args.takeoff_timeout)
    print("Ready for manual control. Switch to ALT_HOLD/LOITER when needed.", flush=True)


if __name__ == "__main__":
    main()
