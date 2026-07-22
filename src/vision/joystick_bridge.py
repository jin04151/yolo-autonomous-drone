#!/usr/bin/env python3
import argparse
import os
import struct
import sys
import time
from pathlib import Path


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


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
        raise SystemExit("pymavlink not found. Check ~/venv-ardupilot or install pymavlink.")


def parse_axis_map(value):
    result = {}
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        name, axis = item.split(":", 1)
        result[name.strip()] = int(axis)
    required = {"roll", "pitch", "throttle", "yaw"}
    missing = required - set(result)
    if missing:
        raise argparse.ArgumentTypeError(f"missing axis map entries: {', '.join(sorted(missing))}")
    return result


def parse_reverse(value):
    return {item.strip() for item in value.split(",") if item.strip()}


def parse_args():
    parser = argparse.ArgumentParser(description="Read /dev/input/js0 and send MAVLink RC override.")
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--mavlink", default="tcp:127.0.0.1:5773")
    parser.add_argument(
        "--axis-map",
        type=parse_axis_map,
        default=parse_axis_map("roll:0,pitch:1,throttle:2,yaw:3"),
        help="axis mapping, for example roll:0,pitch:1,throttle:2,yaw:3",
    )
    parser.add_argument(
        "--reverse",
        type=parse_reverse,
        default=parse_reverse(""),
        help="comma-separated controls to reverse: roll,pitch,throttle,yaw",
    )
    parser.add_argument("--rate", type=float, default=20.0, help="MAVLink send rate in Hz")
    parser.add_argument("--deadband", type=float, default=0.04, help="axis deadband after normalization")
    parser.add_argument("--min-pwm", type=int, default=1100)
    parser.add_argument("--mid-pwm", type=int, default=1500)
    parser.add_argument("--max-pwm", type=int, default=1900)
    parser.add_argument("--source-system", type=int, default=255, help="MAVLink source system id for RC override")
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true", help="print RC values without connecting MAVLink")
    return parser.parse_args()


def normalize(value, deadband):
    norm = max(-1.0, min(1.0, value / 32767.0))
    return 0.0 if abs(norm) < deadband else norm


def axis_to_pwm(norm, min_pwm, mid_pwm, max_pwm):
    if norm >= 0:
        return int(round(mid_pwm + norm * (max_pwm - mid_pwm)))
    return int(round(mid_pwm + norm * (mid_pwm - min_pwm)))


def throttle_to_pwm(norm, min_pwm, max_pwm):
    return int(round(min_pwm + ((norm + 1.0) * 0.5) * (max_pwm - min_pwm)))


def build_rc(axis_values, args):
    def control_value(name):
        axis = args.axis_map[name]
        value = normalize(axis_values.get(axis, 0), args.deadband)
        if name in args.reverse:
            value = -value
        return value

    roll = axis_to_pwm(control_value("roll"), args.min_pwm, args.mid_pwm, args.max_pwm)
    pitch = axis_to_pwm(control_value("pitch"), args.min_pwm, args.mid_pwm, args.max_pwm)
    throttle = throttle_to_pwm(control_value("throttle"), args.min_pwm, args.max_pwm)
    yaw = axis_to_pwm(control_value("yaw"), args.min_pwm, args.mid_pwm, args.max_pwm)
    return roll, pitch, throttle, yaw


def connect_mavlink(connection_string, source_system):
    mavutil = load_mavutil()
    print(f"Connecting MAVLink: {connection_string}", flush=True)
    master = mavutil.mavlink_connection(
        connection_string,
        autoreconnect=True,
        source_system=source_system,
    )
    heartbeat = master.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise SystemExit(
            "No MAVLink heartbeat. Start sitl-my-drone first, or run in MAVProxy: "
            "output add tcpin:127.0.0.1:5773"
        )
    print(
        f"Heartbeat: system={master.target_system} component={master.target_component} "
        f"mode={mavutil.mode_string_v10(heartbeat)}",
        flush=True,
    )
    return mavutil, master


def send_override(master, rc):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        rc[0],
        rc[1],
        rc[2],
        rc[3],
        0,
        0,
        0,
        0,
    )


def release_override(master):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def main():
    args = parse_args()
    if not os.path.exists(args.device):
        raise SystemExit(f"Joystick device not found: {args.device}")

    mavutil = master = None
    if not args.dry_run:
        mavutil, master = connect_mavlink(args.mavlink, args.source_system)

    axis_values = {}
    period = 1.0 / max(args.rate, 1.0)
    last_send = 0.0
    last_print = 0.0
    last_rc = (args.mid_pwm, args.mid_pwm, args.min_pwm, args.mid_pwm)

    print(
        f"Reading {args.device}. Axis map={args.axis_map}, reverse={sorted(args.reverse)}. "
        "Ctrl+C to stop.",
        flush=True,
    )

    try:
        fd = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK)
        with os.fdopen(fd, "rb", buffering=0) as joystick:
            while True:
                try:
                    data = joystick.read(8)
                except BlockingIOError:
                    data = None

                if data and len(data) == 8:
                    _timestamp, value, event_type, number = struct.unpack("IhBB", data)
                    event_type &= ~JS_EVENT_INIT
                    if event_type == JS_EVENT_AXIS:
                        axis_values[number] = value

                now = time.time()
                if now - last_send >= period:
                    last_send = now
                    last_rc = build_rc(axis_values, args)
                    if not args.dry_run:
                        send_override(master, last_rc)

                if now - last_print >= args.print_every:
                    last_print = now
                    print(
                        f"rc1={last_rc[0]} rc2={last_rc[1]} rc3={last_rc[2]} rc4={last_rc[3]} "
                        f"axes={dict(sorted(axis_values.items()))}",
                        flush=True,
                    )

                time.sleep(0.005)
    except KeyboardInterrupt:
        print("Stopping joystick bridge", flush=True)
    finally:
        if master is not None:
            release_override(master)
            print("Released RC override", flush=True)


if __name__ == "__main__":
    main()
