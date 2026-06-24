#!/usr/bin/env python3
"""
Interactive spine bending command publisher.

Read a pre-generated spine-servo polynomial model JSON and publish servo6/servo7
target angles. After startup, type a desired spine bending angle in the console.
The program prints the computed target once, then returns to the prompt.

Key behavior:
  - No continuous debug/status printing by default.
  - The latest target is still published in the background at --rate Hz.
  - Default rate is 20 Hz.
  - Torque/load management is ON by default.
  - Default load center is 110 and deadband is 10, so the default acceptable
    abs(load) range is 100 to 120.
  - Default publish style is one ServoControlCmd containing both servo6 and servo7.
    This avoids the second message overwriting the first in downstream controllers.
  - Stop with Ctrl+C, q, quit, or exit.

Model JSON default:
  ./spine_servo_model.json

Usage:
  rosrun gryllo spine_model_interactive_publisher_quiet_combined.py
  rosrun gryllo spine_model_interactive_publisher_quiet_combined.py spine_servo_model.json

Console input:
  0.8          set desired spine angle to 0.8 rad
  angle 0.6    set desired spine angle to 0.6 rad
  status       print current state once
  reset        reset runtime offsets to 0
  q / quit     stop the program
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rospy
from spinal.msg import ServoControlCmd, ServoStates


SERVO_STATES_TOPIC = "/gryllo/servo/states"
SERVO_TARGET_TOPIC = "/gryllo/servo/target_states"

INITIAL_SERVO_OFFSETS = {
    6: 0.0,
    7: 0.0,
}

DEFAULT_MODEL_JSON = "spine_servo_model.json"
DEFAULT_RATE_HZ = 20.0
DEFAULT_TARGET_LOAD6 = 80.0
DEFAULT_TARGET_LOAD7 = 80.0
DEFAULT_DEADBAND = 20.0
DEFAULT_OFFSET_STEP = 3.0


@dataclass
class ServoModel:
    servo_id: int
    coeffs: List[float]
    json_offset: float = 0.0
    yaw_min: Optional[float] = None
    yaw_max: Optional[float] = None


@dataclass
class ServoFeedback:
    angle: float
    load: float


def polyval(coeffs: Sequence[float], x: float) -> float:
    y = 0.0
    for c in coeffs:
        y = y * x + float(c)
    return y


def clamp(value: float, vmin: Optional[float], vmax: Optional[float]) -> float:
    if vmin is not None:
        value = max(vmin, value)
    if vmax is not None:
        value = min(vmax, value)
    return value


def load_servo_model(model_json_path: str, key: str, fallback_servo_id: int) -> ServoModel:
    with open(model_json_path, "r", encoding="utf-8") as f:
        model = json.load(f)

    if key not in model:
        raise KeyError(f"Model JSON does not contain key '{key}'")

    data = model[key]
    coeffs = data.get("coeffs")
    if not coeffs:
        raise ValueError(f"Model JSON key '{key}' has no coeffs")

    return ServoModel(
        servo_id=int(data.get("id", fallback_servo_id)),
        coeffs=[float(c) for c in coeffs],
        json_offset=float(data.get("offset", 0.0)),
        yaw_min=None if data.get("yaw_min") is None else float(data.get("yaw_min")),
        yaw_max=None if data.get("yaw_max") is None else float(data.get("yaw_max")),
    )


def extract_servo_angle_load(msg: ServoStates, servo_id: int) -> Tuple[float, float]:
    if not hasattr(msg, "servos"):
        raise ValueError("ServoStates message has no 'servos' field")

    servos = list(msg.servos)
    candidates = []

    for servo in servos:
        if hasattr(servo, "index") and int(servo.index) == servo_id:
            candidates.append(servo)
            break

    if 0 <= servo_id < len(servos):
        candidates.append(servos[servo_id])

    for servo in candidates:
        if hasattr(servo, "angle") and hasattr(servo, "load"):
            return float(servo.angle), float(servo.load)

    reported_indices = [int(s.index) for s in servos if hasattr(s, "index")]
    raise IndexError(
        f"Could not find servo_id={servo_id} with angle/load. "
        f"array length={len(servos)}, reported indices={reported_indices}"
    )


class SpineModelInteractivePublisher:
    def __init__(
        self,
        model_json_path: str,
        initial_angle: Optional[float],
        rate_hz: float,
        torque_management: bool,
        allow_extrapolation: bool,
        min_angle: Optional[float],
        max_angle: Optional[float],
        target_load6: float,
        target_load7: float,
        offset_step: float,
        deadband: float,
        raw_load: bool,
        command_topic: str,
        state_topic: str,
        verbose: bool,
        separate_publish: bool,
    ) -> None:
        self.model6 = load_servo_model(model_json_path, "servo6", 6)
        self.model7 = load_servo_model(model_json_path, "servo7", 7)

        self.desired_spine_angle: Optional[float] = None if initial_angle is None else float(initial_angle)
        self.rate_hz = float(rate_hz)
        self.torque_management = bool(torque_management)
        self.allow_extrapolation = bool(allow_extrapolation)

        self.min_angle = None if min_angle is None else int(round(min_angle))
        self.max_angle = None if max_angle is None else int(round(max_angle))

        self.target_load = {
            6: float(target_load6),
            7: float(target_load7),
        }
        self.offset_step = abs(float(offset_step))
        self.deadband = abs(float(deadband))
        self.raw_load = bool(raw_load)
        self.verbose = bool(verbose)
        self.separate_publish = bool(separate_publish)

        self.runtime_offsets: Dict[int, float] = {
            6: 0.0,
            7: 0.0,
        }

        self.feedback: Dict[int, ServoFeedback] = {}
        self.last_servo6_cmd: Optional[int] = None
        self.last_servo7_cmd: Optional[int] = None
        self.lock = threading.Lock()

        self.pub = rospy.Publisher(command_topic, ServoControlCmd, queue_size=1)
        self.sub = rospy.Subscriber(state_topic, ServoStates, self.states_cb, queue_size=1)

    def states_cb(self, msg: ServoStates) -> None:
        for servo_id in (6, 7):
            try:
                angle, load = extract_servo_angle_load(msg, servo_id)
                with self.lock:
                    self.feedback[servo_id] = ServoFeedback(angle=angle, load=load)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "Failed to read servo%d feedback: %s", servo_id, str(e))

    def yaw_for_model(self, model: ServoModel) -> float:
        if self.desired_spine_angle is None:
            raise RuntimeError("desired_spine_angle is not set")

        if self.allow_extrapolation:
            return self.desired_spine_angle

        yaw = clamp(self.desired_spine_angle, model.yaw_min, model.yaw_max)
        if yaw != self.desired_spine_angle:
            rospy.logwarn_throttle(
                1.0,
                "Desired spine angle %.6f is outside servo%d model range [%.6f, %.6f]. "
                "Using clamped yaw %.6f. Add --allow-extrapolation to disable clamp.",
                self.desired_spine_angle,
                model.servo_id,
                float("nan") if model.yaw_min is None else model.yaw_min,
                float("nan") if model.yaw_max is None else model.yaw_max,
                yaw,
            )
        return yaw

    def base_angle_from_model(self, model: ServoModel) -> float:
        yaw = self.yaw_for_model(model)
        return polyval(model.coeffs, yaw)

    def managed_load_value(self, load: float) -> float:
        return load if self.raw_load else abs(load)

    def load_bounds(self, servo_id: int) -> Tuple[float, float]:
        target = self.target_load[servo_id]
        return target - self.deadband, target + self.deadband

    def update_offsets_from_load_locked(self) -> None:
        if not self.torque_management or self.desired_spine_angle is None:
            return

        for servo_id in (6, 7):
            fb = self.feedback.get(servo_id)
            if fb is None:
                continue

            measured = self.managed_load_value(fb.load)
            lower, upper = self.load_bounds(servo_id)

            if measured < lower:
                self.runtime_offsets[servo_id] -= self.offset_step
                if self.verbose:
                    rospy.loginfo(
                        "servo%d load LOW: raw_load=%.3f managed_load=%.3f range=[%.3f, %.3f], offset=%.3f",
                        servo_id, fb.load, measured, lower, upper, self.runtime_offsets[servo_id]
                    )
            elif measured > upper:
                self.runtime_offsets[servo_id] += self.offset_step
                if self.verbose:
                    rospy.loginfo(
                        "servo%d load HIGH: raw_load=%.3f managed_load=%.3f range=[%.3f, %.3f], offset=%.3f",
                        servo_id, fb.load, measured, lower, upper, self.runtime_offsets[servo_id]
                    )

    def clamp_command_angle(self, angle: float) -> int:
        angle_i = int(round(angle))
        if self.min_angle is not None:
            angle_i = max(self.min_angle, angle_i)
        if self.max_angle is not None:
            angle_i = min(self.max_angle, angle_i)
        return int(angle_i)

    def compute_command_angles_locked(self) -> Tuple[int, int]:
        base6 = self.base_angle_from_model(self.model6)
        base7 = self.base_angle_from_model(self.model7)

        cmd6 = base6 + self.model6.json_offset + INITIAL_SERVO_OFFSETS[6] + self.runtime_offsets[6]
        cmd7 = base7 + self.model7.json_offset + INITIAL_SERVO_OFFSETS[7] + self.runtime_offsets[7]

        return self.clamp_command_angle(cmd6), self.clamp_command_angle(cmd7)

    def publish_one_servo(self, servo_id: int, angle: int) -> None:
        cmd = ServoControlCmd()
        cmd.index = [int(servo_id)]
        cmd.angles = [int(angle)]
        self.pub.publish(cmd)

    def publish_commands(self, servo6_angle: int, servo7_angle: int) -> None:
        if self.separate_publish:
            self.publish_one_servo(6, servo6_angle)
            self.publish_one_servo(7, servo7_angle)
        else:
            # Publish both servos in one message.
            # Some downstream controllers only apply the latest message per cycle,
            # so two consecutive one-servo messages can make only servo7 move.
            cmd = ServoControlCmd()
            cmd.index = [6, 7]
            cmd.angles = [int(servo6_angle), int(servo7_angle)]
            self.pub.publish(cmd)

    def step_once(self) -> None:
        with self.lock:
            if self.desired_spine_angle is None:
                return

            self.update_offsets_from_load_locked()
            servo6_angle, servo7_angle = self.compute_command_angles_locked()
            self.last_servo6_cmd = servo6_angle
            self.last_servo7_cmd = servo7_angle

        self.publish_commands(servo6_angle, servo7_angle)

    def build_status_line_locked(self) -> str:
        if self.desired_spine_angle is None:
            return "No desired spine angle is set."

        fb6 = self.feedback.get(6)
        fb7 = self.feedback.get(7)

        load6 = None if fb6 is None else fb6.load
        load7 = None if fb7 is None else fb7.load
        fb_angle6 = None if fb6 is None else fb6.angle
        fb_angle7 = None if fb7 is None else fb7.angle

        lower6, upper6 = self.load_bounds(6)
        lower7, upper7 = self.load_bounds(7)

        managed6 = None if load6 is None else self.managed_load_value(load6)
        managed7 = None if load7 is None else self.managed_load_value(load7)

        return (
            "spine={:.6f} rad | "
            "target_servo6={} target_servo7={} | "
            "offset6={:.3f} offset7={:.3f} | "
            "fb_angle6={} fb_angle7={} | "
            "load6={} load7={} | "
            "managed_load6={} range6=[{:.1f},{:.1f}] | "
            "managed_load7={} range7=[{:.1f},{:.1f}]"
        ).format(
            self.desired_spine_angle,
            self.last_servo6_cmd,
            self.last_servo7_cmd,
            self.runtime_offsets[6],
            self.runtime_offsets[7],
            "None" if fb_angle6 is None else f"{fb_angle6:.3f}",
            "None" if fb_angle7 is None else f"{fb_angle7:.3f}",
            "None" if load6 is None else f"{load6:.3f}",
            "None" if load7 is None else f"{load7:.3f}",
            "None" if managed6 is None else f"{managed6:.3f}",
            lower6,
            upper6,
            "None" if managed7 is None else f"{managed7:.3f}",
            lower7,
            upper7,
        )

    def print_status(self) -> None:
        with self.lock:
            print(self.build_status_line_locked(), flush=True)

    def set_desired_angle(self, angle: float) -> None:
        with self.lock:
            self.desired_spine_angle = float(angle)
            servo6_angle, servo7_angle = self.compute_command_angles_locked()
            self.last_servo6_cmd = servo6_angle
            self.last_servo7_cmd = servo7_angle
            status = self.build_status_line_locked()

        self.publish_commands(servo6_angle, servo7_angle)
        print(status, flush=True)

    def reset_offsets(self) -> None:
        with self.lock:
            self.runtime_offsets[6] = 0.0
            self.runtime_offsets[7] = 0.0
            current = self.desired_spine_angle

        print("Runtime offsets reset to zero.", flush=True)
        if current is not None:
            self.set_desired_angle(current)

    def publish_loop(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step_once()
            rate.sleep()

    def handle_console_command(self, line: str) -> bool:
        line = line.strip()
        if not line:
            return True

        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            return False

        if cmd in ("status", "s"):
            self.print_status()
            return True

        if cmd in ("reset", "reset-offset", "reset-offsets"):
            self.reset_offsets()
            return True

        if cmd in ("angle", "yaw", "spine"):
            if len(parts) != 2:
                print("Usage: angle <desired_spine_angle_rad>", flush=True)
                return True
            value_text = parts[1]
        else:
            value_text = line

        try:
            value = float(value_text)
        except ValueError:
            print("Invalid input. Type a number such as 0.8, or commands: status, reset, quit.", flush=True)
            return True

        self.set_desired_angle(value)
        return True

    def run(self) -> None:
        lower6, upper6 = self.load_bounds(6)
        lower7, upper7 = self.load_bounds(7)

        rospy.loginfo(
            "Loaded models. initial_spine_angle=%s, rate=%.1f Hz, torque_management=%s",
            "None" if self.desired_spine_angle is None else f"{self.desired_spine_angle:.6f}",
            self.rate_hz,
            str(self.torque_management),
        )
        rospy.loginfo(
            "Load control: servo6 target=%.3f deadband=%.3f range=[%.3f, %.3f], "
            "servo7 target=%.3f deadband=%.3f range=[%.3f, %.3f], raw_load=%s",
            self.target_load[6],
            self.deadband,
            lower6,
            upper6,
            self.target_load[7],
            self.deadband,
            lower7,
            upper7,
            str(self.raw_load),
        )

        rospy.sleep(0.2)

        if self.desired_spine_angle is not None:
            self.set_desired_angle(self.desired_spine_angle)

        thread = threading.Thread(target=self.publish_loop, daemon=True)
        thread.start()

        print("", flush=True)
        print("Interactive spine command mode.", flush=True)
        print("Type a desired spine angle in rad, e.g. 0.8, then press Enter.", flush=True)
        print("Other commands: status, reset, quit", flush=True)
        print("Stop with Ctrl+C.", flush=True)
        print("", flush=True)

        while not rospy.is_shutdown():
            try:
                line = input("> ")
            except EOFError:
                break
            except KeyboardInterrupt:
                break

            keep_running = self.handle_console_command(line)
            if not keep_running:
                break

        rospy.signal_shutdown("console exit")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively publish servo6/servo7 target angles from a pre-generated spine-servo JSON model."
    )
    parser.add_argument(
        "model_json",
        nargs="?",
        default=DEFAULT_MODEL_JSON,
        help=f"Path to pre-generated model JSON. Default: ./{DEFAULT_MODEL_JSON}",
    )
    parser.add_argument(
        "--initial-angle",
        type=float,
        default=None,
        help="Initial desired spine bending angle [rad]. If omitted, wait for console input.",
    )

    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="Publish rate [Hz]. Default: 20")
    parser.add_argument("--disable-torque-management", action="store_true",
                        help="Disable automatic runtime offset update based on servo load")
    parser.add_argument("--allow-extrapolation", action="store_true",
                        help="Do not clamp desired_spine_angle to yaw_min/yaw_max stored in JSON")

    parser.add_argument("--min-angle", type=float, default=None, help="Lower limit for servo command angle")
    parser.add_argument("--max-angle", type=float, default=None, help="Upper limit for servo command angle")

    parser.add_argument("--target-load6", type=float, default=DEFAULT_TARGET_LOAD6,
                        help="Target load center for servo6. Default: 110")
    parser.add_argument("--target-load7", type=float, default=DEFAULT_TARGET_LOAD7,
                        help="Target load center for servo7. Default: 110")
    parser.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND,
                        help="Allowed load half-width around target. Default: 10")
    parser.add_argument("--offset-step", type=float, default=DEFAULT_OFFSET_STEP,
                        help="Offset update per cycle when load is outside threshold range. Default: 1")
    parser.add_argument("--raw-load", action="store_true",
                        help="Use signed raw load instead of abs(load) for load management")
    parser.add_argument("--verbose", action="store_true",
                        help="Print torque/load offset update logs. Default: off")
    parser.add_argument("--separate-publish", action="store_true",
                        help="Publish servo6 and servo7 as two separate ServoControlCmd messages. Default: publish both in one message.")

    parser.add_argument("--state-topic", default=SERVO_STATES_TOPIC,
                        help=f"Servo state topic. Default: {SERVO_STATES_TOPIC}")
    parser.add_argument("--command-topic", default=SERVO_TARGET_TOPIC,
                        help=f"Servo command topic. Default: {SERVO_TARGET_TOPIC}")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.rate <= 0:
        print("ERROR: --rate must be positive", file=sys.stderr)
        return 2
    if args.offset_step < 0:
        print("ERROR: --offset-step must be non-negative", file=sys.stderr)
        return 2
    if args.target_load6 < 0 or args.target_load7 < 0:
        print("ERROR: target loads must be non-negative", file=sys.stderr)
        return 2
    if args.deadband < 0:
        print("ERROR: --deadband must be non-negative", file=sys.stderr)
        return 2
    if args.min_angle is not None and args.max_angle is not None and args.min_angle > args.max_angle:
        print("ERROR: --min-angle must be <= --max-angle", file=sys.stderr)
        return 2

    rospy.init_node("spine_model_interactive_publisher", anonymous=True)

    node = SpineModelInteractivePublisher(
        model_json_path=args.model_json,
        initial_angle=args.initial_angle,
        rate_hz=args.rate,
        torque_management=not args.disable_torque_management,
        allow_extrapolation=args.allow_extrapolation,
        min_angle=args.min_angle,
        max_angle=args.max_angle,
        target_load6=args.target_load6,
        target_load7=args.target_load7,
        offset_step=args.offset_step,
        deadband=args.deadband,
        raw_load=args.raw_load,
        command_topic=args.command_topic,
        state_topic=args.state_topic,
        verbose=args.verbose,
        separate_publish=args.separate_publish,
    )

    try:
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by Ctrl+C.")
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
