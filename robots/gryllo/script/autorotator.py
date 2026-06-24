#!/usr/bin/env python3
"""
Wind one servo in the decreasing-angle direction until its load reaches a target,
then keep holding and monitoring until Ctrl+C.

Usage:
  python3 autorotator.py <servo_id> <target_load>

Examples:
  python3 autorotator.py 6 20
  python3 autorotator.py 6 20 --step 10 --rate 20
  python3 autorotator.py 7 80 --step 5 --rate 20 --min-angle 0

ROS topics:
  Subscribe: /gryllo/servo/states
  Publish  : /gryllo/servo/target_states

Command message:
  spinal/ServoControlCmd

Behavior:
  - Reads the current angle and load of the specified servo.
  - If load < target_load, decreases the commanded angle by --step.
  - If load >= target_load, does NOT exit.
  - While load >= target_load, keeps publishing the current command angle.
  - If load drops below target_load again, resumes winding in the decreasing-angle direction.
  - It never increases the angle, so reverse rotation is not performed.
  - Program continues until Ctrl+C or ROS shutdown.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

import rospy
from spinal.msg import ServoControlCmd, ServoStates


SERVO_STATES_TOPIC = "/gryllo/servo/states"
SERVO_TARGET_TOPIC = "/gryllo/servo/target_states"


@dataclass
class ServoFeedback:
    angle: float
    load: float


class ServoLoadWinder:
    def __init__(
        self,
        servo_id: int,
        target_load: float,
        step: float,
        rate_hz: float,
        min_angle: Optional[float],
        max_angle: Optional[float],
        load_deadband: float,
        command_topic: str,
        state_topic: str,
    ) -> None:
        self.servo_id = int(servo_id)
        self.target_load = float(target_load)
        self.step = int(round(abs(step)))
        self.rate_hz = float(rate_hz)
        self.min_angle = None if min_angle is None else int(round(min_angle))
        self.max_angle = None if max_angle is None else int(round(max_angle))
        self.load_deadband = abs(float(load_deadband))

        if self.step <= 0:
            raise ValueError("--step must be positive after rounding to int")

        self.feedback: Optional[ServoFeedback] = None
        self.command_angle: Optional[int] = None

        self.was_holding = False
        self.limit_reached = False

        self.pub = rospy.Publisher(command_topic, ServoControlCmd, queue_size=1)
        self.sub = rospy.Subscriber(state_topic, ServoStates, self.states_cb, queue_size=1)

    def states_cb(self, msg: ServoStates) -> None:
        try:
            angle, load = self.extract_servo_angle_load(msg, self.servo_id)
        except Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "Failed to read servo %d: %s",
                self.servo_id,
                str(e),
            )
            return

        self.feedback = ServoFeedback(angle=angle, load=load)

        # Initialize command from the measured angle so the first command is continuous.
        if self.command_angle is None:
            self.command_angle = int(round(angle))
            rospy.loginfo(
                "Initialized servo%d command angle from feedback: angle=%.3f, command_angle=%d, load=%.3f",
                self.servo_id,
                angle,
                self.command_angle,
                load,
            )

    @staticmethod
    def extract_servo_angle_load(msg: ServoStates, servo_id: int) -> Tuple[float, float]:
        """
        Extract angle/load from ServoStates.

        First tries msg.servos[servo_id], then falls back to searching a servo whose
        .index field equals servo_id. This handles both array-index based and
        ID-field based drivers/logs.
        """
        if not hasattr(msg, "servos"):
            raise ValueError("ServoStates message has no 'servos' field")

        servos = list(msg.servos)

        candidates = []

        # Prefer .index match if present.
        for servo in servos:
            if hasattr(servo, "index") and int(servo.index) == servo_id:
                candidates.append(servo)
                break

        # Fallback: use servo_id as array index.
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

    def clamp_angle(self, angle: int) -> int:
        if self.min_angle is not None:
            angle = max(self.min_angle, angle)
        if self.max_angle is not None:
            angle = min(self.max_angle, angle)
        return int(angle)

    def publish_angle(self, angle: int) -> None:
        """
        Publish ServoControlCmd.

        Important:
          spinal/ServoControlCmd.angles[] is an integer array.
          Therefore cmd.angles must contain int, not float.
        """
        angle_i = int(round(angle))

        cmd = ServoControlCmd()
        cmd.index = [int(self.servo_id)]
        cmd.angles = [int(angle_i)]
        self.pub.publish(cmd)

    def step_once(self) -> None:
        """
        Run one control step.

        This function never requests program termination.
        The outer loop continues until Ctrl+C / rospy shutdown.
        """
        if self.feedback is None or self.command_angle is None:
            return

        current_load = self.feedback.load
        current_angle = self.feedback.angle

        # If already at or above target load, hold current command and keep monitoring.
        if abs(current_load) >= self.target_load - self.load_deadband:
            self.publish_angle(self.command_angle)

            if not self.was_holding:
                rospy.loginfo(
                    "Target load reached. Continue holding and monitoring. "
                    "servo%d load=%.3f target=%.3f command_angle=%d feedback_angle=%.3f",
                    self.servo_id,
                    current_load,
                    self.target_load,
                    self.command_angle,
                    current_angle,
                )
                self.was_holding = True

            rospy.loginfo_throttle(
                1.0,
                "Holding servo%d: load=%.3f/%.3f, feedback_angle=%.3f, command_angle=%d",
                self.servo_id,
                current_load,
                self.target_load,
                current_angle,
                self.command_angle,
            )
            return

        # Load is below target. Resume winding if we were holding.
        if self.was_holding:
            rospy.loginfo(
                "Load dropped below target. Resume winding servo%d. "
                "load=%.3f target=%.3f command_angle=%d feedback_angle=%.3f",
                self.servo_id,
                current_load,
                self.target_load,
                self.command_angle,
                current_angle,
            )
            self.was_holding = False

        # If angle limit was already reached, keep holding the limit command and monitor.
        if self.limit_reached:
            self.publish_angle(self.command_angle)
            rospy.logwarn_throttle(
                1.0,
                "Angle limit already reached. Holding servo%d: load=%.3f target=%.3f command_angle=%d",
                self.servo_id,
                current_load,
                self.target_load,
                self.command_angle,
            )
            return

        next_angle = self.clamp_angle(int(self.command_angle) - int(self.step))

        # If clamp prevents further decrease, hold but do not exit.
        if next_angle == self.command_angle:
            self.limit_reached = True
            self.publish_angle(self.command_angle)
            rospy.logwarn(
                "Angle limit reached before target load. Continue holding and monitoring. "
                "servo%d load=%.3f target=%.3f command_angle=%d",
                self.servo_id,
                current_load,
                self.target_load,
                self.command_angle,
            )
            return

        self.command_angle = int(next_angle)
        self.publish_angle(self.command_angle)

        rospy.loginfo_throttle(
            0.5,
            "Winding servo%d: load=%.3f/%.3f, feedback_angle=%.3f, command_angle=%d",
            self.servo_id,
            current_load,
            self.target_load,
            current_angle,
            self.command_angle,
        )

    def run(self) -> None:
        rospy.loginfo("Waiting for servo feedback on %s ...", SERVO_STATES_TOPIC)
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown() and self.feedback is None:
            rate.sleep()

        rospy.loginfo(
            "Start continuous winding/holding servo%d with target load >= %.3f. "
            "step=%d, rate=%.1f Hz",
            self.servo_id,
            self.target_load,
            self.step,
            self.rate_hz,
        )

        while not rospy.is_shutdown():
            self.step_once()
            rate.sleep()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decrease one servo angle until its load reaches the target, then keep holding."
    )
    parser.add_argument("servo_id", type=int, help="Servo ID/index to command")
    parser.add_argument(
        "target_load",
        type=float,
        help="Target load. Winding is active while load < this value.",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=1.0,
        help="Angle decrement per command step. Rounded to int. Default: 1.0",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Command update rate [Hz]. Default: 10",
    )
    parser.add_argument(
        "--min-angle",
        type=float,
        default=None,
        help="Lower command angle limit",
    )
    parser.add_argument(
        "--max-angle",
        type=float,
        default=None,
        help="Upper command angle limit",
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=0.0,
        help="Load deadband around target. Default: 0",
    )
    parser.add_argument(
        "--state-topic",
        default=SERVO_STATES_TOPIC,
        help=f"Servo state topic. Default: {SERVO_STATES_TOPIC}",
    )
    parser.add_argument(
        "--command-topic",
        default=SERVO_TARGET_TOPIC,
        help=f"Servo command topic. Default: {SERVO_TARGET_TOPIC}",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.step <= 0:
        print("ERROR: --step must be positive", file=sys.stderr)
        return 2

    if args.rate <= 0:
        print("ERROR: --rate must be positive", file=sys.stderr)
        return 2

    if (
        args.min_angle is not None
        and args.max_angle is not None
        and args.min_angle > args.max_angle
    ):
        print("ERROR: --min-angle must be <= --max-angle", file=sys.stderr)
        return 2

    rospy.init_node(f"wind_servo_{args.servo_id}_until_load", anonymous=True)

    node = ServoLoadWinder(
        servo_id=args.servo_id,
        target_load=args.target_load,
        step=args.step,
        rate_hz=args.rate,
        min_angle=args.min_angle,
        max_angle=args.max_angle,
        load_deadband=args.deadband,
        command_topic=args.command_topic,
        state_topic=args.state_topic,
    )

    node.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
