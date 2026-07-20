#!/usr/bin/env python3

import json
import threading

import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoControlCmd, ServoStates


SPINE_JOINT_NAMES = ["spine_joint_{}".format(i) for i in range(1, 7)]


def polyval(coeffs, value):
    result = 0.0
    for coefficient in coeffs:
        result = result * value + float(coefficient)
    return result


class GrylloRealSpineBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.spine_joint_positions = [0.0] * len(SPINE_JOINT_NAMES)
        self.total_spine_angle = None
        self.servo_feedback = {}
        self.runtime_offsets = {6: 0.0, 7: 0.0}

        self.model = self._load_model(rospy.get_param("~model_json"))
        self.physical_joint_names = self._load_physical_joint_names()

        self.tendon_servo_ids = (self.model["servo6"]["id"], self.model["servo7"]["id"])
        self.tendon_command_rate = float(rospy.get_param("~tendon_command_rate", 20.0))
        self.torque_management = bool(rospy.get_param("~torque_management", True))
        self.target_load = {
            self.tendon_servo_ids[0]: float(rospy.get_param("~target_load6", 80.0)),
            self.tendon_servo_ids[1]: float(rospy.get_param("~target_load7", 80.0)),
        }
        self.load_deadband = abs(float(rospy.get_param("~load_deadband", 20.0)))
        self.offset_step = abs(float(rospy.get_param("~offset_step", 3.0)))
        self.use_raw_load = bool(rospy.get_param("~use_raw_load", False))

        if self.tendon_command_rate <= 0.0:
            raise ValueError("~tendon_command_rate must be positive")

        self.physical_command_pub = rospy.Publisher(
            "physical_gimbals_ctrl", JointState, queue_size=1
        )
        self.tendon_command_pub = rospy.Publisher(
            "servo/target_states", ServoControlCmd, queue_size=1
        )
        self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

        self.gimbals_command_sub = rospy.Subscriber(
            "gimbals_ctrl", JointState, self.gimbals_command_callback, queue_size=1
        )
        self.physical_joint_states_sub = rospy.Subscriber(
            "servo_joint_states", JointState, self.physical_joint_states_callback, queue_size=1
        )
        self.servo_states_sub = rospy.Subscriber(
            "servo/states", ServoStates, self.servo_states_callback, queue_size=1
        )

        self.tendon_command_timer = rospy.Timer(
            rospy.Duration(1.0 / self.tendon_command_rate), self.publish_tendon_command
        )

        rospy.loginfo(
            "Gryllo real spine bridge started: physical joints=%s, tendon servo ids=%s",
            sorted(self.physical_joint_names),
            self.tendon_servo_ids,
        )

    @staticmethod
    def _load_model(model_json_path):
        with open(model_json_path, "r") as model_file:
            raw_model = json.load(model_file)

        result = {}
        for key, fallback_id in (("servo6", 6), ("servo7", 7)):
            if key not in raw_model or not raw_model[key].get("coeffs"):
                raise ValueError("Invalid spine servo model for {}".format(key))
            data = raw_model[key]
            result[key] = {
                "id": int(data.get("id", fallback_id)),
                "coeffs": [float(value) for value in data["coeffs"]],
                "offset": float(data.get("offset", 0.0)),
                "yaw_min": None if data.get("yaw_min") is None else float(data["yaw_min"]),
                "yaw_max": None if data.get("yaw_max") is None else float(data["yaw_max"]),
            }
        return result

    @staticmethod
    def _load_physical_joint_names():
        servo_controller = rospy.get_param("servo_controller")
        names = set()
        for group in servo_controller.values():
            if not isinstance(group, dict):
                continue
            for key, controller in group.items():
                if "controller" not in key or not isinstance(controller, dict):
                    continue
                if "name" in controller:
                    names.add(str(controller["name"]))
        if not names:
            raise ValueError("No physical joints found in servo_controller parameters")
        return names

    @staticmethod
    def _copy_selected_command(source, selected_indices):
        output = JointState()
        output.header = source.header
        output.name = [source.name[index] for index in selected_indices]
        output.position = [source.position[index] for index in selected_indices]
        if len(source.velocity) == len(source.name):
            output.velocity = [source.velocity[index] for index in selected_indices]
        if len(source.effort) == len(source.name):
            output.effort = [source.effort[index] for index in selected_indices]
        return output

    def gimbals_command_callback(self, message):
        if len(message.position) != len(message.name):
            rospy.logerr_throttle(
                1.0,
                "Gryllo real spine bridge: gimbals_ctrl name/position size mismatch (%d/%d)",
                len(message.name),
                len(message.position),
            )
            return

        physical_indices = [
            index for index, name in enumerate(message.name)
            if name in self.physical_joint_names
        ]
        if physical_indices:
            self.physical_command_pub.publish(
                self._copy_selected_command(message, physical_indices)
            )

        command_by_name = dict(zip(message.name, message.position))
        if "spine_joint_1" not in command_by_name:
            return

        reference_angle = float(command_by_name["spine_joint_1"])
        positions = [
            float(command_by_name.get(name, reference_angle))
            for name in SPINE_JOINT_NAMES
        ]
        max_difference = max(abs(position - reference_angle) for position in positions)
        if max_difference > 1.0e-6:
            rospy.logwarn_throttle(
                1.0,
                "Spine joint commands are not equal; servo6/7 use 6 * spine_joint_1",
            )

        with self.lock:
            self.spine_joint_positions = positions
            self.total_spine_angle = 6.0 * reference_angle

    def physical_joint_states_callback(self, message):
        output = JointState()
        output.header = message.header

        for index, name in enumerate(message.name):
            if index >= len(message.position) or name in SPINE_JOINT_NAMES:
                continue
            output.name.append(name)
            output.position.append(message.position[index])

        with self.lock:
            spine_positions = list(self.spine_joint_positions)

        output.name.extend(SPINE_JOINT_NAMES)
        output.position.extend(spine_positions)
        self.joint_states_pub.publish(output)

    def servo_states_callback(self, message):
        wanted_ids = set(self.tendon_servo_ids)
        feedback = {}
        for servo in message.servos:
            servo_id = int(servo.index)
            if servo_id in wanted_ids:
                feedback[servo_id] = (float(servo.angle), float(servo.load))
        with self.lock:
            self.servo_feedback.update(feedback)

    @staticmethod
    def _clamp(value, lower, upper):
        if lower is not None:
            value = max(lower, value)
        if upper is not None:
            value = min(upper, value)
        return value

    def _update_runtime_offsets(self):
        if not self.torque_management:
            return
        for servo_id in self.tendon_servo_ids:
            if servo_id not in self.servo_feedback:
                continue
            load = self.servo_feedback[servo_id][1]
            measured_load = load if self.use_raw_load else abs(load)
            lower = self.target_load[servo_id] - self.load_deadband
            upper = self.target_load[servo_id] + self.load_deadband
            if measured_load < lower:
                self.runtime_offsets[servo_id] -= self.offset_step
            elif measured_load > upper:
                self.runtime_offsets[servo_id] += self.offset_step

    def _command_for_model(self, model, total_spine_angle):
        clamped_angle = self._clamp(
            total_spine_angle, model["yaw_min"], model["yaw_max"]
        )
        if clamped_angle != total_spine_angle:
            rospy.logwarn_throttle(
                1.0,
                "Total spine angle %.6f rad is outside servo%d model range; clamped to %.6f rad",
                total_spine_angle,
                model["id"],
                clamped_angle,
            )
        command = (
            polyval(model["coeffs"], clamped_angle)
            + model["offset"]
            + self.runtime_offsets[model["id"]]
        )
        return int(round(command))

    def publish_tendon_command(self, _event):
        with self.lock:
            if self.total_spine_angle is None:
                return
            self._update_runtime_offsets()
            servo6_command = self._command_for_model(
                self.model["servo6"], self.total_spine_angle
            )
            servo7_command = self._command_for_model(
                self.model["servo7"], self.total_spine_angle
            )

        command = ServoControlCmd()
        command.index = [self.tendon_servo_ids[0], self.tendon_servo_ids[1]]
        command.angles = [servo6_command, servo7_command]
        self.tendon_command_pub.publish(command)


def main():
    rospy.init_node("gryllo_real_spine_bridge")
    GrylloRealSpineBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
