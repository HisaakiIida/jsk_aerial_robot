#!/usr/bin/env python3
import rospy
from spinal.srv import SetDirectServoConfig, SetDirectServoConfigRequest


def main():
    rospy.init_node("set_servo_profile_velocity")

    service_name = rospy.get_param("~service_name", "/gryllo/direct_servo_config")
    servo_indices = rospy.get_param("~servo_indices", [6, 7])
    profile_velocity = rospy.get_param("~profile_velocity", 10)
    startup_delay = rospy.get_param("~startup_delay", 2.0)

    rospy.sleep(startup_delay)

    rospy.loginfo("Waiting for service: %s", service_name)
    rospy.wait_for_service(service_name)

    proxy = rospy.ServiceProxy(service_name, SetDirectServoConfig)

    for index in servo_indices:
        req = SetDirectServoConfigRequest()
        req.command = req.SET_SERVO_PROFILE_VEL
        req.data = [int(index), int(profile_velocity)]

        rospy.loginfo(
            "Setting servo index %d profile_velocity = %d",
            index,
            profile_velocity,
        )

        try:
            res = proxy(req)
            if not res.success:
                rospy.logerr(
                    "Failed to set profile_velocity for servo index %d",
                    index,
                )
            else:
                rospy.loginfo(
                    "Succeeded: servo index %d profile_velocity = %d",
                    index,
                    profile_velocity,
                )
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed for servo index %d: %s", index, e)

    rospy.loginfo("Finished setting servo profile velocities.")


if __name__ == "__main__":
    main()
