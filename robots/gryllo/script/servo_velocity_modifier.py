#!/usr/bin/env python3
import rospy
from spinal.srv import SetDirectServoConfig, SetDirectServoConfigRequest


def set_profile_velocity_with_retry(
    service_proxy,
    servo_index,
    profile_velocity,
    retry_count,
    retry_delay,
):
    for attempt in range(1, retry_count + 1):
        req = SetDirectServoConfigRequest()
        req.command = req.SET_SERVO_PROFILE_VEL
        req.data = [int(servo_index), int(profile_velocity)]

        rospy.loginfo(
            "Setting servo index %d profile_velocity = %d, attempt %d/%d",
            servo_index,
            profile_velocity,
            attempt,
            retry_count,
        )

        try:
            res = service_proxy(req)

            if res.success:
                rospy.loginfo(
                    "Succeeded: servo index %d profile_velocity = %d",
                    servo_index,
                    profile_velocity,
                )
                return True

            rospy.logwarn(
                "Service returned success=false for servo index %d",
                servo_index,
            )

        except rospy.ServiceException as e:
            rospy.logwarn(
                "Service call failed for servo index %d: %s",
                servo_index,
                e,
            )

        if attempt < retry_count:
            rospy.loginfo(
                "Retrying servo index %d after %.1f sec",
                servo_index,
                retry_delay,
            )
            rospy.sleep(retry_delay)

    rospy.logerr(
        "Failed to set profile_velocity for servo index %d after %d attempts",
        servo_index,
        retry_count,
    )
    return False


def main():
    rospy.init_node("servo_velocity_modifier")

    service_name = rospy.get_param("~service_name", "/gryllo/direct_servo_config")
    servo_indices = rospy.get_param("~servo_indices", [6, 7])
    profile_velocity = rospy.get_param("~profile_velocity", 10)

    startup_delay = rospy.get_param("~startup_delay", 8.0)
    per_call_delay = rospy.get_param("~per_call_delay", 3.0)
    retry_count = rospy.get_param("~retry_count", 3)
    retry_delay = rospy.get_param("~retry_delay", 3.0)

    rospy.loginfo("servo_velocity_modifier started")
    rospy.loginfo("service_name: %s", service_name)
    rospy.loginfo("servo_indices: %s", servo_indices)
    rospy.loginfo("profile_velocity: %d", profile_velocity)
    rospy.loginfo("startup_delay: %.1f sec", startup_delay)

    rospy.sleep(startup_delay)

    rospy.loginfo("Waiting for service: %s", service_name)
    rospy.wait_for_service(service_name)

    service_proxy = rospy.ServiceProxy(service_name, SetDirectServoConfig)

    all_success = True

    for i, servo_index in enumerate(servo_indices):
        success = set_profile_velocity_with_retry(
            service_proxy=service_proxy,
            servo_index=servo_index,
            profile_velocity=profile_velocity,
            retry_count=retry_count,
            retry_delay=retry_delay,
        )

        all_success = all_success and success

        if i != len(servo_indices) - 1:
            rospy.loginfo(
                "Waiting %.1f sec before next servo config",
                per_call_delay,
            )
            rospy.sleep(per_call_delay)

    if all_success:
        rospy.loginfo("Finished setting all servo profile velocities.")
    else:
        rospy.logerr("Failed to set one or more servo profile velocities.")


if __name__ == "__main__":
    main()
