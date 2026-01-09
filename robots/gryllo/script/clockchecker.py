#!/usr/bin/env python3
import rospy
from spinal.msg import Imu

last = None

def cb(msg: Imu):
    global last
    t = msg.stamp.to_sec()
    if last is not None and t <= last:
        rospy.logwarn("NON-MONOTONIC imu stamp: curr=%.9f prev=%.9f dt=%.9f",
                      t, last, t-last)
    last = t

if __name__ == "__main__":
    rospy.init_node("imu_stamp_monotonic_checker")
    rospy.Subscriber("/gryllo/imu", Imu, cb, queue_size=200)
    rospy.spin()
