#!/usr/bin/env python
import rospy
from spinal.msg import ServoControlCmd

def main():
    rospy.init_node("init_dynamixel_position")

    pub = rospy.Publisher('/gryllo/servo/target_states', ServoControlCmd, queue_size=10)
    rospy.sleep(0.5)

    cmd = ServoControlCmd()
    # cmd.index = [0, 1, 2, 3, 4, 5, 6, 7]
    # cmd.angles = [2048, 2048, 2048, 2048, 2048, 2048, 2000, 2000]
    cmd.index = [4, 5, 6, 7]
    cmd.angles = [2048, 2048, 2650, 2550]
    
    rate = rospy.Rate(10)
    for _ in range(10):
        if rospy.is_shutdown():
            break
        pub.publish(cmd)
        rate.sleep()

    rospy.loginfo("Initial Dynamixel positions sent. Shutting down node.")
    rospy.signal_shutdown("Initial positions sent")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
