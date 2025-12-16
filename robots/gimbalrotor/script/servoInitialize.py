#!/usr/bin/env python
import rospy
from spinal.msg import ServoControlCmd

def main():
    rospy.init_node("init_dynamixel_position")

    pub = rospy.Publisher('/gimbalrotor/servo/target_states', ServoControlCmd, queue_size=10)

    # パブリッシャ接続待ち（少し待ってから送る）
    rospy.sleep(0.5)

    cmd = ServoControlCmd()
    # 対象サーボID
    cmd.index = [0, 1, 2, 3, 4, 5, 6, 7]
    # 対応する角度
    cmd.angles = [2048, 2048, 2048, 2048, 2048, 2048, 1900, 1550]

    rate = rospy.Rate(10)  # 10 Hz

    # 起動直後に数回送っておく（1回だけでも良いが、念のため複数回）
    for _ in range(10):
        if rospy.is_shutdown():
            break
        pub.publish(cmd)
        rate.sleep()

    rospy.loginfo("Initial Dynamixel positions sent. Shutting down node.")
    # 一度送ればよいノードなので終了
    rospy.signal_shutdown("Initial positions sent")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
