#!/usr/bin/env python
from __future__ import print_function
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from sensor_msgs.msg import JointState
from geometry_msgs.msg import QuaternionStamped
import tf.transformations as tft

guide = """ 
Instruction:

---------------------------
r: arming motor (please do before takeoff)
t: takeoff
l: land
f: force landing
h: halt (force stop motor)

     q          w           e            [         u         o 
(turn left) (forward)  (turn right)  (move up)  (roll +) (spine +)
     a          s           d            ]         i         p
(move left) (backward) (move right) (move down) (roll -) (spine -)

c: reset roll/spine

Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def printMsg(msg, msg_len=60):
    print(msg.ljust(msg_len) + "\r", end="")

def clamp(x, x_min, x_max):
    return max(x_min, min(x, x_max))

def publish_roll_quaternion(pub, roll_val):
    q = tft.quaternion_from_euler(roll_val, 0.0, 0.0)

    msg = QuaternionStamped()
    msg.header.stamp = rospy.Time.now()
    msg.quaternion.x = q[0]
    msg.quaternion.y = q[1]
    msg.quaternion.z = q[2]
    msg.quaternion.w = q[3]
    pub.publish(msg)

def publish_spine(pub, spine_val):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = [
        "spine_joint_1",
        "spine_joint_2",
        "spine_joint_3",
        "spine_joint_4",
        "spine_joint_5",
        "spine_joint_6",
    ]
    js.position = [spine_val] * 6
    pub.publish(js)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("keyboard_spine_command")
    robot_ns = rospy.get_param("~robot_ns", "")

    print(guide)

    if not robot_ns:
        master = rosgraph.Master('/rostopic')
        try:
            _, subs, _ = master.getSystemState()
        except Exception:
            subs = []

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        if len(teleop_topics) == 1:
            robot_ns = teleop_topics[0].split('/teleop')[0]

    ns = robot_ns + "/teleop_command"

    land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
    halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
    start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
    takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
    force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
    nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)

    joint_pub = rospy.Publisher(robot_ns + '/manual_spine_joints_ctrl', JointState, queue_size=1)

    quat_pub = rospy.Publisher(robot_ns + '/final_target_baselink_rot',
                               QuaternionStamped, queue_size=1)

    xy_vel = rospy.get_param("~xy_vel", 0.2)
    z_vel = rospy.get_param("~z_vel", 0.2)
    yaw_vel = rospy.get_param("~yaw_vel", 0.2)

    roll_step = rospy.get_param("~roll_step", 0.02)
    roll_min = rospy.get_param("~roll_min", -1.57)
    roll_max = rospy.get_param("~roll_max", 1.57)

    spine_step = rospy.get_param("~spine_step", 0.02)
    spine_min = rospy.get_param("~spine_min", -0.52)
    spine_max = rospy.get_param("~spine_max", 0.52)

    initial_roll = rospy.get_param("~initial_roll", 0.0)
    initial_spine = rospy.get_param("~initial_spine", 0.0)

    roll_val = initial_roll
    spine_val = initial_spine

    try:
        while not rospy.is_shutdown():
            nav_msg = FlightNav()
            nav_msg.control_frame = FlightNav.WORLD_FRAME
            nav_msg.target = FlightNav.COG

            key = getKey()
            msg = ""

            if key == 'l':
                land_pub.publish(Empty())
                msg = "send land command"

            elif key == 'r':
                start_pub.publish(Empty())
                msg = "send motor-arming command"

            elif key == 'h':
                halt_pub.publish(Empty())
                msg = "send halt command"

            elif key == 'f':
                force_landing_pub.publish(Empty())
                msg = "send force landing command"

            elif key == 't':
                takeoff_pub.publish(Empty())
                msg = "send takeoff command"

            elif key == 'w':
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = xy_vel
                nav_pub.publish(nav_msg)
                msg = "send +x vel command"

            elif key == 's':
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = -xy_vel
                nav_pub.publish(nav_msg)
                msg = "send -x vel command"

            elif key == 'a':
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_y = xy_vel
                nav_pub.publish(nav_msg)
                msg = "send +y vel command"

            elif key == 'd':
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_y = -xy_vel
                nav_pub.publish(nav_msg)
                msg = "send -y vel command"

            elif key == 'q':
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = yaw_vel
                nav_pub.publish(nav_msg)
                msg = "send +yaw vel command"

            elif key == 'e':
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = -yaw_vel
                nav_pub.publish(nav_msg)
                msg = "send -yaw vel command"

            elif key == '[':
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = z_vel
                nav_pub.publish(nav_msg)
                msg = "send +z vel command"

            elif key == ']':
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = -z_vel
                nav_pub.publish(nav_msg)
                msg = "send -z vel command"

            elif key == 'u':
                roll_val = clamp(roll_val + roll_step, roll_min, roll_max)
                publish_roll_quaternion(quat_pub, roll_val)
                msg = "send quaternion roll target = {:.3f}".format(roll_val)

            elif key == 'i':
                roll_val = clamp(roll_val - roll_step, roll_min, roll_max)
                publish_roll_quaternion(quat_pub, roll_val)
                msg = "send quaternion roll target = {:.3f}".format(roll_val)

            elif key == 'o':
                spine_val = clamp(spine_val + spine_step, spine_min, spine_max)
                publish_spine(joint_pub, spine_val)
                msg = "spine_joint_1-6 = {:.3f}".format(spine_val)

            elif key == 'p':
                spine_val = clamp(spine_val - spine_step, spine_min, spine_max)
                publish_spine(joint_pub, spine_val)
                msg = "spine_joint_1-6 = {:.3f}".format(spine_val)

            elif key == 'c':
                roll_val = initial_roll
                spine_val = initial_spine

                publish_roll_quaternion(quat_pub, roll_val)
                publish_spine(joint_pub, spine_val)
                msg = "reset roll quaternion and spine"

            elif key == '\x03':
                break

            else:
                printMsg("")
                rospy.sleep(0.001)
                continue

            printMsg(msg)
            rospy.sleep(0.001)

    except Exception as e:
        print(repr(e))

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
