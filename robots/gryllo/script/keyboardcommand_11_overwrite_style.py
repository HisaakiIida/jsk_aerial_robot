#!/usr/bin/env python
from __future__ import print_function
import sys
import select
import termios
import tty

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

     q          w           e            [         u         j         o
(turn left) (forward)  (turn right)  (move up)  (roll +) (pitch +) (spine +)
     a          s           d            ]         i         k         p
(move left) (backward) (move right) (move down) (roll -) (pitch -) (spine -)

CTRL+u : act_unit offset +
CTRL+i : act_unit offset -

c: smooth reset roll/pitch/spine/act_unit_offset

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

def printMsg(msg, msg_len=90):
    print(msg.ljust(msg_len) + "\r", end="")

def clamp(x, x_min, x_max):
    return max(x_min, min(x, x_max))

def approach(current, target, max_step):
    if current < target:
        return min(current + max_step, target)
    elif current > target:
        return max(current - max_step, target)
    return current

def publish_attitude_quaternion(pub, roll_val, pitch_val):
    q = tft.quaternion_from_euler(roll_val, pitch_val, 0.0)

    msg = QuaternionStamped()
    msg.header.stamp = rospy.Time.now()
    msg.quaternion.x = q[0]
    msg.quaternion.y = q[1]
    msg.quaternion.z = q[2]
    msg.quaternion.w = q[3]
    pub.publish(msg)


def publish_manual_joints(pub, spine_val, act_unit_offset_val):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = [
        "act_unit_joint_1",
        "act_unit_joint_2",
        "spine_joint_1",
        "spine_joint_2",
        "spine_joint_3",
        "spine_joint_4",
        "spine_joint_5",
        "spine_joint_6",
    ]
    # ここで送る act_unit_joint_* は「最終角」ではなく手動オフセット
    js.position = [
        act_unit_offset_val,
        -act_unit_offset_val,
        spine_val,
        spine_val,
        spine_val,
        spine_val,
        spine_val,
        spine_val,
    ]
    pub.publish(js)

def publish_spine_smooth(joint_pub, start_spine, target_spine,
                         act_unit_offset_val, duration=0.5, rate_hz=50):
    steps = max(1, int(duration * rate_hz))
    rate = rospy.Rate(rate_hz)

    for i in range(1, steps + 1):
        a = float(i) / float(steps)
        spine_val = start_spine + (target_spine - start_spine) * a
        publish_manual_joints(joint_pub, spine_val, act_unit_offset_val)
        rate.sleep()

    return target_spine

def smooth_reset(quat_pub, joint_pub,
                 cur_roll, cur_pitch, cur_spine, cur_act_unit_offset,
                 tgt_roll, tgt_pitch, tgt_spine, tgt_act_unit_offset,
                 duration=10.0, rate_hz=50):
    steps = max(1, int(duration * rate_hz))
    rate = rospy.Rate(rate_hz)

    start_roll = cur_roll
    start_pitch = cur_pitch
    start_spine = cur_spine
    start_act_unit_offset = cur_act_unit_offset

    for i in range(1, steps + 1):
        a = float(i) / float(steps)

        roll_val = start_roll + (tgt_roll - start_roll) * a
        pitch_val = start_pitch + (tgt_pitch - start_pitch) * a
        spine_val = start_spine + (tgt_spine - start_spine) * a
        act_unit_offset_val = start_act_unit_offset + (tgt_act_unit_offset - start_act_unit_offset) * a

        publish_attitude_quaternion(quat_pub, roll_val, pitch_val)
        publish_manual_joints(joint_pub, spine_val, act_unit_offset_val)
        rate.sleep()

    return tgt_roll, tgt_pitch, tgt_spine, tgt_act_unit_offset


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

    joint_pub = rospy.Publisher(robot_ns + '/manual_spine_joints_ctrl',
                                JointState, queue_size=1)

    quat_pub = rospy.Publisher(robot_ns + '/final_target_baselink_rot',
                               QuaternionStamped, queue_size=1)

    xy_vel = rospy.get_param("~xy_vel", 0.2)
    z_vel = rospy.get_param("~z_vel", 0.2)
    yaw_vel = rospy.get_param("~yaw_vel", 0.2)

    roll_step = rospy.get_param("~roll_step", 0.02)
    roll_min = rospy.get_param("~roll_min", -1.57)
    roll_max = rospy.get_param("~roll_max", 1.57)

    pitch_step = rospy.get_param("~pitch_step", 0.02)
    pitch_min = rospy.get_param("~pitch_min", -1.57)
    pitch_max = rospy.get_param("~pitch_max", 1.57)

    spine_step = rospy.get_param("~spine_step", 0.02)
    spine_min = rospy.get_param("~spine_min", -0.52)
    spine_max = rospy.get_param("~spine_max", 0.52)

    spine_publish_rate = rospy.get_param("~spine_publish_rate", 50.0)
    spine_slew_rate = rospy.get_param("~spine_slew_rate", 0.1)
    
    act_unit_step = rospy.get_param("~act_unit_step", 0.02)
    act_unit_min = rospy.get_param("~act_unit_min", -1.57)
    act_unit_max = rospy.get_param("~act_unit_max", 1.57)

    initial_roll = rospy.get_param("~initial_roll", 0.0)
    initial_pitch = rospy.get_param("~initial_pitch", 0.0)
    initial_spine = rospy.get_param("~initial_spine", 0.0)
    initial_act_unit_offset = rospy.get_param("~initial_act_unit_offset", 0.0)

    roll_val = initial_roll
    pitch_val = initial_pitch
    spine_val = initial_spine
    target_spine_val = initial_spine
    act_unit_offset_val = initial_act_unit_offset

    rospy.sleep(0.2)
    publish_attitude_quaternion(quat_pub, roll_val, pitch_val)
    publish_manual_joints(joint_pub, spine_val, act_unit_offset_val)

    rate = rospy.Rate(spine_publish_rate)
    
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

            # body roll only
            # act_unit compensation is handled in C++ from fc_roll
            elif key == 'u':
                roll_val = clamp(roll_val + roll_step, roll_min, roll_max)
                publish_attitude_quaternion(quat_pub, roll_val, pitch_val)
                msg = "body roll={:.3f}, pitch={:.3f}, act_unit_offset={:.3f}".format(
                    roll_val, pitch_val, act_unit_offset_val)

            elif key == 'i':
                roll_val = clamp(roll_val - roll_step, roll_min, roll_max)
                publish_attitude_quaternion(quat_pub, roll_val, pitch_val)
                msg = "body roll={:.3f}, pitch={:.3f}, act_unit_offset={:.3f}".format(
                    roll_val, pitch_val, act_unit_offset_val)

            elif key == 'j':
                pitch_val = clamp(pitch_val + pitch_step, pitch_min, pitch_max)
                publish_attitude_quaternion(quat_pub, roll_val, pitch_val)
                msg = "body roll={:.3f}, pitch={:.3f}".format(roll_val, pitch_val)

            elif key == 'k':
                pitch_val = clamp(pitch_val - pitch_step, pitch_min, pitch_max)
                publish_attitude_quaternion(quat_pub, roll_val, pitch_val)
                msg = "body roll={:.3f}, pitch={:.3f}".format(roll_val, pitch_val)

            elif key == 'o':
                target_spine_val = clamp(target_spine_val + spine_step, spine_min, spine_max)
                msg = "target_spine={:.3f}, spine={:.3f}, act_unit_offset={:.3f}".format(
                    target_spine_val, spine_val, act_unit_offset_val)
                
            elif key == 'p':
                target_spine_val = clamp(target_spine_val - spine_step, spine_min, spine_max)
                msg = "target_spine={:.3f}, spine={:.3f}, act_unit_offset={:.3f}".format(
                    target_spine_val, spine_val, act_unit_offset_val)
                
            elif key == '\x15':   # Ctrl+u
                act_unit_offset_val = clamp(act_unit_offset_val + act_unit_step,
                                            act_unit_min, act_unit_max)
                publish_manual_joints(joint_pub, spine_val, act_unit_offset_val)
                msg = "act_unit offset + : {:.3f}".format(act_unit_offset_val)

            elif key == '\t':     # Ctrl+i == TAB
                act_unit_offset_val = clamp(act_unit_offset_val - act_unit_step,
                                            act_unit_min, act_unit_max)
                publish_manual_joints(joint_pub, spine_val, act_unit_offset_val)
                msg = "act_unit offset - : {:.3f}".format(act_unit_offset_val)

            elif key == 'c':
                roll_val, pitch_val, spine_val, act_unit_offset_val = smooth_reset(
                    quat_pub, joint_pub,
                    roll_val, pitch_val, spine_val, act_unit_offset_val,
                    initial_roll, initial_pitch, initial_spine, initial_act_unit_offset,
                    duration=2.5, rate_hz=50
                )
                msg = "smooth reset roll, pitch, spine and act_unit_offset"

            elif key == '\x03':
                break

            else:
                printMsg("")
                rospy.sleep(0.001)
                continue

            max_spine_step_per_loop = spine_slew_rate / spine_publish_rate
            new_spine_val = approach(spine_val, target_spine_val, max_spine_step_per_loop)
            
            if new_spine_val != spine_val:
                spine_val = new_spine_val
                publish_manual_joints(joint_pub, spine_val, act_unit_offset_val)
            
            printMsg(msg)
            rate.sleep()

    except Exception as e:
        print(repr(e))

    finally:
        print("")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
