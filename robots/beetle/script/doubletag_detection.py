#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

class twotagPIDcontroller():
    def __init__(self):
        rospy.init_node("twotagpidVel")
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
        self.rate = rospy.Rate(40)

        # regulator
        self.scale = 0.5
        
        # gains
        self.kp = -0.2 * self.scale
        self.ki = -0.0002 * self.scale
        self.kd = -0.01 * self.scale

        self.yaw_kp = -0.4 * self.scale
        self.yaw_ki = -0.001 * self.scale
        self.yaw_kd = -0.02 * self.scale

        # terms for pid
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])

        self.yaw_p_term = np.array([0.0, 0.0, 0.0])
        self.yaw_i_term = np.array([0.0, 0.0, 0.0])
        self.yaw_d_term = np.array([0.0, 0.0, 0.0])

        # position / angular error
        self.pos_err = np.array([0.0, 0.0, 0.0])
        self.pos_err_i = np.array([0.0, 0.0, 0.0])
        self.pos_err_d = np.array([0.0, 0.0, 0.0])

        self.yaw_err = np.array([0.0, 0.0, 0.0])
        self.yaw_err_i = np.array([0.0, 0.0, 0.0])
        self.yaw_err_d = np.array([0.0, 0.0, 0.0])

        #velocity
        self.currentVel = np.array([0.0, 0.0, 0.0])
        self.targetVel = np.array([0.0, 0.0, 0.0])

        #angular
        self.yaw = 0.0
        self.targetYaw = 0.0
        
        
        # duration
        self.dt = 0.025

        # variables for position data
        self.id0Position = np.array([0.0, 0.0, 0.0])
        self.id1Position = np.array([0.0, 0.0, 0.0])
        
    def callback(self,data):
        if data.detections:
            if len(data.detections) == 0:
                rospy.loginfo("no tags!")
                ### write programs for stop and yaw turn ###

            elif len(data.detections) == 1: # detecting only 1 tag
                if data.detections[0].id[0] == 0:
                    rospy.loginfo("detected 0!")
                    ### write programs for move to front ###
                    
                elif data.detections[0].id[0] == 1:
                    rospy.loginfo("detected 1!")
                    ### write programs for docking ###

            elif len(data.detections) == 2: # detecting both big and small tag
                rospy.loginfo(data.detections[0].id)
                id0_data = data.detections[0].pose.pose.pose.position
                self.id0Position = np.array([id0_data.z, -id0_data.x, -id0_data.y])
                id1_data = data.detections[1].pose.pose.pose.position
                self.id1Position = np.array([id0_data.z, -id0_data.x, -id0_data.y])
                rospy.loginfo(self.id1Position)
                ### write programs for aproaching ###

            else:
                rospy.loginfo("too many tags!")
                ### write programs for stop ###
    
            
    def main(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("1")
            self.rate.sleep()

if __name__ == "__main__":
    try:
        twoTagPID = twotagPIDcontroller()
        twoTagPID.main()
    except rospy.ROSInterruptException: pass
