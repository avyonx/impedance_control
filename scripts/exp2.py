#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from geometry_msgs.msg import PoseStamped, WrenchStamped
from mmuav_impedance_control.msg import Modes

class exp2():

    def __init__(self):

        self.step_one = True
        self.step_two = False
        self.step_three = False
        self.step_four = False

        self.pose_threshold = 0.1

        self.tempForce = WrenchStamped()
        self.tempPose = PoseStamped()
        self.modes = Modes()

        self.r = 0.0
        self.r_max = 0.03
        self.omega = 0.6
        self.r_time = 20;

        self.hole_x = 0.0
        self.hole_y = 0.0

        self.start_x = self.hole_x + self.r
        self.start_y = self.hole_y
        self.start_z = 0.0

        self.time_start = rospy.Time.now().to_sec()

        self.pose_meas = PoseStamped()

        self.pose_ref_pub_ = rospy.Publisher('impedance_control/pose_ref', PoseStamped, queue_size=1)
        self.force_torque_ref_pub = rospy.Publisher('impedance_control/force_torque_ref', WrenchStamped, queue_size=1)
        self.modes_pub = rospy.Publisher('impedance_control/modes', Modes, queue_size=1)

        self.pose_sub_ = rospy.Subscriber('neo_position', PoseStamped, self.pose_cb)
        # Services for requesting trajectory interpolation
        rospy.sleep(5.)

        self.run()

    def pose_cb(self, msg):
        if (self.step_two):
            if (math.fabs(self.pose_meas.pose.position.z - msg.pose.position.z) > self.pose_threshold):
                self.step_two = False
                self.step_three = True
                self.time_start = rospy.Time.now().to_sec()

        self.pose_meas = msg


    def run(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            if (self.step_one):
                self.tempPose.header.stamp = rospy.Time.now() 
                self.tempPose.pose.position.x = self.start_x;
                self.tempPose.pose.position.y = self.start_y;
                self.tempPose.pose.position.z = self.start_z;
                self.tempPose.pose.orientation.x = 0;
                self.tempPose.pose.orientation.y = 0;
                self.tempPose.pose.orientation.z = 0;
                self.tempPose.pose.orientation.w = 1;

                self.tempForce.header.stamp = rospy.Time.now()
                self.tempForce.wrench.force.x = 0.0
                self.tempForce.wrench.force.y = 0.0
                self.tempForce.wrench.force.z = 2.0
                self.tempForce.wrench.torque.x = 0.0
                self.tempForce.wrench.torque.y = 0.0
                self.tempForce.wrench.torque.z = 0.0

                self.modes.controller_fx = False
                self.modes.controller_fy = False
                self.modes.controller_fz = True
                self.modes.controller_tx = False
                self.modes.controller_ty = False
                self.modes.controller_tz = False

                self.modes_pub.publish(self.modes)
                self.force_torque_ref_pub.publish(self.tempForce)
                self.pose_ref_pub_.publish(self.tempPose)

                if ((rospy.Time.now().to_sec() - self.time_start) > 10):
                    self.step_one = False
                    self.step_two = True
                    self.time_start = rospy.Time.now().to_sec()

            if (self.step_two):

                self.tempPose.header.stamp = rospy.Time.now() 
                self.tempPose.pose.position.x = self.hole_x + self.r * ((rospy.Time.now().to_sec() - self.time_start) / self.r_time) * math.cos(self.omega*(rospy.Time.now().to_sec() - self.time_start))
                self.tempPose.pose.position.y = self.hole_y + self.r * ((rospy.Time.now().to_sec() - self.time_start) / self.r_time) * math.sin(self.omega*(rospy.Time.now().to_sec() - self.time_start))
                self.tempPose.pose.position.z = self.start_z
                self.tempPose.pose.orientation.x = 0;
                self.tempPose.pose.orientation.y = 0;
                self.tempPose.pose.orientation.z = 0;
                self.tempPose.pose.orientation.w = 1;

                if (rospy.Time.now().to_sec() - self.time_start > self.r_time): self.r = self.r_max

                self.pose_ref_pub_.publish(self.tempPose)

            if (self.step_three):
                self.tempForce.header.stamp = rospy.Time.now()
                self.tempForce.wrench.force.x = 0.0
                self.tempForce.wrench.force.y = 0.0
                self.tempForce.wrench.force.z = 2.0
                self.tempForce.wrench.torque.x = 0.0
                self.tempForce.wrench.torque.y = 0.0
                self.tempForce.wrench.torque.z = 0.0

                self.modes.controller_fx = True
                self.modes.controller_fy = True
                self.modes.controller_fz = True
                self.modes.controller_tx = False
                self.modes.controller_ty = False
                self.modes.controller_tz = False

                self.tempPose.header.stamp = rospy.Time.now() 

                self.modes_pub.publish(self.modes)
                self.force_torque_ref_pub.publish(self.tempForce)
                self.pose_ref_pub_.publish(self.tempPose)

                if ((rospy.Time.now().to_sec() - self.time_start) > 10):
                    self.step_three = False
                    self.step_four = True
                    self.time_start = rospy.Time.now().to_sec()

            rate.sleep()



if __name__=="__main__":
    rospy.init_node("exp2")
    ex = exp2()
    