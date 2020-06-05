#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from geometry_msgs.msg import PoseStamped, WrenchStamped

class exp3():

    def __init__(self):

        self.pose_ref_pub_ = rospy.Publisher('/uav/impedance_control/pose_stamped_ref_input', PoseStamped, queue_size=1)
        self.force_torque_ref_pub = rospy.Publisher('/uav/impedance_control/force_torque_ref_input', WrenchStamped, queue_size=1)
        # Services for requesting trajectory interpolation
        rospy.sleep(5.)
        self.force_setpoint_flag = [False]
        self.pose_setpoint_flag = [False]

        self.force_setpoint = [WrenchStamped()]
        self.pose_setpoint = [PoseStamped()]

        self.pose_setpoint[0].header.stamp = rospy.Time.now()
        self.pose_setpoint[0].pose.position.x = 1.00;
        self.pose_setpoint[0].pose.position.y = 0;
        self.pose_setpoint[0].pose.position.z = 1;
        self.pose_setpoint[0].pose.orientation.x = 0;
        self.pose_setpoint[0].pose.orientation.y = 0;
        self.pose_setpoint[0].pose.orientation.z = 0;
        self.pose_setpoint[0].pose.orientation.w = 1;


        self.force_setpoint[0].wrench.force.x = 1.0

        self.ramp_velocity = 0.05
        self.up = False
        self.time = 30

        self.run()

    def run(self):
        rate = rospy.Rate(100)

        time_next = rospy.Time.now().to_sec() + self.time

        force_ref = WrenchStamped();

        while not rospy.is_shutdown():

            time_now = rospy.Time.now().to_sec()

            self.pose_setpoint[0].header.stamp = rospy.Time.now()
            self.pose_ref_pub_.publish(self.pose_setpoint[0])


            if ( not self.pose_setpoint_flag[0]):
                if ((time_next - time_now) < 0.0):
                    self.pose_setpoint_flag[0] = True
                    time_next = rospy.Time.now().to_sec() + self.time


            if (self.pose_setpoint_flag[0]):
                if (not self.up):
                    for i in range(len(self.force_setpoint)):
                        if (not self.force_setpoint_flag[i]):
                            if ((time_next - time_now) < 0.0):
                                self.force_setpoint_flag[i] = True
                                if (i+1 != len(self.force_setpoint)):
                                    self.force_setpoint_flag[i+1] = False
                                else:
                                    self.up = True
                                    self.force_setpoint_flag[i] = False
                                time_next = rospy.Time.now().to_sec() + self.time
                            if (force_ref.wrench.force.x < self.force_setpoint[i].wrench.force.x):
                                force_ref.wrench.force.x = force_ref.wrench.force.x + self.ramp_velocity/100.0
                else:
                    for i in range(len(self.force_setpoint)-1, -1, -1):
                        if (not self.force_setpoint_flag[i]):
                            if ((time_next - time_now) < 0.0):
                                self.force_setpoint_flag[i] = True
                                if (i-1 != -1):
                                    self.force_setpoint_flag[i-1] = False
                                time_next = rospy.Time.now().to_sec() + self.time

                            if (i-1 != -1):
                                if (force_ref.wrench.force.x > (self.force_setpoint[i-1].wrench.force.x + self.ramp_velocity/100.0)):
                                    force_ref.wrench.force.x = force_ref.wrench.force.x - self.ramp_velocity/100.0
                            else:
                                if (force_ref.wrench.force.x > 1.5*self.ramp_velocity/100.0):
                                    force_ref.wrench.force.x = force_ref.wrench.force.x - self.ramp_velocity/100.0
                                else:
                                    force_ref.wrench.force.x = 0.0

            force_ref.header.stamp = rospy.Time.now()
            self.force_torque_ref_pub.publish(force_ref)

            rate.sleep()



if __name__=="__main__":
    rospy.init_node("exp3")
    exp3()