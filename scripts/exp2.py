#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from geometry_msgs.msg import PoseStamped, WrenchStamped
from mmuav_impedance_control.msg import Modes
from std_srvs.srv import SetBool
from std_msgs.msg import Empty

class exp2():

    def __init__(self):

        self.step_one = True
        self.step_two = False
        self.step_three = False
        self.step_four = False
        self.step_five = False
        self.step_release = False

        self.pose_threshold = 0.1

        self.tempForce = WrenchStamped()
        self.tempPose = PoseStamped()
        self.tempPoseUAV = PoseStamped()
        self.modes = Modes()
        self.left_arm_pose = PoseStamped()
        self.right_arm_pose = PoseStamped()

        self.x_screw_in = 0
        self.y_screw_in = 0

        self.r = 0.02
        self.r_max = 0.02
        self.omega = 1.5
        self.r_time = 40

        self.yaw_time = 20
        self.yaw_max = 2.5*6.24
        self.current_yaw = 0

        self.hole_x = -0.310091998534
        self.hole_y = -0.4

        self.start_x = self.hole_x
        self.start_y = self.hole_y
        self.start_z = 1.04

        self.arm_current_mode = True
        self.uav_current_mode = True

        self.time_start = rospy.Time.now().to_sec()

        self.pose_meas = PoseStamped()

        self.pose_ref_pub_ = rospy.Publisher('impedance_control/pose_ref', PoseStamped, queue_size=1)
        self.force_torque_ref_pub = rospy.Publisher('impedance_control/force_torque_ref', WrenchStamped, queue_size=1)
        self.modes_pub = rospy.Publisher('impedance_control/modes', Modes, queue_size=1)
        self.left_arm_pub = rospy.Publisher('left_manipulator/set_point', PoseStamped, queue_size=1)
        self.right_arm_pub = rospy.Publisher('right_manipulator/set_point', PoseStamped, queue_size=1)
        self.uav_pub = rospy.Publisher('uav/set_point', PoseStamped, queue_size=1)

        self.pose_sub_ = rospy.Subscriber('insert', Empty, self.insert_cb)
        self.pose_sub_ = rospy.Subscriber('release', Empty, self.release_cb)
        rospy.wait_for_service('dual_arm_manipulator/dual_arm_mode')

        self.manipulator_mode_srv = rospy.ServiceProxy('dual_arm_manipulator/dual_arm_mode', SetBool)
        self.uav_mode_srv = rospy.ServiceProxy('dual_arm_manipulator/uav_mode', SetBool)
        # Services for requesting trajectory interpolation
        rospy.sleep(5.)

        self.run()

    def insert_cb(self, msg):
        if (self.step_two):
                self.step_two = False
                self.step_three = True
                print "step three"
                self.time_start = rospy.Time.now().to_sec()
                self.x_screw_in = self.tempPose.pose.position.x
                self.y_screw_in = self.tempPose.pose.position.y

    def release_cb(self, msg):
        if (self.step_four):
            self.step_four = False
            self.step_five = True
            print "step five"
            self.time_start = rospy.Time.now().to_sec()


    def euler2quaternion(self, euler):
        quaternion = [0, 0, 0, 0]

        cy = math.cos(euler[2] * 0.5)
        sy = math.sin(euler[2] * 0.5)
        cr = math.cos(euler[0] * 0.5)
        sr = math.sin(euler[0] * 0.5)
        cp = math.cos(euler[1] * 0.5)
        sp = math.sin(euler[1] * 0.5)

        quaternion[0] = cy * cr * cp + sy * sr * sp
        quaternion[1] = cy * sr * cp - sy * cr * sp
        quaternion[2] = cy * cr * sp + sy * sr * cp
        quaternion[3] = sy * cr * cp - cy * sr * sp

        return quaternion

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

                if ((rospy.Time.now().to_sec() - self.time_start) > 5):
                    self.step_one = False
                    self.step_two = True
                    print "step two"
                    self.time_start = rospy.Time.now().to_sec()

            elif (self.step_two):

                self.gain = (rospy.Time.now().to_sec() - self.time_start) / self.r_time
                if (rospy.Time.now().to_sec() - self.time_start > self.r_time): self.gain = 1


                self.tempPose.header.stamp = rospy.Time.now() 
                self.tempPose.pose.position.x = self.hole_x + self.r * self.gain  * math.cos(self.omega*(rospy.Time.now().to_sec() - self.time_start))
                self.tempPose.pose.position.y = self.hole_y + self.r * self.gain  * math.sin(self.omega*(rospy.Time.now().to_sec() - self.time_start))
                self.tempPose.pose.position.z = self.start_z
                self.tempPose.pose.orientation.x = 0;
                self.tempPose.pose.orientation.y = 0;
                self.tempPose.pose.orientation.z = 0;
                self.tempPose.pose.orientation.w = 1;

                self.pose_ref_pub_.publish(self.tempPose)

            elif (self.step_three):
                #if self.arm_current_mode:
                #    arm_current_mode_temp = self.manipulator_mode_srv(False)
                #    self.arm_current_mode = arm_current_mode_temp.success

                    #uav_current_mode_temp = self.uav_mode_srv(False)

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
                self.tempPose.pose.position.x = self.hole_x #self.x_screw_in
                self.tempPose.pose.position.y = self.hole_y #self.y_screw_in
                self.tempPose.pose.position.z = self.start_z
                self.tempPose.pose.orientation.x = 0;
                self.tempPose.pose.orientation.y = 0;
                self.tempPose.pose.orientation.z = 0;
                self.tempPose.pose.orientation.w = 1;
                self.tempPose.header.stamp = rospy.Time.now()

                self.left_arm_pose.header.stamp = rospy.Time.now()
                self.left_arm_pose.pose.position.x = 0.0;
                self.left_arm_pose.pose.position.y = 0.0;
                self.left_arm_pose.pose.position.z = 0.0;
                self.left_arm_pose.pose.orientation.x = 0;
                self.left_arm_pose.pose.orientation.y = 0;
                self.left_arm_pose.pose.orientation.z = -0.707;
                self.left_arm_pose.pose.orientation.w = 0.707; 

                self.right_arm_pose.header.stamp = rospy.Time.now()
                self.right_arm_pose.pose.position.x = 0.0;
                self.right_arm_pose.pose.position.y = -0.0;
                self.right_arm_pose.pose.position.z = 0.0;
                self.right_arm_pose.pose.orientation.x = 0;
                self.right_arm_pose.pose.orientation.y = 0;
                self.right_arm_pose.pose.orientation.z = 0.707;
                self.right_arm_pose.pose.orientation.w = 0.707; 

                self.tempPoseUAV.header.stamp = rospy.Time.now() 
                self.tempPoseUAV.pose.position.x = self.hole_x
                self.tempPoseUAV.pose.position.y = self.hole_y
                self.tempPoseUAV.pose.position.z = self.start_z
                self.tempPoseUAV.pose.orientation.x = 0;
                self.tempPoseUAV.pose.orientation.y = 0;
                self.tempPoseUAV.pose.orientation.z = 0;
                self.tempPoseUAV.pose.orientation.w = 1;
                self.tempPoseUAV.header.stamp = rospy.Time.now()

                self.uav_pub.publish(self.tempPoseUAV)
                #self.modes_pub.publish(self.modes)
                #self.force_torque_ref_pub.publish(self.tempForce)
                #self.pose_ref_pub_.publish(self.tempPose)

                self.modes_pub.publish(self.modes)
                self.force_torque_ref_pub.publish(self.tempForce)
                self.pose_ref_pub_.publish(self.tempPose)
                #self.left_arm_pub.publish(self.left_arm_pose)
                #self.right_arm_pub.publish(self.right_arm_pose)

                if ((rospy.Time.now().to_sec() - self.time_start) > 5):
                    self.step_three = False
                    self.step_four = True
                    self.time_start = rospy.Time.now().to_sec()
                    print "step four"

            elif (self.step_four):

                gain = (rospy.Time.now().to_sec() - self.time_start) / self.yaw_time
                if (rospy.Time.now().to_sec() - self.time_start > self.yaw_time): gain = 1
                self.current_yaw = -self.yaw_max * gain

                euler = [0, 0, self.current_yaw]

                q = self.euler2quaternion(euler)

                self.tempPose.pose.orientation.x = 0
                self.tempPose.pose.orientation.y = 0
                self.tempPose.pose.orientation.z = self.current_yaw
                self.tempPose.pose.orientation.w = 1

                self.modes.controller_fx = True
                self.modes.controller_fy = True
                self.modes.controller_fz = True
                self.modes.controller_tx = False
                self.modes.controller_ty = False
                self.modes.controller_tz = True

                self.tempForce.header.stamp = rospy.Time.now()
                self.tempForce.wrench.force.x = 0.0
                self.tempForce.wrench.force.y = 0.0
                self.tempForce.wrench.force.z = 2.0
                self.tempForce.wrench.torque.x = 0.0
                self.tempForce.wrench.torque.y = 0.0
                self.tempForce.wrench.torque.z = -0.1

                self.modes_pub.publish(self.modes)
                self.pose_ref_pub_.publish(self.tempPose)
                self.force_torque_ref_pub.publish(self.tempForce)

            elif (self.step_five):

                if self.arm_current_mode:
                    arm_current_mode_temp = self.manipulator_mode_srv(False)
                    self.arm_current_mode = arm_current_mode_temp.success

                self.tempPose.pose.orientation.x = 0
                self.tempPose.pose.orientation.y = 0
                self.tempPose.pose.orientation.z = self.current_yaw
                self.tempPose.pose.orientation.w = 1

                self.modes.controller_fx = True
                self.modes.controller_fy = True
                self.modes.controller_fz = True
                self.modes.controller_tx = False
                self.modes.controller_ty = False
                self.modes.controller_tz = True

                self.tempForce.header.stamp = rospy.Time.now()
                self.tempForce.wrench.force.x = 0.0
                self.tempForce.wrench.force.y = 0.0
                self.tempForce.wrench.force.z = 2.0
                self.tempForce.wrench.torque.x = 0.0
                self.tempForce.wrench.torque.y = 0.0
                self.tempForce.wrench.torque.z = -0.1

                self.modes_pub.publish(self.modes)
                self.pose_ref_pub_.publish(self.tempPose)
                self.force_torque_ref_pub.publish(self.tempForce)

                if ((rospy.Time.now().to_sec() - self.time_start) > 5):
                    self.step_five = False
                    self.step_release = True
                    self.time_start = rospy.Time.now().to_sec()
                    print "step release"

            elif (self.step_release):

                self.modes.controller_fx = False
                self.modes.controller_fy = False
                self.modes.controller_fz = False
                self.modes.controller_tx = False
                self.modes.controller_ty = False
                self.modes.controller_tz = True

                self.left_arm_pose.header.stamp = rospy.Time.now()
                self.left_arm_pose.pose.position.x = 0.0;
                self.left_arm_pose.pose.position.y = 0.045;
                self.left_arm_pose.pose.position.z = 0.0;
                self.left_arm_pose.pose.orientation.x = 0;
                self.left_arm_pose.pose.orientation.y = 0;
                self.left_arm_pose.pose.orientation.z = -0.707;
                self.left_arm_pose.pose.orientation.w = 0.707; 

                self.right_arm_pose.header.stamp = rospy.Time.now()
                self.right_arm_pose.pose.position.x = 0.0;
                self.right_arm_pose.pose.position.y = -0.045;
                self.right_arm_pose.pose.position.z = 0.0;
                self.right_arm_pose.pose.orientation.x = 0;
                self.right_arm_pose.pose.orientation.y = 0;
                self.right_arm_pose.pose.orientation.z = 0.707;
                self.right_arm_pose.pose.orientation.w = 0.707; 

                self.modes_pub.publish(self.modes)
                self.left_arm_pub.publish(self.left_arm_pose)
                self.right_arm_pub.publish(self.right_arm_pose)

                #poslati letjelicu iznad

            rate.sleep()



if __name__=="__main__":
    rospy.init_node("exp2")
    ex = exp2()
    