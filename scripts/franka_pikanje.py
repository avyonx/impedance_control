#!/usr/bin/env python3

import rospy

import ros_numpy

from std_msgs.msg import Header, Bool

from geometry_msgs.msg import PointStamped, PoseStamped, WrenchStamped, Pose

import numpy as np
import math 

import time

p1 = Pose()
p1.position.x = 0.06396985410898132
p1.position.y = 0.02539255302335815
p1.position.z = 0.6394872138604897
p1.orientation.x = 0.90537661814846
p1.orientation.y = 0.4195209053774979
p1.orientation.z = -0.054973257742973215
p1.orientation.w = 0.03568375252725741

p2 = Pose()
p2.position.x = -1.
p2.position.y = -1.
p2.position.z = -1.
p2.orientation.x = -0.9458922841394939
p2.orientation.y = -0.32320301608521373
p2.orientation.z = 0.0063548040013790535
p2.orientation.w = 0.02805732818527244

# p3 = Pose()
# p3.position.x = 
# p3.position.y = 
# p3.position.z = 
# p3.orientation.x = 
# p3.orientation.y = 
# p3.orientation.z = 
# p3.orientation.w = 


class ExperimentFlow():
    def __init__(self):

        self.get_ground_centre = False

        # time.sleep(2.)
        self.image_sub = rospy.Subscriber("/ground_center", PointStamped, self.ground_center_cb)

        self.pose_ref_pub = rospy.Publisher("/impedance_control/pose_stamped_ref_input", PoseStamped, queue_size=10)
        self.ft_ref_pub = rospy.Publisher("/impedance_control/force_torque_ref_input", WrenchStamped, queue_size=10)

        time.sleep(2.)


        self.ground_center = None
        self.ground_msg = None

        self.pose_init = PoseStamped()
        self.pose_init.pose = p1

        self.pose_up = PoseStamped()
        self.pose_up.pose = p2

        # self.pose_down = PoseStamped()
        # self.pose_down.pose = p3

        self.do_force = WrenchStamped()
        self.do_force.wrench.force.z = -10.

        self.no_force = WrenchStamped()

    def ground_center_cb(self, msg):
        
        if self.get_ground_centre:
            self.ground_msg = msg.point


def main():

    rospy.init_node("experiment_1", disable_signals=True)

    exp = ExperimentFlow()

    # while not rospy.is_shutdown():
    try:        
        print("Go to init point..")
        msg = exp.pose_init
        msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
        exp.pose_ref_pub.publish(msg)
        print("Went to init. Sleeping for...")
        sleep_dur = 5
        for i in range(sleep_dur):
            print(int(sleep_dur - i))    
            time.sleep(1.0)
        
        exp.get_ground_centre = True
        while exp.ground_msg is None:
            time.sleep(1.0)
        
        exp.ground_center = exp.ground_msg
        exp.get_ground_centre = False
        
        print("Done. Go to upper point..")

        msg = exp.pose_up
        msg.pose.position.x = exp.ground_center.x
        msg.pose.position.y = exp.ground_center.y
        msg.pose.position.z = exp.ground_center.z + 0.1
        msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
        exp.pose_ref_pub.publish(msg)
        print("Went to upper point. Sleeping for...")
        sleep_dur = 5
        for i in range(sleep_dur):
            print(int(sleep_dur - i))
            time.sleep(1.0)

        print("Done. Go to lower point..")
        msg.pose.position.z = exp.ground_center.z
        msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
        exp.pose_ref_pub.publish(msg)
        print("Went to upper point. Sleeping for...")
        sleep_dur = 2
        for i in range(sleep_dur):
            print(int(sleep_dur - i))
            time.sleep(1.0)

        print("Done. Apply force 10N..")
        msg = exp.do_force
        msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
        exp.ft_ref_pub.publish(msg)
        print("Applying force. Sleeping for...")
        sleep_dur = 30
        for i in range(sleep_dur):
            print(int(sleep_dur - i))
            time.sleep(1.0)

        print("Done. release force..")
        msg = exp.no_force
        msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
        exp.ft_ref_pub.publish(msg)
        print("Releasing force. Sleeping for...")
        sleep_dur = 2
        for i in range(sleep_dur):
            print(int(sleep_dur - i))
            time.sleep(1.0)

        print("Done. Go up..")
        msg = exp.pose_up
        msg.pose.position.x = exp.ground_center.x
        msg.pose.position.y = exp.ground_center.y
        msg.pose.position.z = exp.ground_center.z + 0.1
        msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
        exp.pose_ref_pub.publish(msg)
        print("Going up. Sleeping for...")
        sleep_dur = 3
        for i in range(sleep_dur):
            print(int(sleep_dur - i))
            time.sleep(1.0)


        print("Done. Go home..")
        msg = exp.pose_init
        msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
        exp.pose_ref_pub.publish(msg)
        print("Going home. Sleeping for...")
        sleep_dur = 5
        for i in range(sleep_dur):
            print(int(sleep_dur - i))
            time.sleep(1.0)


        print("Done. Bye bye.")
    except KeyboardInterrupt:
        print("seems we're interrupted. Premature Bye.")

    # rospy.spin()


if __name__ == "__main__":
    main()