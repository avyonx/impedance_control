#!/usr/bin/env python3

import rospy

import ros_numpy

from std_msgs.msg import Header, Bool

from geometry_msgs.msg import PointStamped, PoseStamped, WrenchStamped, Pose

import numpy as np
import math 

import time

p1 = Pose()
p1.position.x = -0.025852538094631283
p1.position.y = 0.27975319765236456
p1.position.z = 0.7926032630407481
p1.orientation.x = -0.916246833276856
p1.orientation.y = -0.4003696860841502
p1.orientation.z = -0.005945824587120584
p1.orientation.w = 0.012668944115097896


p2 = Pose()
p2.position.x = -0.04989398453485186
p2.position.y = 0.5831070015134662
p2.position.z = 0.7338375310166932
p2.orientation.x = -0.9250248390731137
p2.orientation.y = -0.3799053166266796
p2.orientation.z = -0.00022782289315277833
p2.orientation.w = 0.0009724162029052998

p3 = Pose()
p3.position.x = -0.04989398453485186
p3.position.y = 0.5831070015134662
p3.position.z = 0.6338375310166932
p3.orientation.x = -0.9250248390731137
p3.orientation.y = -0.3799053166266796
p3.orientation.z = -0.00022782289315277833
p3.orientation.w = 0.0009724162029052998


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
        self.start_sub = rospy.Subscriber("/start", Bool, self.start_cb)

        self.pose_ref_pub = rospy.Publisher("/impedance_control/pose_stamped_ref_input", PoseStamped, queue_size=10)
        self.ft_ref_pub = rospy.Publisher("/impedance_control/force_torque_ref_input", WrenchStamped, queue_size=10)

        time.sleep(2.)

        self.start_flag = False

        self.pose_init = PoseStamped()
        self.pose_init.pose = p1

        self.pose_up = PoseStamped()
        self.pose_up.pose = p2

        self.pose_down = PoseStamped()
        self.pose_down.pose = p3

        self.do_force = WrenchStamped()
        self.do_force.wrench.force.z = -5.

        self.no_force = WrenchStamped()

    def start_cb(self, msg):
        
        if not self.start_flag:
            self.start_flag = True


def main():

    rospy.init_node("experiment_1", disable_signals=True)

    exp = ExperimentFlow()

    print("Go to init point..")
    msg = exp.pose_init
    msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
    exp.pose_ref_pub.publish(msg)
    print("Went to init. Sleeping for...")
    sleep_dur = 5
    for i in range(sleep_dur):
        print(int(sleep_dur - i))    
        time.sleep(1.0)
        


    while not rospy.is_shutdown():
        try:
            while not exp.start_flag:
                time.sleep(1.0)

            print("Done. Go to upper point..")

            msg = exp.pose_up
            msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
            exp.pose_ref_pub.publish(msg)
            print("Went to upper point. Sleeping for...")
            sleep_dur = 10
            for i in range(sleep_dur):
                print(int(sleep_dur - i))
                time.sleep(1.0)

            print("Done. Go to lower point..")
            msg = exp.pose_down
            msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
            exp.pose_ref_pub.publish(msg)
            print("Went to lower point. Sleeping for...")
            sleep_dur = 5
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

            exp.start_flag = False

            print("Done. Bye bye.")
        except KeyboardInterrupt:
            print("seems we're interrupted. Premature Bye.")

    # rospy.spin()


if __name__ == "__main__":
    main()