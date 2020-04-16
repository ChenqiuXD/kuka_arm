#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
import numpy as np
import tf

class posGedit:
    def __init__(self):
        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callBack)
        self.pub = rospy.Publisher("/kuka_arm/target_pos", Pose, queue_size=10)
        self.target_pos = Pose()
        self.base_pos = Pose()

    def callBack(self, msg):
        # Get the base pose in gazebo
        # base_index = msg.name.index('kuka_arm::base_link')
        # self.base_pos = msg.pose[base_index]
        # IMPORTANT: By experimenting, the base of robot is at the origin in the gazebo world. 
        # Thus the target_pos is indeed the transform between the target and the robot_base

        # Get the target pose in gazebo
        target_index = msg.name.index('box::link')
        self.target_pos = msg.pose[target_index]

def main():
    rospy.init_node("get_pose", anonymous=True)
    pG = posGedit()
    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            pG.pub.publish(pG.target_pos)
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
    