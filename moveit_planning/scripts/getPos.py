import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
import numpy as np

class posGedit:
    def __init__(self):
        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callBack)
        self.pub = rospy.Publisher("/kuka_arm/target_pos", Pose, queue_size=10)
        self.target_pos = Pose()
        self.target_twist = Twist()

    def callBack(self, msg):
        targetInd = msg.name.index('box::link')
        self.target_pos = msg.pose[targetInd]
        # print(self.target_pos)

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
    