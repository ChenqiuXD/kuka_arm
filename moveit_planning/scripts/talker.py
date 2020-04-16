#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8

if __name__ == "__main__":
    rospy.init_node("publish_sth")
    talker = rospy.Publisher("some_topic", Int8, queue_size=10)

    loop_rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        talker.publish(i)
        i+=1
        if i%5==0:
            i=0
        print "Outputing data:" + str(i) + " in topic 'some_topic'."
        loop_rate.sleep()