#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node("try_pub")
    listener = tf.TransformListener()

    try:
        listener.waitForTransform("/base_link", '/target_pos',rospy.Time(), rospy.Duration(4))
        (tran, rot) = listener.lookupTransform('/base_link', '/target_pos', rospy.Time(0))
        print(tran)
        print(rot)
    except BaseException, error:
        print(str(error))
