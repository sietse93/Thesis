#!/usr/bin/env python

import rospy

from tf2_msgs.msg import TFMessage

def callback(data):
    length = len(data.transforms)
    for index in range(0, length-1):
        translation = data.transforms[index].transform.translation
        rotation = data.transforms[index].transform.rotation
        rospy.loginfo("%f   %f  %f  %f  %f  %f  %f\n" %
                      (translation.x,
                       translation.y,
                       translation.z,
                       rotation.x,
                       rotation.y,
                       rotation.z,
                       rotation.w))
def logger():

    rospy.init_node('GTLogger', anonymous=True)

    rospy.Subscriber("/tf", TFMessage, callback)

    rospy.spin()


if __name__ == '__main__':

    logger()