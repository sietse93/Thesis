#!/usr/bin/env python

import rospy

from tf2_msgs.msg import TFMessage


class CarlaGtLogger(object):

    def __init__(self):
        self.fname = rospy.get_param('rosbag_fname')
        self.logger_layout = '%f   %f   %f  %f  %f  %f  %f  %f\n'
        self.gt_logger = {}

    def callback(self, data):
        # data.transforms contain information of all agents
        index = 0
        time = data.transforms[index].header.stamp

        # Convert time from integer to float. Needed to match timestamp with ORB
        game_stamp = time.to_sec()
        translation = data.transforms[index].transform.translation
        rotation = data.transforms[index].transform.rotation
        log_string = (self.logger_layout % (game_stamp,
                                            translation.x,
                                            translation.y,
                                            translation.z,
                                            rotation.x,
                                            rotation.y,
                                            rotation.z,
                                            rotation.w))
        rospy.loginfo(log_string)
        self.gt_logger.write(log_string)

    def logger(self):

        rospy.init_node('GTLogger', anonymous=True)

        rospy.Subscriber("/tf", TFMessage, self.callback)

        rospy.spin()

    def __enter__(self):
        self.gt_logger = open(self.fname + ".txt", "w")
        rospy.loginfo("Gt file is open")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.gt_logger.close()
        rospy.loginfo("Gt file is closed")
        return None


def main():
    with CarlaGtLogger() as gt_log:
        gt_log.logger()


if __name__ == "__main__":
    main()
