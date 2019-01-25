"""
RosBridge class with rosbag support
"""

import time

from tf2_msgs.msg import TFMessage
import rosbag
import rospy
import os

from bridge_experiment import CarlaRosBridgeExperiment


class CarlaRosBridgeWithBagExperiment(CarlaRosBridgeExperiment):
    def __init__(self, client, params, start_position, distance_experiment, *args, **kwargs):
        super(CarlaRosBridgeWithBagExperiment, self).__init__(client, params, start_position, distance_experiment, *args, **kwargs)

        # Bag file is created in the users home directory
        # Automatic name generated based on Number of Vehicles and the Seed used
        NV = params.get('NumberOfVehicles', None)
        SV = params.get('SeedVehicles', None)
        home_user = os.path.expanduser('~')
        filename = home_user + "/SL_{}_NV_{}_SV_{}.bag"
        filename = filename.format(self.start, NV, SV)
        self.bag = rosbag.Bag(filename, mode='w')

    def send_msgs(self):
        for publisher, msg in self.msgs_to_publish:
            self.bag.write(publisher.name, msg, self.cur_time)

        tf_msg = TFMessage(self.tf_to_publish)
        self.bag.write('tf', tf_msg, self.cur_time)
        super(CarlaRosBridgeWithBagExperiment, self).send_msgs()

    def __exit__(self, exc_type, exc_value, traceback):
        rospy.loginfo("Closing the bag file")
        self.bag.close()
        super(CarlaRosBridgeWithBagExperiment, self).__exit__(exc_type, exc_value,
                                                    traceback)
