#!/usr/bin/env python
"""
Ros Bridge node for carla simulator experiment

Allows to specify a starting position
"""

import rospy

from carla.client import make_carla_client
from bridge_experiment import CarlaRosBridgeExperiment
from bridge_with_rosbag_experiment import CarlaRosBridgeWithBagExperiment


def main():
    rospy.init_node("carla_client", anonymous=True)

    params = rospy.get_param('carla')
    host = params['host']
    port = params['port']

    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=host, port=port))

    with make_carla_client(host, port) as client:
        rospy.loginfo("Connected")

        start_position = 10

        bridge_cls = CarlaRosBridgeWithBagExperiment if rospy.get_param(
             'rosbag_fname', '') else CarlaRosBridgeExperiment
        with bridge_cls(client=client, params=params, start_position=start_position) as carla_ros_bridge:
            rospy.on_shutdown(carla_ros_bridge.on_shutdown)
            carla_ros_bridge.run()


if __name__ == "__main__":
    main()
