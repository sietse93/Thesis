"""
RosBridge class for SLAM experiment

Runs a Carla session either with the autopilot on and logging the control
or autopilot off and using these logged controls.

"""

import rospy
from rosgraph_msgs.msg import Clock
from carla_ros_bridge.bridge import CarlaRosBridge


class CarlaRosBridgeExperiment(CarlaRosBridge):
    def __init__(self, client, params, start_position):
        # These parameters are needed for CarlaRosBridge
        CarlaRosBridge.__init__(self, client, params)

        # These are new properties for the experiment
        # Fixed starting position
        self.start = start_position

        # This will contain the control data which is either written or read
        self.control_data = {}

        # This will only be used if control data needs to be read
        self.log_control_gamestamp = []
        self.log_control_steer = []
        self.log_control_throttle = []
        self.log_control_brake = []

    def run(self):
        self.publishers['clock'] = rospy.Publisher(
            "clock", Clock, queue_size=10)

        # load settings into the server
        scene = self.client.load_settings(self.carla_settings)
        self.client.start_episode(self.start)
        rospy.loginfo("we are running")

        # load control commands if autopilot is turned off
        if not rospy.get_param('carla_autopilot', True):
            for line_data in self.control_data:
                line_list = line_data.split(" ")
                floatLine = [float(element) for element in line_list[0:-1]]
                self.log_control_gamestamp.append(floatLine[0])
                self.log_control_steer.append(floatLine[1])
                self.log_control_throttle.append(floatLine[2])
                self.log_control_brake.append(floatLine[3])
            rospy.loginfo(" control commands loaded ")

        while not (rospy.is_shutdown()):
            measurements, sensor_data = self.client.read_data()

            # handle time
            self.carla_game_stamp = measurements.game_timestamp
            self.cur_time = rospy.Time.from_sec(self.carla_game_stamp * 1e-3)
            self.compute_cur_time_msg()

            # handle agents
            self.player_handler.process_msg(
                measurements.player_measurements, cur_time=self.cur_time)
            self.non_players_handler.process_msg(
                measurements.non_player_agents, cur_time=self.cur_time)

            # handle sensors
            for name, data in sensor_data.items():
                self.sensors[name].process_sensor_data(data, self.cur_time)

            # publish all messages
            self.send_msgs()

            # handle control: log control commands if autopilot is true
            if rospy.get_param('carla_autopilot', True):
                control = measurements.player_measurements.autopilot_control
                self.client.send_control(control)
                self.control_data.write('{} {} {} {} \n'.format(self.carla_game_stamp,
                                                                    control.steer,
                                                                    control.throttle,
                                                                     control.brake))
            # handle control: use logged control commands if autopilot is false
            else:
                control = measurements.player_measurements.autopilot_control
                time_index = self.log_control_gamestamp.index(self.carla_game_stamp)
                control.steer = self.log_control_steer[time_index]
                control.throttle = self.log_control_throttle[time_index]
                control.brake = self.log_control_brake[time_index]
                rospy.loginfo(control)
                self.client.send_control(control)

    def __enter__(self):
        # Using this method so you don't use with .. as functions outside the while loop
        # Using with .. as inside the loop will only save one line.
        if rospy.get_param('carla_autopilot', True):
            self.control_data = open("/home/sietse/control.txt", 'w')
            rospy.loginfo("Opened control file to write")
        else:
            self.control_data = open("/home/sietse/control.txt", 'r')
            rospy.loginfo("Opened control file to read")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.control_data.close()
        rospy.loginfo("Closed control file")
        CarlaRosBridge.__exit__(self, exc_type, exc_val, exc_tb)
