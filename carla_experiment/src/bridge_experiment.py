"""
RosBridge class for SLAM experiment

Runs a Carla session either with the autopilot on and logging the control
or autopilot off and using these logged controls. Session stops when specified
distance is reached.

"""

import rospy
import os
import math
import sys
from rosgraph_msgs.msg import Clock
from carla_ros_bridge.bridge import CarlaRosBridge
from carla.settings import CarlaSettings


class CarlaRosBridgeExperiment(CarlaRosBridge):
    def __init__(self, client, params, start_position, distance_experiment):
        # These parameters are needed for CarlaRosBridge
        CarlaRosBridge.__init__(self, client, params)

        # These are new properties for the experiment
        # Fixed starting position
        self.start = start_position

        # Fix distance that should be traveled
        self.dist_exp = distance_experiment

        # This will contain the control data which is either written or read
        self.control_data = {}

        # This will contain the id, timestamp, location of dynamic agents
        # Note this will only be used when autopilot is on
        self.da_data = {}

        # This will only be used if control data needs to be read
        self.log_control_gamestamp = []
        self.log_control_steer = []
        self.log_control_throttle = []
        self.log_control_brake = []
        self.log_control_hand_brake = []
        self.log_control_reverse = []

        # This will contain the ground truth pose data
        self.gt_data = {}

    def setup_carla_client(self, client, params):
        # redefine setup_client so the environment is not randomized.
        self.client = client
        self.param_sensors = params.get('sensors', {})
        self.carla_settings = CarlaSettings()
        self.carla_settings.set(
            SendNonPlayerAgentsInfo=params.get('SendNonPlayerAgentsInfo', True),
            NumberOfVehicles=params.get('NumberOfVehicles', 20),
            NumberOfPedestrians=params.get('NumberOfPedestrians', 40),
            WeatherId=params.get('WeatherId', 1),
            SynchronousMode=params.get('SynchronousMode', True),
            QualityLevel=params.get('QualityLevel', 'Low'),
            SeedVehicles=params.get('SeedVehicles', 123456789),
            SeedPedestrians=params.get('SeedPedestrians', 123456789),
            DisableTwoWheeledVehicles=params.get('DisableTwoWheeledVehicles', True)
        )

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
                floatLine = [float(element) for element in line_list[0:4]]
                handbrake_reverse = [element for element in line_list[4:6]]
                self.log_control_gamestamp.append(floatLine[0])
                self.log_control_steer.append(floatLine[1])
                self.log_control_throttle.append(floatLine[2])
                self.log_control_brake.append(floatLine[3])
                # Weird written I know, but now you convert the string "False" to a boolean
                self.log_control_hand_brake.append("True" == handbrake_reverse[0])
                self.log_control_reverse.append("True" == handbrake_reverse[1])
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
            # log dynamic agent ground truth
            if rospy.get_param('carla_autopilot', True):
                control = measurements.player_measurements.autopilot_control
                control_log_line = '{} {} {} {} {} {}\n'.format(self.carla_game_stamp,
                                                                    control.steer,
                                                                    control.throttle,
                                                                    control.brake,
                                                                    control.hand_brake,
                                                                    control.reverse)
                self.control_data.write(control_log_line)
                self.client.send_control(control)

                # log location all dynamic agents for situation labeling
                for agent in measurements.non_player_agents:

                    # NOTE: add pedestrian statement if these will be in experiments
                    if agent.HasField('vehicle'):
                        agent_location = agent.vehicle.transform.location
                        da_log_line = '{} {} {} {}\n'.format(self.carla_game_stamp,
                                                             agent.id,
                                                             agent_location.x,
                                                             agent_location.y)
                        self.da_data.write(da_log_line)

            # handle control: use logged control commands if autopilot is false
            else:
                # use logged control if autopilot is false
                # find the actual timestamp
                time_index = self.log_control_gamestamp.index(self.carla_game_stamp)
                steer_control = self.log_control_steer[time_index]
                throttle_control = self.log_control_throttle[time_index]
                brake_control = self.log_control_brake[time_index]
                hand_brake_control = self.log_control_hand_brake[time_index]
                reverse_control = self.log_control_reverse[time_index]
                self.client.send_control(steer=steer_control,
                                         throttle=throttle_control,
                                         brake=brake_control,
                                         hand_brake=hand_brake_control,
                                         reverse=reverse_control)

            # handle groundtruth logging:
            location = measurements.player_measurements.transform.location
            rotation = measurements.player_measurements.transform.rotation
            if rospy.get_param('log_gt', True):
                gt_log_line = "{} {} {} {} {} {} {}\n".format(measurements.game_timestamp,
                                                             location.x,
                                                             location.y,
                                                             location.z,
                                                             rotation.roll,
                                                             rotation.pitch,
                                                             rotation.yaw)
                self.gt_data.write(gt_log_line)

            # Compute distance travelled
            try:
                dx = location.x - prev_x
                dy = location.y - prev_y
                distance = distance + math.sqrt(dx**2 + dy**2)
                rospy.loginfo(distance)
            # UnboundLocalError occurs when prev_x does not exist
            except UnboundLocalError:
                distance = 0
            prev_x = location.x
            prev_y = location.y

            # Stop experiment if the traveled distance is larger than the experiment distance
            if distance >= self.dist_exp:
                sys.exit()

    def __enter__(self):
        # Using this method so you don't use with .. as functions outside the while loop
        # Since using with .. as inside the loop will only log one line.
        # Any other log files can be added here.

        # Create a name filling system that names file to the starting location (SL)
        # number of vehicles (NV) and seed number (SV)
        # Will be used by the ground truth logger
        home_user = os.path.expanduser('~')
        params = rospy.get_param('carla')
        NV = params.get('NumberOfVehicles', None)
        SV = params.get('SeedVehicles', None)
        filename = home_user+"/SL_{}_NV_{}_SV_{}".format(self.start, NV, SV)

        # Log the controls of the autopilot if autopilot is on else read the logged autopilot commands
        # We are not saving it under a specific scenario since (in theory) the autopilot is going to be set once.
        # It is difficult to specify which autopilot needs to be used (could be set in a launch file though...)
        if rospy.get_param('carla_autopilot', True):
            self.control_data = open(home_user + "/control.txt", 'w')
            self.da_data = open(filename + "_da.txt", 'w')
            rospy.loginfo("Opened control and dynamic agents file to write")
        else:
            self.control_data = open(home_user + "/control.txt", 'r')
            rospy.loginfo("Opened control file to read")

        # Log the ground truth if the parameter is true
        if rospy.get_param('log_gt', True):
            self.gt_data = open(filename + "_gt.txt", 'w')
            rospy.loginfo("Opened ground truth file to write")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.control_data.close()
        self.da_data.close()
        rospy.loginfo("Closed control and dynamic agent file")

        if rospy.get_param('log_gt', True):
            self.gt_data.close()
            rospy.loginfo("Closed groundtruth file")
        CarlaRosBridge.__exit__(self, exc_type, exc_val, exc_tb)
