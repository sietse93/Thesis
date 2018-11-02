# My very first CARLA pythonCLient
# GOAL: Connect to CARLA server, make the vehicle drive from a chosen start position to a chosen end position

from __future__ import print_function

import argparse
import logging
import time

from carla.client import make_carla_client #Context manager
from carla.sensor import Camera
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError

host = 'localhost'
port = 2000
out_filename_format = '_out/{:s}/{:0>6d}' #add string and 6 digit number

def run_carla_client():
    with make_carla_client(host, port) as client: #make_carla_client(localhost, port)
        print('CarlaClient connected')
        settings = CarlaSettings()
        settings.set(
            SynchronousMode = True,
            SendNonPlayerAgentsInfo = True,
            QualityLevel = 'Low',
            PlayerVehicle = None,
            NumberOfVehicles = 5,
            NumberOfPedestrians =3,
            WeatherId =1,
            DisableTwoWheeledVehicles = True,
            SeedVehicles = 123456789,
            SeedPedestrians = 123456789
        )

        Camera0 = Camera('CameraRGB')
        Camera0.set_image_size(800, 600)
        Camera0.set_position(2.0, 0.0, 1.4)
        Camera0.set_rotation(0.0, 0.0, 0.0)
        Camera0.set_FOV=90
        settings.add_sensor(Camera0)
        print('loading settings')
        scene = client.load_settings(settings)
        client.start_episode(15) #location of start position vehicle

        #Export a log file of location of all agents

        player_log = open("player_location.txt","w")
        vehicle_log = open("vehicle_locations.txt", "w")
        pedestrian_log= open("pedestrian_locations.txt","w")
        timestamp_log= open("experiment_time","w")

        for frame in range(0, 3600): # note these are the amount of frames that you play
            measurements, sensor_data = client.read_data() # Important otherwise it just shuts off
            player_location = measurements.player_measurements.transform.location
            player_rotation = measurements.player_measurements.transform.rotation
            playerstringtemp = '{: d}   {:6.2f}      {:6.2f}        {:6.2f}   {:6.2f}      {:6.2f}        {:6.2f}\n'
            playerstring = playerstringtemp.format(measurements.frame_number,
                        player_location.x,
                        player_location.y,
                        player_location.z,
                        player_rotation.pitch,
                        player_rotation.yaw,
                        player_rotation.roll
                        )
            player_log.write(playerstring)

            agentstringtemp = '{: d}     {: d}     {:6.2f}       {:6.2f}     {:6.2f}\n'
            for agent in measurements.non_player_agents:
                if agent.HasField('vehicle'):
                    vehiclestring=agentstringtemp.format(measurements.frame_number,
                                                            agent.id,
                                                            agent.vehicle.transform.location.x,
                                                            agent.vehicle.transform.location.y,
                                                            agent.vehicle.transform.location.z)
                    vehicle_log.write(vehiclestring)
                elif agent.HasField('pedestrian'):
                    pedestrianstring=agentstringtemp.format(measurements.frame_number,
                                                            agent.id,
                                                            agent.vehicle.transform.location.x,
                                                            agent.vehicle.transform.location.y,
                                                            agent.vehicle.transform.location.z)
                    pedestrian_log.write(pedestrianstring)
            # export data from sensors
            for name, measurement in sensor_data.items():
                filename= out_filename_format.format(name, frame)
                measurement.save_to_disk(filename)

            control = measurements.player_measurements.autopilot_control
            timestringtemp= '{: d}\n'
            timestring= timestringtemp.format(measurements.game_timestamp)
            timestamp_log.write(timestring)
            client.send_control(control)
        player_log.close()
        vehicle_log.close()
        pedestrian_log.close()
        timestamp_log.close()

def main():

    log_level = logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', host, port)




    while True:
        try:
            run_carla_client()
            print('Done.')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)

if __name__=='__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


