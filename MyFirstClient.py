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
            SendNonPlayerAgentsInfo = False,
            QualityLevel = 'Low',
            PlayerVehicle = None,
            NumberOfVehicles = 0,
            NumberOfPedestrians =0,
            WeatherId =1,
            DisableTwoWheeledVehicles = True
        )

        Camera0 = Camera('CameraRGB')
        Camera0.set_image_size(800, 600)
        Camera0.set_position(2.0, 0.0, 1.4)
        Camera0.set_rotation(0.0, 0.0, 0.0)
        settings.add_sensor(Camera0)
        print('loading settings')
        scene = client.load_settings(settings)
        client.start_episode(15) #location of start position vehicle
        with open("player_location.txt", "w+") as player_log:
            somestring = '{: f}      {: f}        {: f}\n'
            for frame in range(0, 3600): # note these are the amount of frames that you play
                measurements, sensor_data = client.read_data() # Important otherwise it just shuts off
                newstring = somestring.format(measurements.player_measurements.transform.location.x,
                                  measurements.player_measurements.transform.location.y,
                                  measurements.player_measurements.transform.location.z)
                #player_log.write(str(measurements.player_measurements.transform.location.x) + '\n')
                player_log.write(newstring)
                for name, measurement in sensor_data.items():
                    filename= out_filename_format.format(name, frame)
                    measurement.save_to_disk(filename)

                control = measurements.player_measurements.autopilot_control

                client.send_control(control)

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


