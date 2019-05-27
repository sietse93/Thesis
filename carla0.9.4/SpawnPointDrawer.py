
import glob
import os
import sys

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# import numpy as np
import math

import random
import time
import pdb
sys.path.append('/home/sietse/carla/PythonAPI')

def main():
    # all the actors in the world. For destroying later.
    actor_list = []
    # pygame.init()

    # create client
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    # access world from client
    world = client.get_world()


    try:
        # set weather conditions
        world.set_weather(carla.WeatherParameters.ClearNoon)
        lt = -1



        # define location
        map = world.get_map()
        # Note spawn_points are Transform objects not Waypoint objects
        spawn_points = map.get_spawn_points()

        # Visualize the spawn locations in the world
        # allows you to draw stuff in the world. Arrows and shit
        debug = world.debug

        # list of strings that are drawn in the world

        for index, transform in enumerate(spawn_points):
            w_loc = transform.location
            waypoint = map.get_waypoint(w_loc)
            w_rid = waypoint.road_id
            w_lid = waypoint.lane_id
            # draws the waypoint index number in the world. Allows you to select way points

            debug.draw_string(w_loc + carla.Location(z=0.25),
                              "Waypoint index: {} \n"
                              "road id: {} \n"
                              "lane id: {} \n".format(index, w_rid, w_lid),
                              False,
                              carla.Color(255, 0, 0), life_time=lt, persistent_lines=False)

        # tracks time and frame rate management class.
        clock = pygame.time.Clock()

        while True:
            clock.tick()
            world.tick()
            world.wait_for_tick()

    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        # pygame.quit()
        print("pygame quit, done")


if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        # pygame.quit()

        print("pygame quit")
        print('\nCancelled by user. Bye!')
