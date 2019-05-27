
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
        # lt = -1
        x_step = 10.0
        # define location
        map = world.get_map()
        # Note spawn_points are Transform objects not Waypoint objects
        spawn_points = map.get_spawn_points()
        map_waypoint_list = map.generate_waypoints(x_step)
        bp_lib = world.get_blueprint_library()

        # define hero
        prius_bp = bp_lib.find('vehicle.toyota.prius')
        prius_bp.set_attribute('color', '255, 255, 255')

        # define other agent
        van_bp = bp_lib.find('vehicle.carlamotors.carlacola')

        hero_waypoint_list = []
        opposite_waypoint_list = []

        hero_spawn = spawn_points[143]  # note this is a transform object
        hero_waypoint = map.get_waypoint(hero_spawn.location)
        hero_waypoint_list.append(hero_waypoint)

        hero_waypoint_list.append(hero_waypoint.next(x_step)[0])

        # find opposite waypoint
        for hero_waypoint in hero_waypoint_list:
            hero_loc = hero_waypoint.transform.location
            hero_road_id = hero_waypoint.road_id
            hero_lane_id = hero_waypoint.lane_id
            opp_lane_id = hero_lane_id * -1

            closest_waypoint = map_waypoint_list[0]

            for map_waypoint in map_waypoint_list:
                map_road_id = map_waypoint.road_id
                map_lane_id = map_waypoint.lane_id
                map_loc = map_waypoint.transform.location

                if map_road_id == hero_road_id and map_lane_id == opp_lane_id:
                    ref_loc = closest_waypoint.transform.location
                    d_map = hero_loc - map_loc
                    d_ref = hero_loc - ref_loc

                    map_dist = math.sqrt(d_map.x**2 + d_map.y**2 + d_map.z**2)
                    ref_dist = math.sqrt(d_ref.x**2 + d_ref.y**2 + d_ref.z**2)

                    if map_dist < ref_dist:
                        closest_waypoint = map_waypoint

            opposite_waypoint_list.append(closest_waypoint)

        # spawn hero
        hero = world.spawn_actor(prius_bp, hero_spawn)
        actor_list.append(hero)

        van_transform = opposite_waypoint_list[0].transform
        van_transform.location.z += 2.0
        # spawn agent
        van = world.spawn_actor(van_bp, van_transform)
        actor_list.append(van)

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
