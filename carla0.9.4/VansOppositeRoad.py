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
    SL = 0
    fps = 20.0
    speed = 15.0
    total_distance = 500

    v = speed / 3.6
    x_step = v * (1 / fps)

    actor_list = []

    # create client
    client = carla.Client('localhost', 2000)
    # set timeout so client can connect
    client.set_timeout(10.0)

    print("connected to client")

    # get world
    world = client.get_world()

    # activate synchronous mode
    print('Enable synchronous mode')
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)

    # set world settings
    try:
        # set weather conditions
        world.set_weather(carla.WeatherParameters.ClearNoon)

        # get all spawn point locations and choose one for hero transform
        map = world.get_map()

        # get all the blueprints
        bp_lib = world.get_blueprint_library()

        # define hero
        prius_bp = bp_lib.find('vehicle.toyota.prius')
        prius_bp.set_attribute('color', '255, 255, 255')

        # define other agent
        van_bp = bp_lib.find('vehicle.carlamotors.carlacola')

        # define a spawn location
        spawn_points = map.get_spawn_points()
        hero_spawn = spawn_points[SL]
        hero_waypoint = map.get_waypoint(hero_spawn.location)
        hero_route = [hero_waypoint]

        total_dist = 0
        # the hero's route defined before the simulation is started
        while total_dist < total_distance:
            last_waypoint = hero_route[-1]
            next_waypoint = last_waypoint.next(x_step)[0]
            next_transform = next_waypoint.transform
            this_transform = last_waypoint.transform
            dtrav = next_transform.location - this_transform.location
            total_dist += math.sqrt(dtrav.x**2 + dtrav.y**2 + dtrav.z**2)
            hero_route.append(next_waypoint)

        hero = world.spawn_actor(prius_bp, hero_spawn)
        actor_list.append(hero)
        hero.set_simulate_physics(False)

        clock = pygame.time.Clock()

        hero_route_index = 0

        # actual simulation loop
        while hero_route_index < len(hero_route)-1:
            clock.tick(fps)
            world.tick()
            world.wait_for_tick()

            next_trans = hero_route[hero_route_index+1].transform
            batch = [carla.command.ApplyTransform(hero.id, next_trans)]
            hero_route_index += 1

            client.apply_batch(batch)

    finally:
        print("Disable Synchronous Mode")
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        print("Destroy actor")
        for actor in actor_list:
            actor.destroy()
        print("Actors destroyed")


if __name__ == '__main__':
    main()

