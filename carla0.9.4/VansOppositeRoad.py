import glob
import os
import sys
import time

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

        # define the route of the hero first
        while total_dist < total_distance:
            last_waypoint = hero_route[-1]
            next_waypoint = last_waypoint.next(x_step)[0]
            next_transform = next_waypoint.transform
            this_transform = last_waypoint.transform
            dtrav = next_transform.location - this_transform.location
            total_dist += math.sqrt(dtrav.x**2 + dtrav.y**2 + dtrav.z**2)
            hero_route.append(next_waypoint)

        # now define the route on the opposite lane, based on hero's route
        opposite_route = []

        # get all waypoints from the map
        map_waypoint_list = map.generate_waypoints(x_step)

        # note: map_waypoint_list does not contain waypoints on intersections
        indices_intersection = []
        for index, hero_waypoint in enumerate(hero_route):
            # print progress
            print(float(index)/float(len(hero_route))*100.0)

            # get relevant road information from hero waypoint
            hero_loc = hero_waypoint.transform.location
            hero_road_id = hero_waypoint.road_id
            hero_lane_id = hero_waypoint.lane_id
            opp_lane_id = hero_lane_id * -1

            # initialize value for closest waypoint
            closest_waypoint = map_waypoint_list[0]

            # check whether the current hero waypoint is at the beginning of an intersection
            if hero_waypoint.is_intersection is True:
                indices_intersection.append(index)
                closest_waypoint = 0
            else:
                # check for each hero waypoint the closest opposite waypoint in the generated waypoint list.
                # THIS SHOULD BE MORE EFFICIENTLY PROGRAMMED
                for map_waypoint in map_waypoint_list:
                    map_road_id = map_waypoint.road_id
                    map_lane_id = map_waypoint.lane_id
                    map_loc = map_waypoint.transform.location
                    # check if it is on the same road but on the opposite lane
                    if map_road_id == hero_road_id and map_lane_id == opp_lane_id:
                        ref_loc = closest_waypoint.transform.location
                        d_map = hero_loc - map_loc
                        d_ref = hero_loc - ref_loc

                        map_dist = math.sqrt(d_map.x ** 2 + d_map.y ** 2 + d_map.z ** 2)
                        ref_dist = math.sqrt(d_ref.x ** 2 + d_ref.y ** 2 + d_ref.z ** 2)

                        if map_dist < ref_dist:
                            closest_waypoint = map_waypoint
            opposite_route.append(closest_waypoint)

        # opposite_route contains zeros at intersections.
        # find the start of the intersection on the opposite route
        opposite_route_start_intersection_indices = [index for index, waypoint in enumerate(opposite_route) if opposite_route[index-1] == 0 and waypoint != 0]

        # Opposite_route_start_intersection is an index that is a waypoint, but it is still an intersection.
        # This means the road id is wrong. You need the next waypoint
        # this for loop is to ensure when multiple intersections occur
        print("COUNTERACT FOR INTERSECTIONS")
        for start_intersection_index_og in opposite_route_start_intersection_indices:
            # first ensure that although the hero_waypoint is not an intersection anymore, the opposite route is also
            # not an intersection anymore. If it still is an intersection go to the next index
            start_intersection_index = start_intersection_index_og
            while opposite_route[start_intersection_index].is_intersection is True:
                start_intersection_index = start_intersection_index_og + 1
            # now start_intersection_index contains the first waypoint that is not an intersection on the opposite road.
            while start_intersection_index != start_intersection_index_og:
                # TO DO: THE CHOICE OF THE WAYPOINTS IN THE LIST SHOULD ENSURE THAT THE CORRECT TURN IS MADE
                opposite_next_waypoint = opposite_route[start_intersection_index].next(x_step)[0]
                start_intersection_index -= 1
                opposite_route[start_intersection_index] = opposite_next_waypoint

            # Now we are at the point where the hero waypoint is off the intersection and we go back on the intersection
            # for both hero and opposite car. These indices are stored in the indices_intersection list
            opposite_intersection_index = start_intersection_index_og
            opposite_next_waypoint = opposite_route[opposite_intersection_index].next(x_step)[0]
            opposite_intersection_index -= 1
            while opposite_intersection_index in indices_intersection:
                print("while opposite_intersection_index in indices_intersection:")
                opposite_route[opposite_intersection_index] = opposite_next_waypoint
                opposite_intersection_index -= 1
                opposite_next_waypoint = opposite_next_waypoint.next(x_step)[0]








        pdb.set_trace()



        hero = world.spawn_actor(prius_bp, hero_spawn)
        actor_list.append(hero)
        hero.set_simulate_physics(False)
        van_transform = opposite_route[0].transform
        van_transform.location.z += 2.0
        van = world.spawn_actor(van_bp, van_transform)
        actor_list.append(van)
        van.set_simulate_physics(False)

        clock = pygame.time.Clock()

        hero_route_index = 0

        # actual simulation loop
        while hero_route_index < len(hero_route)-1:
            clock.tick(fps)
            world.tick()
            world.wait_for_tick()
            print(hero_route_index+1)
            next_trans = hero_route[hero_route_index+1].transform
            batch = [carla.command.ApplyTransform(hero.id, next_trans)]

            next_trans_van = opposite_route[hero_route_index+1].transform
            batch += [carla.command.ApplyTransform(van.id, next_trans_van)]
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

