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

import numpy as np
import math
try:
    import queue
except ImportError:
    import Queue as queue
import random
import time
import pdb
sys.path.append('/home/sietse/carla/PythonAPI')


def main():
    Town = 2
    starting_locations = [18, 37, 78]
    nr_vans = 10
    scenario = "dynamic"
    for SL in starting_locations:
        start_time = time.time()
        run_simulation(Town, SL, scenario, nr_vans)
        end_time = time.time()
        print("Sim time took: {}".format(end_time-start_time))

def run_simulation(Town, SL, scenario, nr_vans):
    fps = 20.0
    speed = 15.0
    total_distance = 500

    v = speed / 3.6
    x_step = v * (1 / fps)
    image_length = 680

    # Get systematic file extensions
    # TownNumber_StartingLocation_(S)tatic(D)ynamic_nrvans
    home_user = os.path.expanduser('~')
    if scenario == "dynamic":
        file_sys = "/T{}_SL{}_{}{}".format(Town, SL, scenario[0], nr_vans)
        print("run dynamic simulation")
    else:
        print("run static simulation")
        file_sys = "/T{}_SL{}_{}".format(Town, SL, scenario[0])

    # A directory per scenario
    dir = home_user + "/results_carla0.9/VansOppositeRoad" + file_sys

    try:
        os.mkdir(dir)
    except OSError:
        print("directory exists")

    filename = dir + file_sys

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

    # logging groundtruth
    gt_filename = filename + "_gt.txt"
    if not os.path.exists(os.path.dirname(gt_filename)):
        try:
            os.makedirs(os.path.dirname(gt_filename))
        except OSError as exc:  # guard against race conditions
            if exc.errno != errno.EEXIST:
                raise
    gt_log = open(gt_filename, 'w')

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

        # spawn the hero
        hero = world.spawn_actor(prius_bp, hero_spawn)
        actor_list.append(hero)
        hero.set_simulate_physics(False)

        # define stereo system
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(image_length))
        camera_bp.set_attribute('image_size_y', str(image_length))
        camera_bp.set_attribute('fov', '90')
        # don't install a sensor tick, it fucks up sync mode.

        camera_loc = carla.Transform(carla.Location(x=1.8, z=1.3))
        camera_left = world.spawn_actor(camera_bp, camera_loc, attach_to=hero)
        actor_list.append(camera_left)
        image_left_queue = queue.Queue()
        camera_left.listen(image_left_queue.put)

        camera_loc_right = carla.Transform(carla.Location(x=1.8, y=0.54, z=1.3))
        camera_right = world.spawn_actor(camera_bp, camera_loc_right, attach_to=hero)
        actor_list.append(camera_right)
        image_right_queue = queue.Queue()
        camera_right.listen(image_right_queue.put)

        ## Now define the route on the opposite lane, based on hero's route

        if scenario == "dynamic":
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
            # this for loop is to ensure when multiple intersections occur in the trajectory
            print("COUNTERACT FOR INTERSECTIONS")
            for start_intersection_index_og in opposite_route_start_intersection_indices:
                # first ensure that although the hero_waypoint is not an intersection anymore, the opposite route is also
                # not an intersection anymore. If it still is an intersection go to the next index
                start_intersection_index = start_intersection_index_og
                while opposite_route[start_intersection_index].is_intersection is True:
                    start_intersection_index = start_intersection_index_og + 1
                # now start_intersection_index contains the first waypoint that is not an intersection on the opposite road.
                # go down the list until both the hero as the opposite van are on the intersection
                while start_intersection_index != start_intersection_index_og:
                    list_opposite_next_waypoint = opposite_route[start_intersection_index].next(x_step)
                    opposite_next_waypoint = choose_correct_waypoint(list_opposite_next_waypoint, hero_route,
                                                                     start_intersection_index)
                    start_intersection_index -= 1
                    opposite_route[start_intersection_index] = opposite_next_waypoint

                # Now we are at the point where the hero waypoint is off the intersection and we go back on the intersection
                # for both hero and opposite car. These indices are stored in the indices_intersection list
                opposite_intersection_index = start_intersection_index_og
                opposite_next_waypoint_list = opposite_route[opposite_intersection_index].next(x_step)
                opposite_intersection_index -= 1
                opposite_next_waypoint = choose_correct_waypoint(opposite_next_waypoint_list, hero_route,
                                                                 opposite_intersection_index)
                while opposite_intersection_index in indices_intersection:
                    opposite_route[opposite_intersection_index] = opposite_next_waypoint
                    opposite_intersection_index -= 1
                    opposite_next_waypoint_list = opposite_next_waypoint.next(x_step)
                    opposite_next_waypoint = choose_correct_waypoint(opposite_next_waypoint_list, hero_route,
                                                                     opposite_intersection_index)

            # now reverse the route so the vans starts at the other side of the route
            opposite_route.reverse()
            print("ROUTE DEFINED")

            # spawn the vans in a dict construct
            # NOTE:NOT ACTUALLY THE NUMBER OF VEHICLES

            van_dict = {}
            van_index = 0
            spawn_points = range(0, len(opposite_route), len(opposite_route)/(nr_vans-1))
            for i in spawn_points:
                van_transform = opposite_route[i].transform
                van_transform.location.z += 2.0
                van_key = "van{}".format(van_index)
                van_route_index = "van{}_index".format(van_index)
                van_dict[van_key] = world.spawn_actor(van_bp, van_transform)
                van_dict[van_key].set_simulate_physics(False)
                van_dict[van_route_index] = i
                van_index += 1

        frame = None
        start_frame = None
        frame_skip_counter = 0

        clock = pygame.time.Clock()

        hero_route_index = 0

        time = 0

        # actual simulation loop
        while hero_route_index < len(hero_route)-1:
            clock.tick(fps)
            world.tick()
            ts = world.wait_for_tick()

            if frame is not None:
                if ts.frame_count != frame + 1:
                    frame_skip_counter = frame_skip_counter + 1
                    print('frame skip nr: {}'.format(frame_skip_counter))

            frame = ts.frame_count

            while True:
                image_left = image_left_queue.get()
                image_right = image_right_queue.get()
                if image_left.frame_number == ts.frame_count and image_right.frame_number == ts.frame_count:
                    image_left.save_to_disk(dir + '/images/left/{}.png'.format(image_left.frame_number))
                    image_right.save_to_disk(dir + '/images/right/{}.png'.format(image_right.frame_number))
                    break
                else:
                    pdb.set_trace()

            if start_frame is None:
                start_frame = frame

            time = time + 1 / fps

            # next transform for hero
            next_trans = hero_route[hero_route_index+1].transform
            batch = [carla.command.ApplyTransform(hero.id, next_trans)]

            if scenario == "dynamic":
                for i in range(len(spawn_points)):
                    cur_index = van_dict["van{}_index".format(i)]
                    next_index = cur_index + 1
                    van = van_dict["van{}".format(i)]
                    if next_index > len(opposite_route)-1:
                        # if the van is at the end of its route, set the vehicle at the beginning of the route
                        van_dict["van{}_index".format(i)] = 0
                        next_trans_van = opposite_route[0].transform
                        batch += [carla.command.ApplyTransform(van.id, next_trans_van)]
                    else:
                        next_trans_van = opposite_route[next_index].transform
                        van_dict["van{}_index".format(i)] = next_index
                        batch += [carla.command.ApplyTransform(van.id, next_trans_van)]

            hero_route_index += 1

            client.apply_batch(batch)

            # log ground truth
            hero_loc = next_trans.location
            hero_rot = next_trans.rotation

            gt_line = "{} {} {} {} {} {} {}\n".format(time, hero_loc.x, hero_loc.y, hero_loc.z,
                                                      hero_rot.roll, hero_rot.pitch, hero_rot.yaw)
            gt_log.write(gt_line)

    finally:
        print("Disable Synchronous Mode")
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        print("Destroy prius")
        for actor in actor_list:
            actor.destroy()
        if scenario == "dynamic":
            for i in range(nr_vans):
                van = van_dict["van{}".format(i)]
                van.destroy()

        print("Actors destroyed")
        gt_log.close()


def choose_correct_waypoint(waypoint_list, hero_route, current_index):
    if waypoint_list > 1:
        eq_hero_waypoint = hero_route[current_index]
        eq_hero_road_id = eq_hero_waypoint.road_id
        diff_road_id_min = float('Inf')

        for waypoint_index, waypoint in enumerate(waypoint_list):
            waypoint_road_id = waypoint.road_id
            diff_road_id = abs(waypoint_road_id - eq_hero_road_id)
            if diff_road_id < diff_road_id_min:
                chosen_index = waypoint_index
                diff_road_id_min = diff_road_id

    else:
        chosen_index = 0

    chosen_waypoint = waypoint_list[chosen_index]

    return chosen_waypoint


if __name__ == '__main__':
    main()

