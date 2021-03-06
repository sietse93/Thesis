import glob
import os
import sys
import errno
import time

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import logging
import random
import pdb

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue

import math
# this allows symbolic links from git in the directory
sys.path.append('/home/sietse/carla/PythonAPI')


def draw_image(surface, image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def main():
    Town = 1
    # Starting_locations = [58, 27, 0]
    scenarios = ["static", "dynamic"]
    distances2van = [20, 15]
    # distances2van = [20, 15, 10]
    # for SL in Starting_locations:
    SL = 27
    scenario = "static"
    #for scenario in scenarios:
        # print(scenario)
    if scenario == "dynamic":
        for distance2van in distances2van:
            print("run simulation")
            start_sim_time = time.time()
            run_simulation(Town, SL, scenario, distance2van)
            end_sim_time = time.time()
            print("One simulation takes {}".format(end_sim_time-start_sim_time))
    else:
        start_sim_time = time.time()
        run_simulation(Town, SL, scenario, distances2van[0])
        end_sim_time = time.time()
        print("One simulation takes {}".format(end_sim_time - start_sim_time))


def run_simulation(Town, SL, scenario, dist2van):
    print("SL = {}, scenario = {}, dist2van = {}".format(SL, scenario, dist2van))
    # input values
    fps = 20.0
    speed = 15.0  # in km/h
    total_distance = 500  # meters

    # Get systematic file extensions
    # TownNumber_StartingLocation_(S)tatic(D)ynamic_DynamicScenario
    home_user = os.path.expanduser('~')
    if scenario == "dynamic":
        file_sys = "/T{}_SL{}_{}{}".format(Town, SL, scenario[0], dist2van)
        print("run dynamic simulation")
    else:
        print("run static simulation")
        file_sys = "/T{}_SL{}_{}".format(Town, SL, scenario[0])

    # A directory per scenario
    dir = home_user + "/results_carla0.9/stuckbehindvan/test" + file_sys

    try:
        os.mkdir(dir)
    except OSError:
        print("directory exists")

    filename = dir + file_sys

    # Calculate required distance between waypoints to simulate certain speed
    v = speed / 3.6
    x_step = v * (1 / fps)

    image_length = 680

    print("try to initialize pygame")
    actor_list = []
    pygame.init()
    print("init pygame")

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

    debug = world.debug


    # logging groundtruth
    gt_filename = filename + "_gt.txt"
    if not os.path.exists(os.path.dirname(gt_filename)):
        try:
            os.makedirs(os.path.dirname(gt_filename))
        except OSError as exc:  # guard against race conditions
            if exc.errno != errno.EEXIST:
                raise
    gt_log = open(gt_filename, 'w')

    try:
        # set weather conditions
        world.set_weather(carla.WeatherParameters.ClearNoon)

        # get all spawn point locations and choose one for hero transform
        map = world.get_map()

        # get all the blueprints
        bp_lib = world.get_blueprint_library()

        # create waypoints with a certain distance between the waypoints
        waypoint_list = map.generate_waypoints(x_step)

        # get prius bp
        prius_bp = bp_lib.find('vehicle.toyota.prius')
        # set color of prius -> white
        prius_bp.set_attribute('color', '255,255,255')

        # note spawn_points are Transform class not Waypoint class
        spawn_points = map.get_spawn_points()
        hero_spawn = spawn_points[SL]

        # Create a route, set destination and get a list of waypoints to get there
        # the simplest route planner I can think of
        begin_waypoint = map.get_waypoint(hero_spawn.location)
        cur_waypoint = begin_waypoint

        # spawn our hero: give blueprint and location:
        hero = world.spawn_actor(prius_bp, hero_spawn)
        # add hero to actor list so it can be destroyed at the end of the session
        actor_list.append(hero)

        # Transforming your vehicle only works when the physics = false,
        # when an image is involved/ pygame is involved
        hero.set_simulate_physics(False)

        # add van if scenario is dynamic
        if scenario == "dynamic":
            van_bp = bp_lib.find('vehicle.carlamotors.carlacola')
            van_waypoint = begin_waypoint.next(dist2van)[0]
            # Note that you cannot change waypoints!! Copy the transforms and use that
            van_transform = van_waypoint.transform
            van_transform.location.z += 2.0
            van = world.spawn_actor(van_bp, van_transform)
            actor_list.append(van)
            van.set_simulate_physics(False)

        # Now add a camera to the hero
        # What blueprint should the sensor have?
        camera_bp = bp_lib.find('sensor.camera.rgb')
        # What attributes are there?
        camera_bp.set_attribute('image_size_x', str(image_length))
        camera_bp.set_attribute('image_size_y', str(image_length))
        camera_bp.set_attribute('fov', '90')
        # don't install a sensor tick, it fucks up sync mode.

        # Where should the camera be placed on the vehicle (in Transform class)
        camera_loc = carla.Transform(carla.Location(x=1.8, z=1.3))

        # Spawn camera in the world: blueprint, transform and attach to which actor

        # camera purely used for visuals in pygame window
        # camera = world.spawn_actor(camera_bp, camera_loc, attach_to=hero)
        # add actor to list
        # actor_list.append(camera)

        # Make sync queue for sensor data.
        # image_queue = queue.Queue()
        # camera.listen(image_queue.put)

        # attach cameras used for stereo vision data
        camera_left = world.spawn_actor(camera_bp, camera_loc, attach_to=hero)
        actor_list.append(camera_left)
        image_left_queue = queue.Queue()
        camera_left.listen(image_left_queue.put)

        camera_loc_right = carla.Transform(carla.Location(x=1.8, y=0.54, z=1.3))
        camera_right = world.spawn_actor(camera_bp, camera_loc_right, attach_to=hero)
        actor_list.append(camera_right)
        image_right_queue = queue.Queue()
        camera_right.listen(image_right_queue.put)

        frame = None
        start_frame = None
        frame_skip_counter = 0

        # variable on which to show the camera images
        # display = pygame.display.set_mode(
        #     (image_length, image_length),
        #     pygame.HWSURFACE | pygame.DOUBLEBUF)
        # # which font to use
        # font = get_font()

        # Now the simulation needs to run, we need
        # a class that tracks time
        clock = pygame.time.Clock()

        dist_travel = 0
        time = 0

        # The loop that allows the simulation
        while dist_travel < total_distance:
            if should_quit():
                return

            clock.tick(fps)
            world.tick()
            # World needs to wait to get the tick otherwise actors are not destroyed
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

            # log ground truth
            # hero_loc = next_waypoint.transform.location
            # hero_rot = next_waypoint.transform.rotation
            hero_loc = hero.get_transform().location
            hero_rot = hero.get_transform().rotation

            gt_line = "{} {} {} {} {} {} {}\n".format(time, hero_loc.x, hero_loc.y, hero_loc.z,
                                                      hero_rot.roll, hero_rot.pitch, hero_rot.yaw)
            gt_log.write(gt_line)

            next_waypoint = cur_waypoint.next(x_step)[0]

            dx = distance_between_waypoints(cur_waypoint, next_waypoint)
            dist_travel = dist_travel + dx

            time = time + 1 / fps

            # hero.set_transform(next_waypoint.transform)
            batch = [carla.command.ApplyTransform(hero.id, next_waypoint.transform)]

            if scenario == "dynamic":
                van_waypoint = van_waypoint.next(x_step)[0]
                batch += [carla.command.ApplyTransform(van.id, van_waypoint.transform)]
                # van.set_transform(van_waypoint.transform)

            # Apply batch transform
            client.apply_batch(batch)



            # print(dist_travel)

            cur_waypoint = next_waypoint

            # draw_image(display, image)
            #
            # text_surface = font.render('% 5d FPS' % clock.get_fps(), True, (255, 255, 255))
            # display.blit(text_surface, (8, 10))
            #
            # pygame.display.flip()

    finally:
        print('Disable synchronous mode')
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        print('Destroy actors')
        for actor in actor_list:
            actor.destroy()
        pygame.quit()
        print('Actors destroyed')
        gt_log.close()


def distance_between_waypoints(waypoint1, waypoint2):
    wp1_loc = waypoint1.transform.location
    wp2_loc = waypoint2.transform.location
    return math.sqrt((wp1_loc.x-wp2_loc.x)**2+(wp1_loc.y-wp2_loc.y)**2+(wp1_loc.z-wp2_loc.z)**2)


if __name__ == '__main__':
    try:
        start = time.time()
        # [1:] gets the complete string
        main()
        end = time.time()
        print("Total time simulation: {}".format(end-start))

    except KeyboardInterrupt:
        pygame.quit()
        print('\nCancelled by user. Bye!')