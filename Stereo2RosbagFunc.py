import os
from ros import rosbag
import roslib
import rospy
import numpy as np
import time
import pdb
from sensor_msgs.msg import Image
roslib.load_manifest('sensor_msgs')
import cv2
from cv_bridge import CvBridge

# sys.path.append('/home/sietse/carla/PythonAPI')

def Stereo2Rosbag(dir, file_sys, fps):
    '''Creates the actual bag file by successively adding images'''
    all_imgs, left_imgs, right_imgs = GetFilesFromDir(dir)
    if len(all_imgs) <= 0:
        print("No images found in {}".format(dir))
        exit()

    # list of integers of the images that are missing
    missing_im_left = MissingImages(left_imgs)
    missing_im_right = MissingImages(right_imgs)

    # if no images are missing create Stereo Rosbag
    if len(missing_im_left) == 0 and len(missing_im_right) == 0:
        print("No missing images")
        # Remove first image, so spawning of vehicle is not registered
        left_imgs.pop(0)
        right_imgs.pop(0)
        if len(left_imgs) > 0 and len(right_imgs) > 0:
            # create bagfile with stereo camera image pairs
            CreateStereoBag(dir, file_sys, left_imgs, right_imgs, fps)
        else:
            print("No stereo images")
    else:
        print("Images are missing \n"
              "Left images({}): {} \n"
              "Right images({}): {} \n".format(len(missing_im_left), [im for im in missing_im_left],
                                               len(missing_im_right), [im for im in missing_im_right]))


def CreateStereoBag(dir, file_sys, left_imgs, right_imgs, fps):
    '''Creates a bag file containing stereo image pairs '''

    bagname = dir + file_sys + ".bag"
    bag = rosbag.Bag(bagname, 'w')
    t = 0.0
    bridge = CvBridge()

    try:
        print("writing rosbag")
        for i in range(len(left_imgs)):
            # print("Adding %s" % left_imgs[i])
            img_cv_left = cv2.imread(left_imgs[i])
            Img_left = bridge.cv2_to_imgmsg(img_cv_left, encoding="bgr8")

            # print("Adding %s" % right_imgs[i])
            img_cv_right = cv2.imread(right_imgs[i])
            Img_right = bridge.cv2_to_imgmsg(img_cv_right, encoding="bgr8")

            # TO DO: IF IMAGE IS SKIPPED THAN THE HEADER TIME SHOULD BE DIFFERENT
            # fps=10
            t = t + 1/fps

            Stamp = rospy.rostime.Time.from_sec(t)

            Img_left.header.stamp = Stamp
            Img_left.header.frame_id = "camera/left"

            Img_right.header.stamp = Stamp
            Img_right.header.frame_id = "camera/right"

            bag.write('camera_left/image_raw', Img_left, Stamp)
            bag.write('camera_right/image_raw', Img_right, Stamp)
    finally:
        bag.close()


def MissingImages(file_list):
    """ Checks whether the sensor got all the images """

    missing_files = []
    int_list = []

    # convert file_list to an integer list
    for file in file_list:
        file_name = file.split("/")[-1]
        file_int = int(file_name[:-4])
        int_list.append(file_int)

    counter = int_list[0]

    # checks if the list is a continuous stream of images
    while counter != int_list[-1]:
        try:
            int_list.index(counter)
        except ValueError:
            missing_files.append(counter)
        counter = counter + 1

    return missing_files


def GetFilesFromDir(dir):
    dir = dir + "/images"

    all = []
    left_files = []
    right_files = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', '.ppm']:
                    if 'left' in f or 'left' in path:
                        left_files.append(os.path.join(path, f))
                    elif 'right' in f or 'right' in path:
                        right_files.append(os.path.join(path, f))
                    all.append(os.path.join(path, f))

    left_files_sorted = SortFiles(left_files)
    right_files_sorted = SortFiles(right_files)
    all.sort()

    return all, left_files_sorted, right_files_sorted


def SortFiles(file_list):
    """sorting strings will results that /1000.png is sorted before /900.png.
    This function sorts the strings based on integers"""

    int_list = []
    for file in file_list:
        splitfile = file.split("/")

        # select only the name of the image int.png
        im_name = splitfile[-1]
        # remove png
        int_list.append(int(im_name[:-4]))
    int_array = np.array(int_list)
    sort_index = np.argsort(int_array).tolist()
    sorted_files = [file_list[index] for index in sort_index]
    return sorted_files


def main():
    # Towns = (1, 2, 3)
    Town = 1
    fps = 20.0
    nr_of_vans = 10
    # for Town in Towns:
    if Town == 1:
        starting_locations = (27, 0, 58)
    elif Town == 2:
        starting_locations = (18, 37, 78)
    elif Town == 3:
        # starting_locations = (75, 97, 127, 132)
        starting_locations = (127, 132)
    else:
        print("Town does not exist")
        return
    starting_locations = [27]
    # scenarios = ("static", "dynamic")
    scenario = "static"
    dynamic_scenarios = (20, 15, 10)
    # scenario = "static"
    home_user = os.path.expanduser('~')
    base_dir = home_user + "/results_carla0.9/stuckbehindvan/test"
    for SL in starting_locations:
        # for scenario in scenarios:
        if scenario == "dynamic":
            start = time.time()
            file_sys = "/T{}_SL{}_{}{}".format(Town, SL, scenario[0], nr_of_vans)
            print("converting {}".format(file_sys))
            dir = base_dir + file_sys
            Stereo2Rosbag(dir, file_sys, fps)
            end = time.time()
            print("Converting time is {}".format(end-start))
        else:
            start = time.time()
            file_sys = "/T{}_SL{}_{}".format(Town, SL, scenario[0])
            print("converting {}".format(file_sys))
            dir = base_dir + file_sys
            Stereo2Rosbag(dir, file_sys, fps)
            end = time.time()
            print("Converting time is {}".format(end - start))

if __name__ == "__main__":
    main()
