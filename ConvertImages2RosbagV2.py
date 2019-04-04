import time, sys, os
from ros import rosbag
import roslib
import rospy
from sensor_msgs.msg import Image
from PIL import ImageFile
roslib.load_manifest('sensor_msgs')

# for debugging
import pdb

import numpy as np
import cv2
from cv_bridge import CvBridge


def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
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
                        right_files.append( os.path.join( path, f ) )
                    all.append( os.path.join( path, f ) )
    all.sort()
    left_files.sort()
    right_files.sort()
    return all, left_files, right_files


def CreateStereoBag(left_imgs, right_imgs, bagname):
    '''Creates a bag file containing stereo image pairs'''
    bag =rosbag.Bag(bagname, 'w')
    t = 0.0
    bridge = CvBridge()

    try:
        for i in range(len(left_imgs)):
            print("Adding %s" % left_imgs[i])
            img_cv_left = cv2.imread(left_imgs[i])
            Img_left = bridge.cv2_to_imgmsg(img_cv_left, encoding="rgb8")

            print("Adding %s" % right_imgs[i])
            img_cv_right = cv2.imread(right_imgs[i])
            Img_right = bridge.cv2_to_imgmsg(img_cv_right, encoding="rgb8")

            # fps=10
            t = t + 0.1

            Stamp = rospy.rostime.Time.from_sec(t)

            Img_left.header.stamp = Stamp
            Img_left.header.frame_id = "camera/left"

            Img_right.header.stamp = Stamp
            Img_right.header.frame_id = "camera/right"

            bag.write('camera_left/image_raw', Img_left, Stamp)
            bag.write('camera_right/image_raw', Img_right, Stamp)
    finally:
        bag.close()

def CreateMonoBag(imgs,bagname):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])
            fp = open( imgs[i], "r" )
            p = ImageFile.Parser()

            while 1:
                s = fp.read(1024)
                if not s:
                    break
                p.feed(s)

            im = p.close()

            Stamp = rospy.rostime.Time.from_sec(time.time())
            Img = Image()
            Img.header.stamp = Stamp
            Img.width = im.size[0]
            Img.height = im.size[1]
            Img.encoding = "rgb8"
            Img.header.frame_id = "camera"
            Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            Img.data = Img_data

            bag.write('camera/image_raw', Img, Stamp)
    finally:
        bag.close()


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs, left_imgs, right_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()

    if len(left_imgs) > 0 and len(right_imgs) > 0:
        # create bagfile with stereo camera image pairs
        CreateStereoBag(left_imgs, right_imgs, args[1])
    else:
        # create bagfile with mono camera image stream
        CreateMonoBag(all_imgs, args[1])


if __name__ == "__main__":
    if len(sys.argv) == 3:
        CreateBag(sys.argv[1:])
    else:
        print("Usage: img2bag, imagedir bagfilename")