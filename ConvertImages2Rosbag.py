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

#
def CreateStereoBag(left_imgs, right_imgs, bagname):
    '''Creates a bag file containing stereo image pairs'''
    bag =rosbag.Bag(bagname, 'w')
    t = 0.0

    try:
        for i in range(len(left_imgs)):
            print("Adding %s" % left_imgs[i])
            fp_left = open(left_imgs[i], "r")
            p_left = ImageFile.Parser()

            while 1:
                s = fp_left.read(1024)
                if not s:
                    break
                p_left.feed(s)

            im_left = p_left.close()

            fp_right = open( right_imgs[i], "r" )
            print("Adding %s" % right_imgs[i])
            p_right = ImageFile.Parser()

            while 1:
                s = fp_right.read(1024)
                if not s:
                    break
                p_right.feed(s)

            im_right = p_right.close()

            # fps=10
            t = t + 0.1

            Stamp = rospy.rostime.Time.from_sec(t)
            # Stamp = rospy.rostime.Time.from_sec(time.time())

            Img_left = Image()
            Img_left.header.stamp = Stamp
            Img_left.width = im_left.size[0]
            Img_left.height = im_left.size[1]
            # convert RGBA to RGB
            Img_left_data = RGBA2RGB(im_left)
            Img_left.encoding = "rgb8"
            Img_left.header.frame_id = "camera/left"
            # pdb.set_trace()
            # Img_left_data = [pix for pixdata in im_left.getdata() for pix in pixdata]
            Img_left.data = Img_left_data
            Img_right = Image()
            Img_right.header.stamp = Stamp
            Img_right.width = im_right.size[0]
            Img_right.height = im_right.size[1]
            Img_right_data = RGBA2RGB(im_right)
            Img_right.encoding = "rgb8"
            Img_right.header.frame_id = "camera/right"
            # Img_right_data = [pix for pixdata in im_right.getdata() for pix in pixdata]
            Img_right.data = Img_right_data

            bag.write('camera_left/image_raw', Img_left, Stamp)
            bag.write('camera_right/image_raw', Img_right, Stamp)
    finally:
        bag.close()


def RGBA2RGB(image):
    im_np = np.array(image)
    im_np = im_np[:, :, :3]
    flat_im = im_np.flatten()
    rgb_list = flat_im.tolist()
    return rgb_list

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