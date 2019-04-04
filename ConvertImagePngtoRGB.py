import os
import cv2

home_user = os.path.expanduser('~')
f = open(home_user+"/carla/PythonAPI/output_test_carla/left/052141.png", 'r')
image = f.read()
# print(image)
