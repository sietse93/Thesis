# Thesis
Comparing different SLAM algorithms in dynamic environments

## Running an experiment in CARLA
Two windows required: a server and a client

### Window 1: Server
Activate server with this bash script in the compiled game directory

```
./CarlaServer.sh
```

The bash script contains the following command
```
./CarlaUE4.sh -carla-server -benchmark -fps=40 -windowed -ResX=600 -ResY=600
```
where 
```
-benchmark -fps=40
```
allows to establish fps for synchronous mode. 

```
-windowed -ResX=600 -ResY=600
```
without this command, full screen will be used. 

### window 2: client 
```
roslaunch carla_experiment stereo_dynamic_ApOn.launch
```

there are several launch files that start up different client configurations
```
stereo_dynamic_ApOn.launch
```
stereo camera setup, dynamic environment, autopilot is enabled and logged, creates rosbag

```
stereo_dynamic_ApOff.launch
```
dynamic environment, uses the control.txt command files in home directory, creates rosbag

```
stereo_dynamic_debug.launch
```
autopilot is on, but the controls are not logged and no rosbag created

```
stereo_static_ApOff.launch
``` 
static environment, uses the control.txt as control input, creates rosbag

Note that the starting location and traveled distance is in the client_experiment.py file

## Create a pose estimation with ORBSLAM
3 windows: roscore, ORB_SLAM2 node and rosbag play

### Window 1
```
roscore
```

### Window 2
```
rosrun ORB_SLAM2 Stereo ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Examples/Stereo/CARLA.yaml false 
```
* Starts a stereo vision node
* loads the ORB vocabulary needed for loop closure detection
* loads the camera configurations from the yaml file
* images are already rectified

### Window 3
```
rosbag play --pause ~/file_location.bag /camera_left/image_raw:=/camera/left/image_raw /camera_right/image_raw:=/camera/right/image_raw
```
topic name needs to be rewritten to make it work
