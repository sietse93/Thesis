# Thesis
Evaluating SLAM in urban dynamic environments

## Running experiments in CARLA

Download compiled version of CARLA. Two terminals required: a server and a client.

Activate server with this bash script in the compiled game directory. Run Carla server like this

```
./CarlaServer.sh Town01 -windowed -ResX=600 -ResY=600
```
Make sure you loaded the correct town. 

Simulation code are in directory carla0.9.4. 

Note that everything works definitely in python 2.7. 

IMPORTANT everything is dependent on this strict convention. Each name is dependent on town number, starting location, some dynamic varbiable. e.g. in directory stuckbehindvan T1_SL27_d15, means 15m distance to van in front. T1_SL27_d10 in directory VansOppositeRoad means 10 vehicles are spawned. The complete directory system will be based on this. So all converted files (png to rosbag, txt, json whatever) will be saved to the correct directory. 

Furthermore, the main function of the scripts usually describe what is simulated. If you want to simulate a single trajectory/condition. Change the main function. If you want to simulate static conditions, this is usually possible in the stuckbehindvan main simulation scripts. 

### Run specific scenarios

To run simulation of stuck behind van, there is a completely automated script which runs all scenarios in town 1 and writes out all stereo images to disk. Change values of T# accordingly in the name to simulate different town: 
```
python StuckBehindVanAutomaticT1.py 
```
For VansOppositeRoad the following script only works for town01 and town02. Town03 uses different road_ids numbering so it doesn't work with this script. 

```
python VansOppositeRoad.py
```

In the end you will the scenario directory will have a ground truth txt file and stereo images .png files
## Convert .png to rosbag
change in the main function which files you want to convert. 

```
Stereo2RosbagFunc.py
```

In the end scenario directory will have a rosbag file. 

## Running ORB_SLAM2 

So if you want to run ORB SLAM2, download it from github and put the files that from this repository in the correct folder from ORB_SLAM2 and the recompile it. There is a bash script which shows how to run it. This also records everything.

```
./automatic_orb_record.sh
```

In the end scenario directory will have a number of pose estimation files .txt

## Data conversion

A lot of data needs to be converted and there needs to be this balance between automating and flexibility. Therefore the following name convention is used: 
main_... are the scripts that utilizes classes and functions
class_... describe the classes that are used in the main functions
func_... describe the functions that are used by classes and main scripts. 

there is a mode convention throughout this pipeline: 
"SLAM" -> Conventional ORB SLAM
"VO" -> bypassed loop closure ORB SLAM
"MC" -> map point culling bypassed 

### Convert txt files into json files

JSON files contains the data but in the same reference frame. Change the main function to specify which files you want to convert. 

```
python main_ConvertRefFrame.py
```

Note that the json describes a class described in python script: class_ConvertRefFrame. 

### Plot to visualize what is happening. 

Visualizes trajectory in 2D, euler angles, the whole mikmak. Also the relative pose error over a small distance. 
```
python main_InspectData.py
```

### Plot the performance

the python script class_ScenarioLocationPerformance.py converts the json files to usable classes that describe the performance. 

example of how to use them in script Results_Scenario1_SLAM.py

