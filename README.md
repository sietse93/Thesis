# Thesis
Comparing different SLAM algorithms in dynamic environments

## Running an experiment 
Two windows required: a server and a client

### Window 1: Server
Activate server with this bash script in the compiled game directory
'''
./CarlaServer.sh
'''

The bash script contains the following command
'''./CarlaUE4.sh -carla-server -benchmark -fps=40 -windowed -ResX=600 -ResY=600'''
where 
'''-benchmark -fps=40 '''
allows to establish fps for synchronous mode. 

'''-windowed -ResX=600 -ResY=600'''
without this command, full screen will be used. 

### window 2: client 
'''roslaunch carla_experiment stereo_dynamic_ApOn.launch'''

there are several launch files that start up different client configurations
'''stereo_dynamic_ApOn.launch'''
stereo camera setup, dynamic environment, autopilot is enabled and logged

'''stereo_dynamic_ApOff.launch'''
dynamic environment, uses the control.txt command files in home directory

'''stereo_dynamic_debug.launch'''
autopilot is on, but the controls are not logged 

'''stereo_static_ApOff.launch''' 
static environment, uses the control.txt as control input

