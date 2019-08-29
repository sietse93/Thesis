#!/bin/bash
BASESTRING="/home/sietse/results_carla0.9/VansOppositeRoad/T3_SL%i_%s/T3_SL%i_%s" 
 
for SL in  75 97 127 132    
do
	for SC in "d10"
	do
		FILENAME=$(printf $BASESTRING $SL $SC $SL $SC)
		echo $FILENAME
		rosparam set /path_to_bag "$FILENAME" 
		rosparam set /path_to_vocabulary /home/sietse/ORB_SLAM2/Vocabulary/ORBvoc.txt
		rosparam set /path_to_settings /home/sietse/ORB_SLAM2/Examples/Stereo/CARLA.yaml

		for i in 8 9 0 1 2 3 4 
		do 	
			echo "$i" 
			MP4="$FILENAME"_$i.mp4	
			echo "$MP4"
			echo "$FILENAME" 
			rosparam set /n_orb "$i"
			if [ $i -lt 5 ] 
			then
				gnome-terminal -e "ffmpeg -video_size 1920x1080 -framerate 10 -f x11grab -i :0.0 $MP4"
			fi
			gnome-terminal -e "rosrun ORB_SLAM2 Stereo" 
			sleep 15
			rosbag play $FILENAME.bag 
			rosnode kill /RGBD
			kill -INT $(pgrep ffmpeg) 
		 done
	done
done
