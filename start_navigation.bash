#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start navigation with markers 					 "
echo "---- Usage : $ start_navigation.bash "
echo "---- Help : $ start_navigation.bash help "
echo "---------------------------------------------------------------"

if [ -z "$1" ]; then


	#echo "- Launch camera in another terminal"
	#gnome-terminal -e "bash start_senz3D.bash"

    echo "- Setup development environment"
    source devel/setup.bash

    echo "- Launch camera, ar-track-alvar and marker_navigation"
    cd launch
    roslaunch softkinetic_senz3D_camera.launch ar_track_indiv_no_kinect.launch marker_navigation.launch

elif [ "$1" == "help" ]; then

    echo "Help. Start marker "  
	echo "Script executes the following tasks :"
    echo " 1) Setup catkin workspace environment : source devel/setup.bash"
    echo " 2) Launch softkinetic camera : roslaunch softkinetic_camera softkinetic_senz3d_camera"
    echo " 3) Launch ar-track-alvar launch script : roslaunch launch/ar_track_indiv_no_kinect.launch"

fi
