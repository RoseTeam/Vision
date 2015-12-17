#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start marker tracking using Senz3D camera "
echo "---- Usage : $ start_tracking.bash "
echo "---- Help : $ start_tracking.bash help "
echo "---------------------------------------------------------------"

if [ -z "$1" ]; then

    echo "- Setup development environment"
    source devel/setup.bash

    echo "- Launch camera and ar-track-alvar"
    cd launch
    roslaunch softkinetic_senz3D_camera.launch ar_track_indiv_no_kinect.launch marker_navigation.launch

elif [ "$1" == "help" ]; then

    echo "Help. Script executes the following tasks :"
    echo " 1) Setup catkin workspace environment : source devel/setup.bash"
    echo " 2) Launch softkinetic camera : roslaunch softkinetic_camera softkinetic_senz3d_camera"
    echo " 3) Launch ar-track-alvar launch script : roslaunch launch/ar_track_indiv_no_kinect.launch"

fi
