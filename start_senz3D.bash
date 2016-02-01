#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start calibration procedure for Senz3D camera "
echo "---- Usage : $ calibrate_camera.bash "
echo "---------------------------------------------------------------"



echo "- Setup development environment"
source devel/setup.bash

echo "- Launch camera in background"
cd launch
roslaunch softkinetic_senz3D_camera.launch "$@"

