#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start calibration procedure for Senz3D camera "
echo "---- Usage : $ calibrate_camera.bash "
echo "---------------------------------------------------------------"



echo "- Setup development environment"
source devel/setup.bash

echo "- Launch camera in background"
cd launch
roslaunch softkinetic_senz3D_camera.launch

#echo "- Start calibration python script"
#rosrun camera_calibration cameracalibrator.py --size 7x4 --square 0.0009 image:=/softkinetic_camera/rgb/image_mono camera:=/softkinetic_camera --no-service-check
