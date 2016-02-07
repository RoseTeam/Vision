#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start calibration procedure for Senz3D camera "
echo "---- Usage : $ calibrate_camera.bash [calibrate|check] "
echo "---------------------------------------------------------------"

if [ -z "$1" ]; then
    echo "No argument supplied. "
	echo "To calibrate : $ calibrate_camera.bash calibrate "
	echo "To check : $ calibrate_camera.bash check "
    exit 1
fi


if [ $1 == "calibrate" ]
then
	
	echo "- Launch camera in another terminal"
	gnome-terminal -e "bash start_senz3D.bash"

	echo "- Start calibration python script"
	rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0435 image:=/softkinetic_camera/rgb/image_mono camera:=/softkinetic_camera --no-service-check
elif [ $1 == "check" ]
then

	echo "- Launch camera in another terminal"
	if [ -z "$2" ]; then
		gnome-terminal -e "bash start_senz3D.bash output_rgb_image_color:=image_rect output_rgb_camera_info:=camera_info"
	else
		calib_file=`pwd -P`/${2}
		gnome-terminal -e "bash start_senz3D.bash output_rgb_image_color:=image_rect output_rgb_camera_info:=camera_info rgb_calibration_file:=${calib_file}"			
	fi

	echo "- Start calibration python script"
	rosrun camera_calibration cameracheck.py --size 8x6 --square 0.0435 monocular:=/softkinetic_camera
else
	echo "Arguments $1 is unknown"
	exit 1
fi
