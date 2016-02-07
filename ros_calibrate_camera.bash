#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start calibration procedure for Senz3D camera "
echo "---- Usage : $ ros_calibrate_camera.bash [grid N] [calibrate|calibrate_dev|check [calibration_file]], N={1,2}"
echo "---------------------------------------------------------------"


function help() {
	echo "To calibrate : $ ros_calibrate_camera.bash grid [1 or 2] calibrate"
	echo "To calibrate using my dev package: $ ros_calibrate_camera.bash grid [1 or 2] calibrate_dev"
	echo "To check : $ ros_calibrate_camera.bash grid [1 or 2] check <calibration_file>"	
}


if [ -z "$1" ]; then
    echo "No argument supplied. "
	help
    exit 1
fi

# define grids
grid_1="--size 7x4 --square 0.045"
grid_2="--size 8x6 --square 0.0435"


# Setup grid
if [ $1 != "grid" ]; then 
	echo "First argument should be 'grid'"
	help
	exit 1
fi

if [ -z "$2" ]; then 
	echo "Second argument should be 1 or 2"
	help
	exit 1	
fi

if [ "$2" == "1"  ]; then
	grid=${grid_1}
elif [ "$2" == "2"  ]; then 
	grid=${grid_2}
else 
	echo "Wrong argument for the grid_id"
	help
	exit 1
fi

echo "Selected grid : ${grid}"


# Setup operation 
if [ -z $3 ]; then
	echo "No third argument supplied"
	help
	exit 1
fi


if [ "$3" == "calibration" ]; then

	rosrun_command="camera_calibration cameracalibrator.py ${grid} image:=/softkinetic_camera/rgb/image_mono camera:=/softkinetic_camera --no-service-check"

	camera_command="bash start_senz3D.bash"

elif [ "$3" == "calibration_dev" ]; then

	# setup local projects 
	source devel/setup.bash

	rosrun_command="custom_camera_calibration cameracalibrator.py ${grid} image:=/softkinetic_camera/rgb/image_mono camera:=/softkinetic_camera --no-service-check"

	camera_command="bash start_senz3D.bash"

elif [ "$3" == "check" ]; then

	rosrun_command="camera_calibration cameracheck.py ${grid} monocular:=/softkinetic_camera"

	if [ -z $4 ]; then 
		echo "Fourth argument should specify calibration yaml file"
		help
		exit 1
	else
		calib_file=`pwd -P`/${4}
	fi  	
	
	camera_command="bash start_senz3D.bash output_rgb_image_color:=image_rect output_rgb_camera_info:=camera_info rgb_calibration_file:=${calib_file}"


else
	echo "Third argument should be 'calibration', 'calibration_dev' or 'check'"
	help
	exit 1
fi



echo "- Launch camera in another terminal"
gnome-terminal -e "${camera_command}"

echo "- Start python script"
rosrun ${rosrun_command}

exit 0


