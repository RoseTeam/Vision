#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start Qt Creator to work with a ros package ----"
echo "---- Usage : $ dev_with_qtcreator package_folder_path      ----"
echo "---------------------------------------------------------------"

if [ -z "$1" ]; then
    echo "No argument supplied. Usage : $ dev_with_qtcreator package_folder"
    exit 1
fi

# setup ros environment
if [ -d /opt/ros/jade ]; then 
	source /opt/ros/jade/setup.bash
fi

if [ -d /opt/ros/indigo ]; then 
	source /opt/ros/indigo/setup.bash
fi



echo "Package folder path : " $1 

if [ -d $1 ]
then
    echo "Start qtcreator $1/CMakeLists.txt"
    qtcreator $1/CMakeLists.txt
else	
    echo "Error! Package folder path is not found !"
    exit 1
fi 



