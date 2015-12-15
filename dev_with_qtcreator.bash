#!/bin/bash

echo "---------------------------------------------------------------"
echo "---- Script to start Qt Creator to work with a ros package ----"
echo "---- Usage : $ dev_with_qtcreator package_folder           ----"
echo "---------------------------------------------------------------"

if [ -z "$1" ]; then
    echo "No argument supplied. Usage : $ dev_with_qtcreator package_folder"
    exit 1
fi

source /opt/ros/jade/setup.bash

echo "Package folder : " $1 

if [ -d src/$1 ]
then
    echo "Start qtcreator src/$1/CMakeLists.txt"
    qtcreator src/$1/CMakeLists.txt
else	
    echo "Error! Package folder is not found !"
    exit 1
fi 



