#!/usr/bin/env bash

# COMPILE CODE AND START THE SIMULATION WITH TERMINAL FOR BOTH ROBOTS #

MAIN_ROBOT_DIR=/media/DATA/eurobotics2015_software/maindspic
SECONDARY_ROBOT_DIR=/media/DATA/eurobotics2015_software/secondary_robot

if [ "$1" == "" ] 
then
echo "Please select the code you want to compile: main/sec"
exit
fi

if [ "$1" == "sec" ] 
then
cd $SECONDARY_ROBOT_DIR
else
cd $MAIN_ROBOT_DIR
fi

if [ "$2" == "clean" ]
then
make H=1 clean
make H=1 mrproper
fi

#make H=1 -s
colormake H=1 -s
python $MAIN_ROBOT_DIR/display.py &
gnome-terminal --title="Secondary Robot" --tab -e "bash -c $MAIN_ROBOT_DIR/main H=1" gnome-terminal --title="Main Robot" --tab -e "bash -c $SECONDARY_ROBOT_DIR/main H=1" 

