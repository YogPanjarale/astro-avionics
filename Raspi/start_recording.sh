#!/bin/bash

# Define timestamp format
TIMESTAMP=$(date +"%d%m_%H%M%S")
FILE = "$HOME/Videos/flight_$TIMESTAMP.h264"

# Start video recording in a new process group
libcamera-vid -t 0 --height 1080 --framerate 60 -o $FILE &
