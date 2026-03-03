#!/bin/bash

# Recording @720p
libcamera-vid -t 0 --width 1280 --height 720 --framerate 80 -o $HOME/Videos/flight_$(data +"%d%m_%H%M%S").mp4 &
