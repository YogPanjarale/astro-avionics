#!/bin/bash

# Define timestamp format
TIMESTAMP=$(date +"%d%m_%H%M%S")

# Define output file with timestamp
OUTPUT_FILE="/home/pi/Videos/flight_${TIMESTAMP}.mkv"
PID_FILE="/tmp/recording.pid"

# Start video recording in a new process group
{ libcamera-vid -t 0 --height 720 --framerate 180 | \
  ffmpeg -i - -c:v copy -f matroska "$OUTPUT_FILE"; } & echo $! > "$PID_FILE"

echo "Recording started. File: $OUTPUT_FILE, PID: $(cat $PID_FILE)"