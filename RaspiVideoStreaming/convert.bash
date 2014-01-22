#1/bin/bash

avconv -i /home/chris/video.h264 -c:v libx264 -crf 22 -c:a copy /home/chris/Code/Code/robot_student/RaspiVideoStreaming/output_video.mkv
