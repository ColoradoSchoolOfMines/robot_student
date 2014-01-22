#!/bin/bash

avconv -i /home/chris/video.h264 -c:v libx264 -crf 22 -c:a copy output.mp4
