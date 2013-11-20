#!/bin/bash

while true
do
    raspivid --vflip -t 30000 -o - | tee video.h264 | nc -u 192.168.43.247 8000
    if [ $? == 130 ]
    then
        exit 130
    fi
done
