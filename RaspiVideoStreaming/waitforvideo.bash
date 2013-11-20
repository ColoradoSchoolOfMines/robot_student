#!/bin/bash

ncat -u -l -p 8000 | tee video.h264 | mplayer -cache 8192 -vo gl -demuxer lavf -
