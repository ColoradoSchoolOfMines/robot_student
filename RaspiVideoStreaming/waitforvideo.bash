#!/bin/bash

ncat -l -p 8000 | tee video.h264 | mplayer -cache 256 -vo gl -demuxer lavf -
