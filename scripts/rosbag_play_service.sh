#!/bin/bash

nodename=`rosnode list | grep '/play_' `
svcname=/pause_playback

service=$nodename$svcname
value=$1
echo "rosservice calling thru $service with a value of $value ..."
rosservice call $service $value

# rosservice call /play_xxx/pause_playback  1  # pause
# rosservice call /play_xxx/pause_playback  0  # resume
