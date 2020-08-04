#!/bin/bash

rosparam set tf_x 1.2
rosparam set tf_y 0.0
rosparam set tf_z 2.0
rosparam set tf_roll 0.0
rosparam set tf_yaw 0.0
rosparam set tf_pitch 0.0
rosparam set localizer velodyne
rosparam set /use_sim_time true

rosparam list

