#!/bin/bash

echo "rostopic publishing to /move_base_simple/goal ..."
#rostopic pub -1 move_base_simple/goal geometry_msgs/PoseStamped "
rostopic pub move_base_simple/goal geometry_msgs/PoseStamped "
header:
  frame_id: map
pose:
  position:
    x: 337.148162842
    y: -220.664672852
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.944409018108
    w: -0.328772879838"

