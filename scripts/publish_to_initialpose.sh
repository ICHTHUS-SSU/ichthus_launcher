#!/bin/bash

echo "rostopic publishing to /initialpose ..."
#rostopic pub -1 initialpose geometry_msgs/PoseWithCovarianceStamped "
rostopic pub initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    position: 
      x: 202.885482788
      y: -188.513656616
      z: 3.22967529297
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.960707274782
      w: 0.277563564215
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]" 


