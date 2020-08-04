#!/bin/bash

params=`rosparam list`

echo "Clearing rosparams ..."

for param in $params; do
  if [ $param == "/rosdistro" ]; then
    continue
  elif [ $param == "/rosversion" ]; then
    continue
  elif [ $param == "/run_id" ]; then
    continue
  elif [[ $param == *"/roslaunch"* ]]; then
    continue
  fi
  rosparam delete $param
done

