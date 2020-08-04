#!/bin/bash

echo "rostopic publishing to /$1 ..."
rostopic pub $1 std_msgs/Bool "data: true"

