#!/bin/bash

# Set Flightmare Path if it is not set
if [ -z $FLIGHTMARE_PATH ]
then
  export FLIGHTMARE_PATH=$PWD/flightmare
fi

# Launch the simulator, unless it is already running
if [ -z $(pgrep visionsim_node) ]
then
  roslaunch envsim visionenv_sim.launch render:=True &
  ROS_PID="$!"
  echo $ROS_PID
  sleep 1
else
  ROS_PID=""
fi

# Publish simulator reset
rostopic pub /kingfisher/dodgeros_pilot/off std_msgs/Empty "{}" --once
rostopic pub /kingfisher/dodgeros_pilot/reset_sim std_msgs/Empty "{}" --once
rostopic pub /kingfisher/dodgeros_pilot/enable std_msgs/Bool "data: true" --once

export ROLLOUT_NAME="rollout_""$i"
echo "$ROLLOUT_NAME"

cd ./envtest/ros/

python3 run_competition.py &
COMP_PID="$!"

cd -

sleep 0.5
rostopic pub /kingfisher/start_navigation std_msgs/Empty "{}" --once

# Wait until the evaluation script has finished
while :
do
  sleep 1
done

if [ $ROS_PID ]
then
  kill -SIGINT "$ROS_PID"
fi
