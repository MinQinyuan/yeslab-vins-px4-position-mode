#!/bin/bash
# DSH-083 Step E : live on-screen readout for the position_mode.png evidence screenshot
source /opt/ros/noetic/setup.bash 2>/dev/null
while true; do
  clear
  echo "=========== DSH-083  Step E : PX4 POSITION MODE (POSCTL) ==========="
  date
  echo
  echo "[ /mavros/state ]"
  timeout 3 rostopic echo -n1 /mavros/state 2>/dev/null | grep -E 'connected|armed|mode|system_status'
  echo
  echo "[ /mavros/local_position/pose  (EKF2, driven by External Vision / VINS) ]"
  timeout 3 rostopic echo -n1 /mavros/local_position/pose 2>/dev/null | sed -n '/position:/,/orientation:/p' | head -5
  echo
  echo "[ /mavros/local_position/velocity_local ]"
  timeout 3 rostopic echo -n1 /mavros/local_position/velocity_local 2>/dev/null | sed -n '/linear:/,/angular:/p' | head -5
  echo
  echo "[ /mavros/vision_pose/pose  (EV input from VINS) ]"
  timeout 3 rostopic echo -n1 /mavros/vision_pose/pose 2>/dev/null | sed -n '/position:/,/orientation:/p' | head -5
  sleep 2
done
