#!/usr/bin/env sh
export ROS_IP=`ip address | grep "192.168.11" -B 2 | sed -e '/wl/,+3d' | grep "192.168.11" | tail -n1 | awk -F' ' '{print $2}' | awk -F'/' '{print $1}'`
if [ "${ROS_IP}" == "" ]; then # use wireless ip only if there is no wired connection
    export ROS_IP=`ip address | grep "192.168.11" | tail -n1 | awk -F' ' '{print $2}' | awk -F'/' '{print $1}'`
fi
export ROS_MASTER_URI=http://192.168.11.100:11311
export IPC_PUBSUB_SUBSCRIBER_MODES=ros
export IPC_PUBSUB_PUBLISHER_MODES=ros
export GLOG_log_dir=/tmp
export CAMERA_TOPIC_SUFFIX=/compressed
export SHOW_DISPARITY=false
