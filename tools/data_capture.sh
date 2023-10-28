#! /bin/env bash

if [ $# -ne 2 ]; then
    echo -e "Usage:"
    echo -e "  ./data_capture.sh PATH TIMESTAMP"
    echo -e ""
    echo -e "Example:"
    echo -e "  ./data_capture.sh data/test/ 0.3"

    exit
fi

# 環境設定
USERNAME="dai_guard"
TARGETNAME="adrc.local"
TARGETDIR=/home/$USERNAME/Workspaces/adrc2023/

ssh $USERNAME@$TARGETNAME \
    "cd $TARGETDIR/adrc_ws && \
    source /opt/ros/foxy/setup.bash && \
    source install/local_setup.bash && \
    export ROS_DOMAIN_ID=1 && \
    ros2 launch data_capture data_capture.launch.py path:=$1 timespan:=$2"