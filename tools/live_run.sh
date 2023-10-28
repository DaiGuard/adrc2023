#! /bin/env bash

if [[ $# < 1 ]]; then
    echo -e "Usage:"
    echo -e "  ./live_run.sh MODE"
    echo -e ""
    echo -e "  MODE: "
    echo -e "    start MODEL_PATH"
    echo -e "    stop"
    echo -e ""
    echo -e "Example:"
    echo -e "  ./live_run.sh start models/model_weight.pth"
    echo -e "  ./live_run.sh stop"
    exit
fi

# 環境設定
USERNAME="dai_guard"
TARGETNAME="adrc.local"
TARGETDIR=/home/$USERNAME/Workspaces/adrc2023/

MODE=$1

if [[ "$MODE" == "start" ]]; then
    ssh $USERNAME@$TARGETNAME \
        "cd $TARGETDIR && \
        screen -d -S live_run -m ./scripts/run_live_run.sh $2"
elif [[ "$MODE" == "stop" ]]; then
    ssh $USERNAME@$TARGETNAME \
        "cd $TARGETDIR && \
        screen -S live_run -X quit"
fi

# ssh $USERNAME@$TARGETNAME \
#     "cd $TARGETDIR/adrc_ws && \
#     source /opt/ros/foxy/setup.bash && \
#     source install/local_setup.bash && \
#     export ROS_DOMAIN_ID=1 && \
#     ros2 launch data_capture data_capture.launch.py path:=$1 timespan:=$2"


# ssh $USERNAME@$TARGETNAME \
#     "cd $TARGETDIR/adrc_ws && \
#     screen -r data_capture -m \
#     source /opt/ros/foxy/setup.bash && \
#     source install/local_setup.bash && \
#     export ROS_DOMAIN_ID=1 && \
#     ros2 launch data_capture data_capture.launch.py path:=$1 timespan:=$2"

