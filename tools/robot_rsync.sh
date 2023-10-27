#! /bin/env bash

if [ $# -gt 0 ]; then
    MODE=$1
else
    MODE="esp32,ros,scripts"
fi

# 環境設定
USERNAME="dai_guard"
TARGETNAME="adrc.local"
TARGETDIR=/home/$USERNAME/Workspaces/adrc2023/

# ワークディレクトリを取得
WORKDIR=$(cd $(dirname $0)/.. && pwd)

if [[ "$MODE" == *esp32* ]]; then
    echo -e "\e[34msynchronize ESP32 directory \e[m"
    # ESP32ソフトのアップロード
    rsync -rv --exclude=".git" --exclude=".vscode" --exclude=".pio" \
    $WORKDIR/adrc_esp32 \
    $USERNAME@$TARGETNAME:$TARGETDIR
    # ESP32ソフトのマイコン書き込み
    ssh $USERNAME@$TARGETNAME "cd $TARGETDIR/adrc_esp32/ && ~/.local/bin/pio run -t upload"
fi

if [[ "$MODE" == *ros* ]]; then
    echo -e "\e[34msynchronize ROS directory \e[m"
    # ROSソフのアップロード
    rsync -rv --exclude=".git" --exclude=".vscode" --delete \
        $WORKDIR/adrc_ws/src \
        $USERNAME@$TARGETNAME:$TARGETDIR/adrc_ws/
fi

if [[ "$MODE" == *scripts* ]]; then
    echo -e "\e[34msynchronize Scripts directory \e[m"
    # スクリプトのアップロード
    rsync -rv --exclude=".git" --exclude=".vscode" --delete \
        $WORKDIR/scripts \
        $USERNAME@$TARGETNAME:$TARGETDIR
fi



