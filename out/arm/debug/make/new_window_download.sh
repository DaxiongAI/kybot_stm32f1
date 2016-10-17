#!/bin/sh
#if you want to debug this script, append -ex at first line.

pushd $(dirname $(readlink -f "$BASH_SOURCE")) > /dev/null
SCRIPT_DIR="$PWD"
popd > /dev/null

source ./set_env.sh
#echo $ARM_EXEC_NAME
cmd="./login_download.expect "$arm_exec_name""
#cmd="./logging_ut.expect 10.0.1.103 /home/hl/debug ./ftu"
#cmd="./logging_ut.expect 10.0.1.103 /home/hl/debug ./ftu"
#cmd="cd /home/`whoami`/$openocd_dir;$arm_exec_name"
#openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg"

cd $PWD
#tmux -2 attach-session -t arm
tmux kill-window -t :1 &>/dev/null || true
#tmux new-window -k -t :0 -n 'board' 'export $ARM_BOARD_IP'
tmux new-window -k -t :1 "$cmd"
#tmux new-window -k -t :0 -n 'board' 
tmux select-window -t :1

