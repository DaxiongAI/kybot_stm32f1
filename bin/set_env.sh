#!/bin/bash

# directory of shell script, not equal `pwd`
#CDIR=$(pwd)/${0}

# some global config for program running.
# It's source in 
#       ../out/arm/debug/make/set_env.sh
# or
#       ../out/arm/release/make/set_env.sh
chip_type=stm32f1x
adapter=stlink-v2

export openocd_dir=openocd-0.9.0
#export arm_exec_name="openocd -f interface/$adapter.cfg -f target/$chip_type.cfg"
export arm_exec_name="flash write_image erase /home/`whoami`/alpha_tdroid_sw/out/arm/debug/build/bin/alpha_tdroid_base.hex"
#export ARM_BOARD_IP=10.0.1.103
#export USER=`whoami`
#export ARM_EXEC_NAME="nice -n 19 ./ftu -u"
#export ARM_EXEC_NAME='./ftu -u -f="open_close"'
#export ARM_EXEC_NAME="./ftu -u"

