#!/bin/bash
set -e
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/HIL\/slave//g')
##### Variable #####
if [ -z $1 ]; then
    echo "No arguments supplied"
    EGSE_IP="127.0.0.1"
else
    EGSE_IP=$1
fi

if [[ $EGSE_IP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "EGSE_IP: "$EGSE_IP
else
  echo "Remote EGSE IP Input format fail exit..."
  exit
fi

##### FUNCTION #####
source $SIRIUS_HOME_PATH/exe/xil_common/script/text_process_func.sh
#####  Set up communication IP   #####
sed_ipaddr_subst $EGSE_IP "$SIRIUS_HOME_PATH/models/icf/include/icf_trx_ctrl.h"
##### Generate the image#####
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp
