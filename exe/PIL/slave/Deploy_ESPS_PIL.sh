#!/bin/bash
set -e
SCRIPT_FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SIRIUS_HOME_PATH=$(echo $SCRIPT_FILE_DIR | sed 's/\/exe\/PIL\/slave//g')
S_DEFINE_PATH=$SCRIPT_FILE_DIR
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
cd $S_DEFINE_PATH
trick-CP
./S_main_Linux_*_x86_64.exe RUN_golden/golden_fc.cpp