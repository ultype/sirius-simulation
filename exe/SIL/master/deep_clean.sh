#!/bin/bash
set -x
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/SIL\/master//g')
make clean
cd ../slave
make clean
$SIRIUS_HOME_PATH/exe/xil_common/script/clean_script.sh $SIRIUS_HOME_PATH/exe/SIL/
set +x