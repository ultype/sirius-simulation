#!/bin/bash
set -x
LOCAL_FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SIRIUS_HOME_PATH=$(echo $LOCAL_FILE_DIR | sed 's/\/exe//g')
cd $SIRIUS_HOME_PATH/exe/HIL/master
make clean
cd $SIRIUS_HOME_PATH/exe/HIL/slave
make clean
cd $SIRIUS_HOME_PATH/exe/HIL/cannon_sample_code
make clean
cd $SIRIUS_HOME_PATH/exe/PIL/master
make clean
cd $SIRIUS_HOME_PATH/exe/PIL/slave
make clean
cd $SIRIUS_HOME_PATH/exe/SIL/master
make clean
cd $SIRIUS_HOME_PATH/exe/SIL/slave
make clean
$SIRIUS_HOME_PATH/exe/xil_common/script/clean_script.sh $SIRIUS_HOME_PATH/exe/
set +x