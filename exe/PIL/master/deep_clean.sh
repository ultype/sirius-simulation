#!/bin/bash
set -x
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/PIL\/master//g')
make clean
cd ../slave
make clean
$SIRIUS_HOME_PATH/exe/xil_common/script/clean_script.sh $SIRIUS_HOME_PATH/exe/PIL/

sed -i -e "s/\([0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\)/127.0.0.1/g" $SIRIUS_HOME_PATH/models/icf/include/icf_trx_ctrl.h

set +x