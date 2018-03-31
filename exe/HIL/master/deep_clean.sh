#!/bin/bash
set -x
export WORKSPACE=`pwd`/../../../
uncomment_the_Python_code() {
    pattern=$1
    file_path=$2
    sed -i -e "s/\#$pattern/$pattern/g" "$file_path"
}

uncomment_the_C_code() {
    pattern=$1
    file_path=$2
    sed -i -e "s/\/\/$pattern/$pattern/g" "$file_path"
}
% basename "$PWD"
make clean
find . -type d -name "jitlib" -exec rm -rf {} \;
cd ../slave/
make clean
cd ../../../
rm models/icf/QA_Tool/EGSE_System_Profiling_Server/tx_CAN_without_gpio/sendCan
rm models/icf/QA_Tool/ftrace_script/logg.txt
rm models/icf/QA_Tool/ftrace_script/output.txt
rm models/icf/QA_Tool/ftrace_script/output1.png
rm models/icf/QA_Tool/ftrace_script/plot_data_input.txt
rm models/icf/QA_Tool/EGSE_System_Profiling_Server/readRS422
rm models/icf/QA_Tool/EGSE_System_Profiling_Server/server-client/server

sed -i -e "s/\([0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\)/127.0.0.1/g" $WORKSPACE/models/icf/include/icf_trx_ctrl.h

set +x