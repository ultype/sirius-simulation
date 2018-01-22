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
#####  Roll back the trick communication   #####
##  Master part
#####  Roll back the trick communication   #####
uncomment_the_C_code "new_slave->sim_path" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
uncomment_the_C_code "new_slave->S_main_name" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
uncomment_the_C_code "new_slave->run_input_file" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
uncomment_the_C_code "new_slave->sync_error_terminate" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
uncomment_the_C_code "trick_master_slave" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
uncomment_the_C_code '("initialization") transceiver.initialize_connection' "$WORKSPACE/exe/PIL/master/S_define"
sed -i -e "s/\([0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\)/127.0.0.1/g" $WORKSPACE/models/icf/src/icf_trx_ctrl.c
##  Slave Part
uncomment_the_Python_code "new_connection = trick.MSSocket" "$WORKSPACE/exe/PIL/slave/RUN_golden/golden.py"
uncomment_the_Python_code "trick_master_slave" "$WORKSPACE/exe/PIL/slave/RUN_golden/golden.py"
uncomment_the_C_code '("initialization") transceiver.initialize_connection' "$WORKSPACE/exe/PIL/slave/S_define"

set +x