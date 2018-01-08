#!/bin/bash
set -x
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

set +x