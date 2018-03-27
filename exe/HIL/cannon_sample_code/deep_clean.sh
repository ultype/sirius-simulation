#!/bin/bash

make clean
rm -rf DP_Product
rm RUN_test/S_job_execution
rm RUN_test/S_run_summary
rm RUN_test/send_hs
rm RUN_test/varserver_log
rm ftrace_script/logg.txt
rm ftrace_script/output.txt
rm ftrace_script/output1.png
rm ftrace_script/plot_data_input.txt
rm models/icf/QA_Tool/ftrace_script/logg.txt
rm models/icf/QA_Tool/ftrace_script/output.txt
rm models/icf/QA_Tool/ftrace_script/output1.png
rm models/icf/QA_Tool/ftrace_script/plot_data_input.txt
find . -name "*.o" -type f -delete
rm S_default.dat
rm S_main_Linux_5.4_x86_64.exe
rm S_sie.resource
rm S_source.hh
rm -rf build/
rm makefile
rm -rf trick/
rm models/icf/QA_Tool/EGSE_System_Profiling_Server/tx_CAN_without_gpio/sendCan
