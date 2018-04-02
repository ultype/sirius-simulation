#!/bin/bash
set -e
MASTER_PATH="$PWD"
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/SIL\/master//g')

trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden_dm.cpp &

cd ../slave
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden_fc.cpp

cd ../master
python $SIRIUS_HOME_PATH/tools/generate_error.py $SIRIUS_HOME_PATH/public/golden.csv $MASTER_PATH/RUN_golden/log_rocket_csv.csv -l
python $SIRIUS_HOME_PATH/tools/ci_test.py $MASTER_PATH/result.csv 1e-5 | tee test_result

# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0
