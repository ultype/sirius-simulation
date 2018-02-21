#!/bin/bash
set -e
SIL_MASTER_PATH="$PWD"
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/SIL\/master//g')

trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp &

cd ../slave
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.py

cd ../master
python $SIRIUS_HOME_PATH/tools/generate_error.py $SIRIUS_HOME_PATH/public/golden.csv $SIL_MASTER_PATH/RUN_golden/log_rocket_csv.csv -l
python $SIRIUS_HOME_PATH/tools/ci_test.py $SIL_MASTER_PATH/result.csv 1e-5 | tee test_result

# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0
