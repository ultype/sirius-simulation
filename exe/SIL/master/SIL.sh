#!/bin/bash
set -e

SCRIPT_FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SIRIUS_HOME_PATH=$(echo $SCRIPT_FILE_DIR | sed 's/\/exe\/SIL\/master//g')
S_DEFINE_PATH=$SCRIPT_FILE_DIR

cd $S_DEFINE_PATH
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden_dm.cpp &

cd $SIRIUS_HOME_PATH/exe/SIL/slave
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden_fc.cpp

cd $S_DEFINE_PATH
python $SIRIUS_HOME_PATH/tools/generate_error.py $SIRIUS_HOME_PATH/public/golden.csv $S_DEFINE_PATH/RUN_golden/log_rocket_csv.csv -l
python $SIRIUS_HOME_PATH/tools/ci_test.py $S_DEFINE_PATH/result.csv 1e-5 | tee test_result

# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0
