#!/bin/bash
set -e
if env | grep -q ^WORKSPACE=
then
    echo finding slave from $WORKSPACE
else
    export WORKSPACE=`pwd`/../../../
fi
SIL_MASTER_PATH="$PWD"
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/SIL\/master//g')
cd ../slave
trick-CP

cd ../master
trick-CP

./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp &
sleep 3
cd ../slave
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.py

cd $SIL_MASTER_PATH
python $SIRIUS_HOME_PATH/tools/generate_error.py $SIRIUS_HOME_PATH/public/golden.csv $SIL_MASTER_PATH/RUN_golden/log_rocket_csv.csv -l
python $SIRIUS_HOME_PATH/tools/ci_test.py $SIL_MASTER_PATH/result.csv 1e-5 | tee test_result


# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0
