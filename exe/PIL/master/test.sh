#!/bin/bash
set -e
if env | grep -q ^WORKSPACE=
then
    echo finding slave from $WORKSPACE
else
    export WORKSPACE=`pwd`/../../../
fi

cd ../slave
trick-CP

cd ../master
trick-CP

./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp
python ../../../tools/generate_error.py ../../../public/golden.csv RUN_golden/log_rocket_csv.csv -l
python ../../../tools/ci_test.py result.csv 1e-5
