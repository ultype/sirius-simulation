#!/bin/bash
set -e
SIL_MASTER_PATH="$PWD"
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/SIL\/master//g')

trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_test/Hapith-1.cpp &

cd ../slave
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_test/Hapith-1.cpp

# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0
