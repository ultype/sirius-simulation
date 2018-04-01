#!/bin/bash

MASTER_PATH="$PWD"
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/SIL\/master//g')
source $SIRIUS_HOME_PATH/exe/xil_common/script/color_text.sh
set -e

trick-CP
echo -e "${ORANGE} [Sirius] Please trigger the FSW manually...${NC}"
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp

python $SIRIUS_HOME_PATH/tools/generate_error.py $SIRIUS_HOME_PATH/public/golden.csv $MASTER_PATH/RUN_golden/log_rocket_csv.csv -l
python $SIRIUS_HOME_PATH/tools/ci_test.py $MASTER_PATH/result.csv 1e-5 | tee test_result


# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0
