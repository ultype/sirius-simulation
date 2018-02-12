#!/bin/bash
RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
LIGHTBLUE='\033[1;34m'
NC='\033[0m' # No Color

set -e
if env | grep -q ^WORKSPACE=
then
    echo finding slave from $WORKSPACE
else
    export WORKSPACE=`pwd`/../../../
fi

trick-CP
echo -e "${ORANGE} [Sirius] Please trigger the FSW manually...${NC}"
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp

python ../../../tools/generate_error.py ../../../public/golden.csv RUN_golden/log_rocket_csv.csv -l
python ../../../tools/ci_test.py result.csv 1e-5 | tee test_result


# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0
