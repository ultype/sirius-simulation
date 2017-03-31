#!/bin/bash
set -e

trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden-20170317-rcs-fc.py
python ../../../tools/generate_error.py RUN_golden/golden-20170317-rcs-fc.csv RUN_golden/log_rocket_csv.csv -l
python ../../../tools/ci_test.py result.csv 1e-5
