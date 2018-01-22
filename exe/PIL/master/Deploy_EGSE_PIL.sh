#!/bin/bash
set -e
if env | grep -q ^WORKSPACE=
then
    echo finding slave from $WORKSPACE
else
    export WORKSPACE=`pwd`/../../../
fi

#####  Remove the trick communication   #####
sed -i -e 's/new_slave->sim_path = std::string(std::getenv("WORKSPACE")) + "\/exe\/PIL\/slave";/\ /g' $WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp
sed -i -e 's/new_slave->S_main_name = ".\/S_main_Linux_5.4_x86_64.exe";/\ /g' $WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp
sed -i -e 's/new_slave->run_input_file = "RUN_golden\/golden.py";/\ /g' $WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp
sed -i -e 's/new_slave->sync_error_terminate = 1;/\ /g' $WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp
sed -i -e 's/trick_master_slave.master.add_slave(new_slave);/\ /g' $WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp
sed -i -e 's/trick_master_slave.master.enable();/\ /g' $WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp
sed -i -e 's/("initialization") transceiver.initialize_connection("ENVIRONMENT");/\ /g' $WORKSPACE/exe/PIL/master/S_define
##### Set up the ESPS IP#####
sed -i -e 's/127.0.0.1/192.168.0.9/g' $WORKSPACE/models/icf/src/icf_trx_ctrl.c

##### Generate the image#####
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp
python ../../../tools/generate_error.py ../../../public/golden.csv RUN_golden/log_rocket_csv.csv -l
python ../../../tools/ci_test.py result.csv 1e-5 | tee test_result

# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0