#!/bin/bash
set -e

##### Variable #####
if env | grep -q ^WORKSPACE=
then
    echo finding slave from $WORKSPACE
else
    export WORKSPACE=`pwd`/../../../
fi

if [ -z $1 ]; then
    echo "No arguments supplied"
    ESPS_IP="192.168.0.9"
else
    ESPS_IP=$1
fi
echo "ESPS IP: "$ESPS_IP
##### FUNCTION #####
comment_the_C_code() {
    pattern=$1
    file_path=$2
    if grep -q "//$pattern" $file_path; then
        return
    else
        sed -i -e "s/$pattern/\/\/$pattern/g" "$file_path"
    fi
}

sed_substitution() {
    patternA=$1
    patternB=$2
    file_path=$3
    sed -i -e "s/$patternA/$patternB/g" "$file_path"
}
sed_ipaddr_subst() {
    ip=$1
    file_path=$2
    sed -i -e "s/\([0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\)/$ip/g" "$file_path"
}

#####  Remove the trick communication   #####
comment_the_C_code "new_slave->sim_path" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
comment_the_C_code "new_slave->S_main_name" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
comment_the_C_code "new_slave->run_input_file" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
comment_the_C_code "new_slave->sync_error_terminate" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
comment_the_C_code "trick_master_slave" "$WORKSPACE/exe/PIL/master/RUN_golden/golden.cpp"
comment_the_C_code '("initialization") transceiver.initialize_connection' "$WORKSPACE/exe/PIL/master/S_define"
sed_ipaddr_subst $ESPS_IP "$WORKSPACE/models/icf/src/icf_trx_ctrl.c"

##### Generate the image#####
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp
python ../../../tools/generate_error.py ../../../public/golden.csv RUN_golden/log_rocket_csv.csv -l
python ../../../tools/ci_test.py result.csv 1e-5 | tee test_result

# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0