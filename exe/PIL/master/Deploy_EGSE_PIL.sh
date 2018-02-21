#!/bin/bash
set -e

##### Variable #####
if env | grep -q ^WORKSPACE=
then
    echo finding slave from $WORKSPACE
else
    export WORKSPACE=`pwd`/../../../
fi
SIRIUS_HOME_PATH=$(pwd | sed 's/\/exe\/PIL\/master//g')
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

##### Generate the image#####
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.cpp
python $SIRIUS_HOME_PATH/tools/generate_error.py $SIRIUS_HOME_PATH/public/golden.csv RUN_golden/log_rocket_csv.csv -l
python $SIRIUS_HOME_PATH/tools/ci_test.py result.csv 1e-5 | tee test_result

# Test the exit status of the command before pipe
test ${PIPESTATUS[0]} -eq 0