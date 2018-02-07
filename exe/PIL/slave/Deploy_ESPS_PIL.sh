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
    EGSE_IP="192.168.0.9"
else
    EGSE_IP=$1
fi
echo "EGSE_IP: "$EGSE_IP

##### FUNCTION #####
comment_the_Python_code() {
    pattern=$1
    file_path=$2
    if grep -q "#$pattern" $file_path; then
        return
    else
        sed -i -e "s/$pattern/\#$pattern/g" "$file_path"
    fi
}

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
comment_the_Python_code "new_connection = trick.MSSocket" "$WORKSPACE/exe/PIL/slave/RUN_golden/golden.py"
comment_the_Python_code "trick_master_slave" "$WORKSPACE/exe/PIL/slave/RUN_golden/golden.py"
comment_the_C_code '("initialization") transceiver.initialize_connection' "$WORKSPACE/exe/PIL/slave/S_define"
sed_ipaddr_subst $EGSE_IP "$WORKSPACE/models/icf/src/icf_trx_ctrl.c"
##### Generate the image#####
trick-CP
./S_main_Linux_5.4_x86_64.exe RUN_golden/golden.py