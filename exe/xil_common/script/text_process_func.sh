#!/bin/bash
uncomment_the_C_code() {
    pattern=$1
    file_path=$2
    sed -i -e "s/\/\/$pattern/$pattern/g" "$file_path"
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

uncomment_the_Python_code() {
    pattern=$1
    file_path=$2
    sed -i -e "s/\#$pattern/$pattern/g" "$file_path"
}

comment_the_Python_code() {
    pattern=$1
    file_path=$2
    if grep -q "#$pattern" $file_path; then
        return
    else
        sed -i -e "s/$pattern/\#$pattern/g" "$file_path"
    fi
}
