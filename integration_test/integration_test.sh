#!/bin/bash

declare -a arr=("cad" "math" "gnc")
TEST_DIR="$( cd "$( dirname "$0" )" && pwd )"
FSW_DIR="$TEST_DIR/RocketCFS"
echo "Fligh Software DIR: $FSW_DIR"

# Remove previous FSW to prevent the version conflict
if [ -d "$FSW_DIR" ]; then
    echo "Removing the previous FSW directory..."
    rm -rf $FSW_DIR
fi

# Clone FSW
echo "Clone Flight Software..."
git clone git@tainan.tispace.com:FSW/RocketCFS.git $FSW_DIR
cd "$FSW_DIR" && (git submodule update --init --recursive) && (git submodule foreach --recursive git checkout master)
ln -s $FSW_DIR/configs/Hapith-I_defs $FSW_DIR
cd "$FSW_DIR/apps/ecm" && git checkout redundant

# Remove FSW models and link to sirus' models
echo "Replace FSW Models' Directory..."
for model in "${arr[@]}"
do
    rm -rf $FSW_DIR/apps/gnc_lib/fsw/src/$model
    ln -s $FSW_DIR/../../models/$model $FSW_DIR/apps/gnc_lib/fsw/src/$model
done

# Make a symbolink of CMakeLists.txt for include the models
if [ ! -f "$TEST_DIR/CMakeLists.txt" ]; then
    echo "Clone INS CMakeLists.txt..."
    cp $FSW_DIR/apps/ins/CMakeLists.txt $TEST_DIR
    sed -i 's/^#//' $TEST_DIR/CMakeLists.txt
fi
rm $FSW_DIR/apps/ins/CMakeLists.txt
ln -s $TEST_DIR/CMakeLists.txt $FSW_DIR/apps/ins/CMakeLists.txt

# Make a source file to test the availablity of every class
if [ ! -f "$TEST_DIR/ins_class.cpp" ]; then
    echo "Clone INS ins_class.cpp..."
    cp $FSW_DIR/apps/ins/fsw/src/ins_class.cpp $TEST_DIR
fi
rm $FSW_DIR/apps/ins/fsw/src/ins_class.cpp
ln -s $TEST_DIR/ins_class.cpp $FSW_DIR/apps/ins/fsw/src/ins_class.cpp

cd $FSW_DIR && make all

