#!/bin/bash

# User Guide:
# $ ./unit.sh

set -e

# clone google test framework for unit-test if it does not exist
test ! -d googletest && git clone https://github.com/google/googletest.git

# compile unit-test
make -C unit_test

# time test
./unit_test/timeTest --gtest_output=xml:timeTest.xml

# math test
./unit_test/mathTest --gtest_output=xml:mathTest.xml
