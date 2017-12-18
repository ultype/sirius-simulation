#!/bin/bash
set -x
make clean
find . -type d -name "jitlib" -exec rm -rf {} \;
cd ../slave/
make clean
set +x