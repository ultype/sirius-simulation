#!/bin/bash
set -x
TARGET_DIR=$1
find $TARGET_DIR -type d -name "jitlib" -exec rm -rf {} \;
find $TARGET_DIR -type d -name "build" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "S_job_execution" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "S_run_summary" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_gps_slave.csv" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_gps_slave.header" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "send_hs" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "varserver_log" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "Session.dtd" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_gps.csv" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_gps.header" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_nspo.csv" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_nspo.header" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_rocket_csv.csv" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "log_rocket_csv.header" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "result.csv" -exec rm -rf {} \;
find $TARGET_DIR -type f -name "test_result" -exec rm -rf {} \;
set +x