#!/usr/bin/env python
import csv
import itertools
import sys
import re
import math
from parser_utility import *
def main():
    target_file_name = ""
    if len(sys.argv) == 3:
        target_file_name = sys.argv[2] + "_"
        lift_off_time = float(sys.argv[1])
    if len(sys.argv) == 2:
        lift_off_time = float(sys.argv[1])
    csv_answer_fileobj = open('answer_combine_result_dm_and_gpsr_log.csv', 'rbU')
    csv_target_fileobj = open(target_file_name + 'combine_result_dm_and_gpsr_log.csv', 'rbU')

    target_arr = csv.reader(csv_target_fileobj)
    answer_arr = csv.reader(csv_answer_fileobj)

    csv_target_header = ['sim_time (sec)', 'DM_gps_sow (sec)',
                         'DM_LENGTH ', 'DM_SPEED ',
                         'DM_TLM-targetTLM LEN_ERR ', 'DM_TLM-targetTLM SPEED_ERR ',
                         'Answer DM-TLM ERR_Length ', 'Answer DM-TLM ERR_Speed ',
                         'Target DM-TLM ERR_Length ', 'Target DM-TLM ERR_Speed ']

    lift_off_time = 120.0
    mean_speed_square = 0.0
    mean_length_square = 0.0
    original_mean_length_sq = 0.0
    original_mean_speed_sq = 0.0
    if len(sys.argv) > 1:
        lift_off_time = float(sys.argv[1])

    ## Target File
    csv_answer_fileobj.seek(0)
    error_target_file = open('error_with_gpsr_snav_target.csv', 'w')
    num, mean_length_square, mean_speed_square = gpsr_tlm_compare(target_arr, answer_arr, lift_off_time,
                                                         error_target_file, csv_target_header)
    rms_length = get_root_mean_square(mean_length_square, num)
    rms_speed = get_root_mean_square(mean_speed_square, num) * 100
    error_target_file.close()
    print ( "[DM_TLM-targetTLM]The position of root mean square is %f meters." % (rms_length))
    print ( "[DM_TLM-targetTLM]The speed of root mean square is %f cm/s." % (rms_speed))
    csv_answer_fileobj.close()
    csv_target_fileobj.close()
if __name__ == "__main__":
    main()