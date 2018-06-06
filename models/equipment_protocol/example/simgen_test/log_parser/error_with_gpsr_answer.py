#!/usr/bin/env python
import csv
import itertools
import sys
import re
import math
from parser_utility import *
def main():
    csv_answer_fileobj = open('answer_combine_result_dm_and_gpsr_log.csv', 'rbU')
    csv_1khz_egse_fileobj = open('egse_1khz_combine_result_dm_and_gpsr_log.csv', 'rbU')
    csv_target_fileobj = open('combine_result_dm_and_gpsr_log.csv', 'rbU')

    target_arr = csv.reader(csv_target_fileobj)
    answer_arr = csv.reader(csv_answer_fileobj)
    egse_1khz_arr = csv.reader(csv_1khz_egse_fileobj)

    csv_golden_header = ['sim_time (sec)', 'DM_gps_sow (sec)',
                         'DM_LENGTH ', 'DM_SPEED ',
                         '1khz_length_error', '1khz_speed_error (cm/s)']
    csv_target_header = ['sim_time (sec)', 'DM_gps_sow (sec)',
                         'DM_LENGTH ', 'DM_SPEED ',
                         'GPSR_target_answer_LEN_ERR ', 'GPSR_target_answer_SPEED_ERR (cm/s) ']

    lift_off_time = 120.0
    mean_speed_square = 0.0
    mean_length_square = 0.0
    original_mean_length_sq = 0.0
    original_mean_speed_sq = 0.0
    if len(sys.argv) > 1:
        lift_off_time = float(sys.argv[1])

    ## Golden File
    error_golden_file = open('error_with_gpsr_snav_golden.csv', 'w')
    num, original_mean_length_sq, original_mean_speed_sq = gpsr_tlm_compare(egse_1khz_arr, answer_arr, lift_off_time,
                                                                   error_golden_file, csv_golden_header)
    ori_rms_length = get_root_mean_square(original_mean_length_sq, num)
    ori_rms_speed = get_root_mean_square(original_mean_speed_sq, num) * 100
    error_golden_file.close()
    print ( "EGSE_1khz error rms: %f meters" % (ori_rms_length))
    print ( "EGSE_1khz error rms: %f cm/s" % (ori_rms_speed))

    ## Target File
    csv_answer_fileobj.seek(0)
    error_target_file = open('error_with_gpsr_snav_target.csv', 'w')
    num, mean_length_square, mean_speed_square = gpsr_tlm_compare(target_arr, answer_arr, lift_off_time,
                                                         error_target_file, csv_target_header)
    rms_length = get_root_mean_square(mean_length_square, num)
    rms_speed = get_root_mean_square(mean_speed_square, num) * 100
    error_target_file.close()
    print ( "The position of root mean square is %f meters." % (rms_length))
    print ( "The speed of root mean square is %f cm/s." % (rms_speed))
    csv_answer_fileobj.close()
    csv_target_fileobj.close()
    csv_1khz_egse_fileobj.close()
if __name__ == "__main__":
    main()