#!/usr/bin/env python
import csv
import itertools
import sys
import re
import math

def get_root_mean_square( mean_square, number):
    return math.sqrt(mean_square / number)

def compare(target_arr, answer_arr, lift_off_time, filecusor):
    cache_idx = 0
    sim_data_list = []
    start_flight_idx = 0
    iter_idx = 0
    mean_len_sq = 0.0
    mean_speed_sq = 0.0
    for target_elem in enumerate(target_arr):
        sim_data_list = []
        iter_idx += 1
        if target_elem[0] == 0:
            continue;
        if float(target_elem[1][0]) == lift_off_time:
            start_flight_idx = iter_idx
        for answer_elem in enumerate(answer_arr , start = cache_idx):
            cache_idx = answer_elem[0]
            if answer_elem[0] == 0:
                continue;
            if abs(float(target_elem[1][0]) - float(answer_elem[1][0])) == 0.0:
                sim_data_list.append(target_elem[1][0])
                sim_data_list.append(target_elem[1][1])
    # DM Length
                dm_length = math.sqrt(float(answer_elem[1][2])**2 + float(answer_elem[1][3])**2  + float(answer_elem[1][4])**2)
                sim_data_list.append(dm_length)
    # DM SPEED
                dm_speed = math.sqrt(float(answer_elem[1][5])**2 + float(answer_elem[1][6])**2  + float(answer_elem[1][7])**2)
                sim_data_list.append(dm_speed)
    # benchmark
                target_length_err = float(answer_elem[1][15]) - float(target_elem[1][15])
                target_speed_err = float(answer_elem[1][16]) - float(target_elem[1][16])
                sim_data_list.append(target_length_err)
                sim_data_list.append(target_speed_err)
                filecusor.writerow(sim_data_list)
    # Root Mean square
                if iter_idx >= start_flight_idx:
                    mean_len_sq = mean_len_sq + target_length_err**2
                    mean_speed_sq = mean_speed_sq + target_speed_err**2
                break
    return (iter_idx - start_flight_idx), mean_len_sq, mean_speed_sq
# The answer_xxxx.csv is the result of gpsr_s_nav with simgen_log
csv_answer_fileobj = open('answer_combine_result_dm_and_gpsr_log.csv', 'rbU')
csv_1khz_egse_fileobj = open('egse_1khz_combine_result_dm_and_gpsr_log.csv', 'rbU')
csv_target_fileobj = open('combine_result_dm_and_gpsr_log.csv', 'rbU')

error_target_file = open('error_with_gpsr_snav_target.csv', 'w')
csvTargetCursor = csv.writer(error_target_file)

error_golden_file = open('error_with_gpsr_snav_golden.csv', 'w')
csvGoldenCursor = csv.writer(error_golden_file)

target_arr = csv.reader(csv_target_fileobj)
answer_arr = csv.reader(csv_answer_fileobj)
egse_1khz_arr = csv.reader(csv_1khz_egse_fileobj)


csv_golden_header = ['sim_time (sec)', 'DM_gps_sow (sec)',
                     'DM_LENGTH ', 'DM_SPEED ',
                     '1khz_length_error', '1khz_speed_error (cm/s)']
csv_target_header = ['sim_time (sec)', 'DM_gps_sow (sec)',
                     'DM_LENGTH ', 'DM_SPEED ',
                     'GPSR_target_answer_LEN_ERR ', 'GPSR_target_answer_SPEED_ERR (cm/s) ']
csvGoldenCursor.writerow(csv_golden_header)
csvTargetCursor.writerow(csv_target_header)
lift_off_time = 120.0
mean_speed_square = 0.0
mean_length_square = 0.0
original_mean_length_sq = 0.0
original_mean_speed_sq = 0.0
if len(sys.argv) > 1:
    lift_off_time = float(sys.argv[1])

num, original_mean_length_sq, original_mean_speed_sq = compare(egse_1khz_arr, answer_arr, lift_off_time, csvGoldenCursor)
ori_rms_length = get_root_mean_square(original_mean_length_sq, num)
ori_rms_speed = get_root_mean_square(original_mean_speed_sq, num) * 100

num, mean_length_square, mean_speed_square = compare(target_arr, answer_arr, lift_off_time, csvTargetCursor)
rms_length = get_root_mean_square(mean_length_square, num)
rms_speed = get_root_mean_square(mean_speed_square, num) * 100

csv_answer_fileobj.close()
csv_target_fileobj.close()
csv_1khz_egse_fileobj.close()
error_target_file.close()
error_golden_file.close()

print ("error_with_gpsr_snav_answer.csv")
print ( "The position of root mean square is %f meters. (EGSE_1khz error: %f meters)" % (rms_length, ori_rms_length))
print ( "The speed of root mean square is %f cm/s. (EGSE_1khz error: %f cm/s)" % (rms_speed, ori_rms_speed))