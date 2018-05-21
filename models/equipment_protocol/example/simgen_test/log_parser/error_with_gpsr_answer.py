#!/usr/bin/env python
import csv
import itertools
import sys
import re
import math

def get_root_mean_square( mean_square, number):
    return math.sqrt(mean_square / number)
# The answer_xxxx.csv is the result of gpsr_s_nav with simgen_log
csv_answer_fileobj = open('answer_combine_result_dm_and_gpsr_log.csv', 'rbU')
csv_1khz_egse_fileobj = open('egse_1khz_combine_result_dm_and_gpsr_log.csv', 'rbU')
csv_target_fileobj = open('combine_result_dm_and_gpsr_log.csv', 'rbU')

error_file = open('error_with_gpsr_snav_answer.csv', 'w')
csvCursor = csv.writer(error_file)

target_arr = csv.reader(csv_target_fileobj)
answer_arr = csv.reader(csv_answer_fileobj)
egse_1khz_arr = csv.reader(csv_1khz_egse_fileobj)
csvHeader = ['sim_time (sec)', 'DM_gps_sow (sec)',
             'DM_SBEE_X ', 'DM_SBEE_Y ', 'DM_SBEE_Z ',
             'DM_VBEE_X ', 'DM_VBEE_Y ', 'DM_VBEE_Z ',
             'DM_LENGTH ', 'DM_SPEED ',
             'GPSR_ERROR_POS_X', 'GPSR_ERROR_POS_Y', 'GPSR_ERROR_POS_Z',
             'GPSR_ERROR_VEL_X', 'GPSR_ERROR_VEL_Y', 'GPSR_ERROR_VEL_Z',
             'GPSR_LENGTH_ERROR ', 'GPSR_SPEED_ERROR (cm/s) ',
             'egse_1khz ', '1khz_length_error', '1khz_speed_error (cm/s)']
csvCursor.writerow(csvHeader)

sim_data_list = []
iter_idx = 0
start_flight_idx = 0
mean_speed_square = 0.0
mean_length_square = 0.0
original_mean_length_sq = 0.0
original_mean_speed_sq = 0.0
for answer_row, target_row,  egse_1khz_row in itertools.izip(answer_arr, target_arr, egse_1khz_arr):
    iter_idx += 1
    sim_data_list = []
    if iter_idx <= 1:
        continue
    if float(target_row[0]) == 300.0:
        start_flight_idx = iter_idx
    for x in xrange(0,8):
        sim_data_list.append(target_row[x])
# DM Length
    dm_length = math.sqrt(float(answer_row[2])**2 + float(answer_row[3])**2  + float(answer_row[4])**2)
    sim_data_list.append(dm_length)
# DM SPEED
    dm_speed = math.sqrt(float(answer_row[5])**2 + float(answer_row[6])**2  + float(answer_row[7])**2)
    sim_data_list.append(dm_speed)
# GPSR ERROR for each axis: answer_gpsr_s_nav - target_gpsr_s_nav
    for x in xrange(0,6):    
        sim_data_list.append(float(answer_row[x+2]) - float(target_row[x+9]))
# GPSR Length ERROR: answer_gpsr_s_nav - target_gpsr_s_nav
    gpsr_length_err = float(answer_row[15]) - float(target_row[15])
    sim_data_list.append(gpsr_length_err)
# GPSR SPEED ERROR cm/s: answer_gpsr_s_nav - target_gpsr_s_nav
    gpsr_speed_err = float(answer_row[16]) - float(target_row[16])
    sim_data_list.append(gpsr_speed_err * 100)
# EGSE_1KHZ_benchmark
    sim_data_list.append(" ")
    # egse_1khz Answer Error: 1khz_answer_gpsr_s_nav - egse_1khz_gpsr
    egse_1khz_length = math.sqrt(float(egse_1khz_row[2])**2 + float(egse_1khz_row[3])**2  + float(egse_1khz_row[4])**2)
    egse_1khz_length_err = float(answer_row[15]) - egse_1khz_length
    sim_data_list.append(egse_1khz_length_err)
    egse_1khz_speed = math.sqrt(float(egse_1khz_row[5])**2 + float(egse_1khz_row[6])**2  + float(egse_1khz_row[7])**2)
    egse_1khz_speed_err = float(answer_row[16]) - egse_1khz_speed
    sim_data_list.append(egse_1khz_speed_err * 100)
# Write into the CSV
    csvCursor.writerow(sim_data_list)
# Root Mean square
    if iter_idx >= start_flight_idx:
        mean_length_square = mean_length_square + (gpsr_length_err)**2
        mean_speed_square = mean_speed_square + (gpsr_speed_err)**2
        original_mean_length_sq = original_mean_length_sq + egse_1khz_length_err**2
        original_mean_speed_sq = original_mean_speed_sq + egse_1khz_speed_err**2

ori_rms_length = get_root_mean_square(original_mean_length_sq, iter_idx - start_flight_idx)
ori_rms_speed = get_root_mean_square(original_mean_speed_sq, iter_idx - start_flight_idx) * 100

rms_length = get_root_mean_square(mean_length_square, iter_idx - start_flight_idx)
rms_speed = get_root_mean_square(mean_speed_square, iter_idx - start_flight_idx) * 100

csv_answer_fileobj.close()
csv_target_fileobj.close()
error_file.close()

print ("error_with_gpsr_snav_answer.csv")
print ("Start flight idx: %d" % (start_flight_idx))
print ( "The position of root mean square is %f meters. (EGSE_1khz error: %f meters)" % (rms_length, ori_rms_length))
print ( "The speed of root mean square is %f cm/s. (EGSE_1khz error: %f cm/s)" % (rms_speed, ori_rms_speed))