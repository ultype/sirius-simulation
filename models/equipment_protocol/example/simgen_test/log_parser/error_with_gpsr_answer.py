#!/usr/bin/env python
import csv
import itertools
import sys
import re
import math
# The answer_xxxx.csv is the result of gpsr_s_nav with simgen_log
csv_answer_fileobj = open('answer_combine_result_dm_and_gpsr_log.csv', 'rbU')
csv_target_fileobj = open('combine_result_dm_and_gpsr_log.csv', 'rbU')


error_file = open('error_with_gpsr_snav_answer.csv', 'w')
csvCursor = csv.writer(error_file)

target_arr = csv.reader(csv_target_fileobj)
answer_arr = csv.reader(csv_answer_fileobj)
csvHeader = ['sim_time (sec)', 'DM_gps_sow (sec)',
             'DM_SBEE_X ', 'DM_SBEE_Y ', 'DM_SBEE_Z ',
             'DM_VBEE_X ', 'DM_VBEE_Y ', 'DM_VBEE_Z ',
             'DM_LENGTH ', 'DM_SPEED ',
             'GPSR_ERROR_POS_X', 'GPSR_ERROR_POS_Y', 'GPSR_ERROR_POS_Z',
             'GPSR_ERROR_VEL_X', 'GPSR_ERROR_VEL_Y', 'GPSR_ERROR_VEL_Z',
             'GPSR_LENGTH_ERROR ', 'GPSR_SPEED_ERROR (cm/s) ']
csvCursor.writerow(csvHeader)

sim_data_list = []
iter_idx = 0
for answer_row, target_row in itertools.izip(answer_arr, target_arr):
    iter_idx += 1
    sim_data_list = []
    if iter_idx <= 1:
        continue

    for x in xrange(0,8):
        sim_data_list.append(target_row[x])
# DM Length
    sim_data_list.append(math.sqrt(float(answer_row[2])**2 + float(answer_row[3])**2  + float(answer_row[4])**2))
# DM SPEED
    sim_data_list.append(math.sqrt(float(answer_row[5])**2 + float(answer_row[6])**2  + float(answer_row[7])**2))
# GPSR ERROR for each axis: answer_gpsr_s_nav - target_gpsr_s_nav
    for x in xrange(0,6):    
        sim_data_list.append(float(answer_row[x+2]) - float(target_row[x+9]))
# GPSR Length ERROR: answer_gpsr_s_nav - target_gpsr_s_nav
    sim_data_list.append((float(answer_row[15]) - float(target_row[15])) * 100)
# GPSR SPEED ERROR: answer_gpsr_s_nav - target_gpsr_s_nav
    sim_data_list.append((float(answer_row[16]) - float(target_row[16])) * 100)

    csvCursor.writerow(sim_data_list)
csv_answer_fileobj.close()
csv_target_fileobj.close()
error_file.close()


print "error_with_gpsr_snav_answer.csv"