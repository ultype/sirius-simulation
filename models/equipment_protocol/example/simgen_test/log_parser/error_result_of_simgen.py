#!/usr/bin/env python
import csv
import itertools
import sys
import re

csv_gpsr_fileobj = open('gpsr_s_nav_tlm.csv', 'rbU')
csv_dm_fileobj = open('refine_log_nspo.csv', 'rbU')


error_file = open('error_of_dm_and_gpsr.csv', 'w')
csvCursor = csv.writer(error_file)

dm_arr = csv.reader(csv_dm_fileobj)
gpsr_arr = csv.reader(csv_gpsr_fileobj)
csvHeader = ['sim_time (sec)', 
             'DM_gps_sow (sec)', 'DM_SBEE_X ', 'DM_SBEE_Y ', 'DM_SBEE_Z ', 'DM_VBEE_X ', 'DM_VBEE_Y ', 'DM_VBEE_Z ',
             'GPSR_gps_sow (sec)', 'GPSR_SBEE_X ', 'GPSR_SBEE_Y ', 'GPSR_SBEE_Z ', 'GPSR_VBEE_X ', 'GPSR_VBEE_Y ', 'GPSR_VBEE_Z ',
             'ERROR_POS_X', 'ERROR_POS_Y', 'ERROR_POS_Z', 'ERROR_VEL_X', 'ERROR_VEL_Y', 'ERROR_VEL_Z']
csvCursor.writerow(csvHeader)

sim_data_list = []
iter_idx = 0
for dm_row, gpsr_row in itertools.izip(dm_arr, gpsr_arr):
    iter_idx += 1
    sim_data_list = []
    if iter_idx <= 1:
        continue
    print dm_row[0]

    for x in xrange(0,8):
        sim_data_list.append(dm_row[x])
    for x in xrange(0,7):
        sim_data_list.append(gpsr_row[x])
    for x in xrange(0,6):    
        sim_data_list.append(float(dm_row[x+2]) - float(gpsr_row[x+1]))
    csvCursor.writerow(sim_data_list)
csv_dm_fileobj.close()
csv_gpsr_fileobj.close()
error_file.close()


print "error_of_dm_and_gpsr.csv"
