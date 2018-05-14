#!/usr/bin/env python
import csv
import itertools

csv_dm_file = open('refine_nspo_log.csv', 'rU')
csv_gpsr_file = open('gps_recevier.csv', 'rU')
error_file = open('error_gpsr.csv', 'w')
csvCursor = csv.writer(error_file)

dm_arr = csv.reader(csv_dm_file)
gpsr_arr = csv.reader(csv_gpsr_file)
csvHeader = ['sim_time (sec)', 'gps_sow_time (dec)', 'SBEE_X error', 'SBEE_Y error', 'SBEE_Z error', 'VBEE_X error', 'VBEE_Y error', 'VBEE_Z error']
csvCursor.writerow(csvHeader)

sim_data_list = []
iter_idx = 0
for dm_row, gpsr_row in itertools.izip(dm_arr, gpsr_arr):
    iter_idx += 1
    sim_data_list = []
    if iter_idx <= 1800:
        continue
    sim_data_list.append(float(dm_row[0]))
    sim_data_list.append(float(dm_row[1]))
    sim_data_list.append(float(dm_row[2]) - float(gpsr_row[1]))
    sim_data_list.append(float(dm_row[3]) - float(gpsr_row[2]))
    sim_data_list.append(float(dm_row[4]) - float(gpsr_row[3]))
    sim_data_list.append(float(dm_row[5]) - float(gpsr_row[4]))
    sim_data_list.append(float(dm_row[6]) - float(gpsr_row[5]))
    sim_data_list.append(float(dm_row[7]) - float(gpsr_row[6]))
    csvCursor.writerow(sim_data_list)

csv_gpsr_file.close()
csv_dm_file.close()
error_file.close()