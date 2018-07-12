#!/usr/bin/env python
import csv
import itertools
import sys
import re
import math
from parser_utility import *
def main():
    lift_off_time = 120.0
    csv_gpsr_fileobj = open('gpsr_s_nav_tlm.csv', 'rbU')
    csv_dm_fileobj = open('refine_log_nspo.csv', 'rbU')
    out_file_name = ""
    if len(sys.argv) == 3:
        out_file_name = sys.argv[2] + "_"
        lift_off_time = float(sys.argv[1])
    if len(sys.argv) == 2:
        lift_off_time = float(sys.argv[1])

    combine_file = open(out_file_name + 'combine_result_dm_and_gpsr_log.csv', 'w')
    csvCursor = csv.writer(combine_file)

    dm_arr = csv.reader(csv_dm_fileobj)
    gpsr_arr = csv.reader(csv_gpsr_fileobj)
    csvHeader = ['sim_time (sec)', 'DM_gps_sow (sec)',
                 'DM_SBEE_X ', 'DM_SBEE_Y ', 'DM_SBEE_Z ',
                 'DM_VBEE_X ', 'DM_VBEE_Y ', 'DM_VBEE_Z ',
                 'DM_Length' , 'DM_Speed',
                 'GPSR_gps_sow (sec)',
                 'GPSR_SBEE_X ', 'GPSR_SBEE_Y ', 'GPSR_SBEE_Z ',
                 'GPSR_VBEE_X ', 'GPSR_VBEE_Y ', 'GPSR_VBEE_Z ',
                 'GPSR_Length', 'GPSR_Speed ',
                 'DM-GPSR_TLM ERR_Length','DM-GPSR_TLM ERR_Speed']
    csvCursor.writerow(csvHeader)

    sim_data_list = []
    iter_idx = 0
    start_flight_idx = 0
    mean_length_square = 0
    mean_speed_square = 0

    for dm_row, gpsr_row in itertools.izip(dm_arr, gpsr_arr):
        iter_idx += 1
        sim_data_list = []
        if iter_idx <= 1:
            continue
        print (dm_row[0])
        if float(dm_row[0]) == lift_off_time:
            start_flight_idx = iter_idx
    ###### DM Part ######
        for x in xrange(0,8):
            sim_data_list.append(dm_row[x])
    # Calculate DM Length
        DM_Length = math.sqrt(float(dm_row[2])**2 + float(dm_row[3])**2  + float(dm_row[4])**2)
        sim_data_list.append(DM_Length)
    # Calculate DM Speed
        DM_Speed = math.sqrt(float(dm_row[5])**2 + float(dm_row[6])**2  + float(dm_row[7])**2)
        sim_data_list.append(DM_Speed)
    ###### GPSR_TLM Part ######
        for x in xrange(0,7):
            sim_data_list.append(gpsr_row[x])
    # Calculate GPSR_TLM Length
        GPSR_Length = math.sqrt(float(gpsr_row[1])**2 + float(gpsr_row[2])**2  + float(gpsr_row[3])**2)
        sim_data_list.append(GPSR_Length)
    # Calculate GPSR_TLM Speed
        GPSR_Speed = math.sqrt(float(gpsr_row[4])**2 + float(gpsr_row[5])**2  + float(gpsr_row[6])**2)
        sim_data_list.append(GPSR_Speed)
    #Calculate DM - GPSR_TLM Length Error
        sim_data_list.append(DM_Length - GPSR_Length)
    #Calculate DM - GPSR_TLM Speed Error
        sim_data_list.append(DM_Speed - GPSR_Speed)
        csvCursor.writerow(sim_data_list)
    # Root Mean square
        if iter_idx >= start_flight_idx:
            mean_length_square = mean_length_square + (DM_Length - GPSR_Length)**2
            mean_speed_square = mean_speed_square + (DM_Speed - GPSR_Speed)**2


    rms_length = get_root_mean_square(mean_length_square, iter_idx - start_flight_idx)
    rms_speed = get_root_mean_square(mean_speed_square, iter_idx - start_flight_idx) * 100
    csv_dm_fileobj.close()
    csv_gpsr_fileobj.close()
    combine_file.close()

    print ( "The position of root mean square with DM is %f meters." % (rms_length))
    print ( "The speed of root mean square with DM is %f cm/s." % (rms_speed))

    print (out_file_name + "combine_result_dm_and_gpsr_log.csv\n")
    print ("Next step ./error_with_gpsr_tlm_golden.py <lift_off_time> <egse_1khz | rt200hz | ' '>")

if __name__ == "__main__":
    main()