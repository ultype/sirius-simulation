#!/usr/bin/env python
import csv
import itertools
import sys
import re
import math
def main():
    if len(sys.argv) > 0:
        log_nspo_csv_name = sys.argv[1]

    input_simgen_remote_dump_fileobj = open('UDP_mot_receive_in_simgen.csv', 'rbU')
    input_dm_nspo_fileobj = open(log_nspo_csv_name, 'rbU')
    output_received_check_fileobj = open('check_simgen_remote_received.csv', 'w')
    csvCursor = csv.writer(output_received_check_fileobj)
    csvHeader = ['simgen time (sec)', 
                 'SIMGEN SBEE_X ', 'SIMGEN SBEE_Y ', 'SIMGEN SBEE_Z ', 
                 'SIMGEN VBEE_X ', 'SIMGEN VBEE_Y ', 'SIMGEN VBEE_Z ', 'Latency' ,
                 'DM_SBEE_X', 'DM_SBEE_Y', 'DM_SBEE_Z' ,
                 'DM_VBEE_X', 'DM_VBEE_Y', 'DM_VBEE_Z',
                 'DM_Length', 'DM_Speed',
                 'SIMGEN_Length', 'SIMGEN_Speed',
                 'DM-SIMGEN_Length_ERR', 'DM-SIMGEN_Speed_ERR']
    csvCursor.writerow(csvHeader)

    sim_data_list = []

    simgen_dump_arr = csv.reader(input_simgen_remote_dump_fileobj)
    dm_arr = csv.reader(input_dm_nspo_fileobj)

    cache_idx = 0
    for simgen_elem in enumerate(simgen_dump_arr):
        sim_data_list = []
        if simgen_elem[0] == 0:
            continue;
        for dm_elem in enumerate(dm_arr , start = cache_idx):
            cache_idx = dm_elem[0]
            if dm_elem[0] == 0:
                continue;
            if float(simgen_elem[1][0]) == float(dm_elem[1][0]):
                print("simgen sim_time:" + simgen_elem[1][0] + " dm_sim time:" + dm_elem[1][0])
                sim_data_list.append(simgen_elem[1][0])

                sim_data_list.append(simgen_elem[1][1])
                sim_data_list.append(simgen_elem[1][2])
                sim_data_list.append(simgen_elem[1][3])
                sim_data_list.append(simgen_elem[1][4])
                sim_data_list.append(simgen_elem[1][5])
                sim_data_list.append(simgen_elem[1][6])
                sim_data_list.append(simgen_elem[1][7])
                sim_data_list.append(dm_elem[1][2])
                sim_data_list.append(dm_elem[1][3])
                sim_data_list.append(dm_elem[1][4])
                sim_data_list.append(dm_elem[1][5])
                sim_data_list.append(dm_elem[1][6])
                sim_data_list.append(dm_elem[1][7])
                dm_length = math.sqrt(float(dm_elem[1][2])**2 + float(dm_elem[1][3])**2 + float(dm_elem[1][4])**2)
                simgen_length = math.sqrt(float(simgen_elem[1][1])**2 + float(simgen_elem[1][2])**2 + float(simgen_elem[1][3])**2)
                
                dm_speed = math.sqrt(float(dm_elem[1][5])**2 + float(dm_elem[1][6])**2 + float(dm_elem[1][7])**2)
                simgen_speed = math.sqrt(float(simgen_elem[1][4])**2 + float(simgen_elem[1][5])**2 + float(simgen_elem[1][6])**2)
                sim_data_list.append(dm_length)
                sim_data_list.append(dm_speed)
                sim_data_list.append(simgen_length)
                sim_data_list.append(simgen_speed)
                sim_data_list.append(dm_length - simgen_length)
                sim_data_list.append(dm_speed - simgen_speed)
                csvCursor.writerow(sim_data_list)
                break

    input_dm_nspo_fileobj.close()
    input_simgen_remote_dump_fileobj.close()
    print ("check_simgen_remote_received.csv\n")

if __name__ == "__main__":
    main()