#!/usr/bin/env python
import csv
import itertools
import sys
import re
import numpy
from numpy import genfromtxt
import Gnuplot
def main():
    s_nav_log_input_file_name = sys.argv[1]
    print (s_nav_log_input_file_name)
    output_s_nav_file = open('gpsr_s_nav_tlm.csv', 'w')
    search_string = "$POS,"

    csvHeader = "GPS_SOW_TIME (sec), SBEE_X, SBEE_Y, SBEE_Z, VBEE_X, VBEE_Y, VBEE_Z, unknow0, unknow1, unknow2"

    with open(s_nav_log_input_file_name, 'rU') as s_nav_file_src:
        s_nav_input_log = s_nav_file_src.readlines()
    sim_data_list = []
    for element in enumerate(s_nav_input_log):
        if (search_string in element[1]) == True:
            print (element[1])
            line = element[1].replace(search_string, "")
            sim_data_list.append(line)

    output_s_nav_file.write("%s\n" % csvHeader)
    for item in sim_data_list:
        output_s_nav_file.write("%s" % item)

    s_nav_file_src.close()
    output_s_nav_file.close()
    print ("gpsr_s_nav_tlm.csv\n")
    print ("Next step ./time_log_alignment.py <DM log_nspo file>")

    g = Gnuplot.Gnuplot(persist = 1)
    g('set grid')
    g('set key left ')
    g('set title noenhanced')
    g('set label 1 at screen 0.92, screen 0.6 left')

    g.title('GPS-S-NAV-TLM')
    g.xlabel('Sequence (50ms)')
    g.ylabel('GPS Time (sec)')

    plotdata = Gnuplot.File('gpsr_s_nav_tlm.csv', using = '0:1', with_ = 'line')
    g.plot(plotdata)
    g.hardcopy (filename = 'gpsr_s_nav_tlm.png', terminal = 'png') # write last plot to another terminal

if __name__ == "__main__":
    main()
