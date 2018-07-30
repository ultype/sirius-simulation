#!/usr/bin/env python
import re
import numpy
import sys
def counting_actual_size(file, output):
    body_size = 0
    not_counting_size = 0
    # Read the first line
    line = file.readline()
    while line:
        if line[:1] == "#" or line[:] == "\0" or line[:] == "\n":
            not_counting_size +=1
        else:
            output.write(line)
        line = file.readline()
        body_size += 1
    body_size -= not_counting_size
    print("%s lines = %d" % (file.name, body_size))
    return body_size

### CONSTANT ###
SAMPLE_HZ = "200 "
GRAPH_TITLE = "No title"
file_input = open("logg.txt","r")
file_out = open("output.txt","w+")

#plot_file = open("plot_data_input.txt", "w+")
logger_size = counting_actual_size(file_input, file_out)
file_input.close()
file_out.close()


with open("output.txt", "r") as file_out:
    data = file_out.readlines()

if len(sys.argv) < 0:
    print "Please imput the < 1 | 2 | 3 > <sample HZ> <Graph title>"
    sys.exit(0)
IDX = int(sys.argv[1])
SAMPLE_HZ = int(sys.argv[2])
GRAPH_TITLE = str(sys.argv[3])
#### GNU PLOT Setting: User defined ####
PLOT_CONFIG_INFO = [
    ["Search Keywords", "Search secondary Keywords","Title ", "x label, y label", "parsing_mode"],
    ["sys_enter: NR 510", " ",                  "[HIL] " + GRAPH_TITLE, "Time frame (sec)", "Schedule Period (ms)", "SCHEDULE"], # 1
    ["daq_irq_handler"  , " ",                  "[HIL] " + GRAPH_TITLE, "Timeframe 1 Hz",   "Jitter(ms)",           "JITTER"], # 2
    ["sys_enter: NR 510", "sys_enter: NR 511",  "[HIL] " + GRAPH_TITLE, "Time frame (sec)", "Period (ms)",          "PERIOD"], # 3
    ["udp_sendmsg",       "net_dev_xmit",       "[HIL] " + GRAPH_TITLE, "Time frame (sec)", "Period (ms)",          "TRACE-CMD"], # 4
]


search_string_1 = PLOT_CONFIG_INFO[IDX][0]
search_string_2 = PLOT_CONFIG_INFO[IDX][1]
gtitle= PLOT_CONFIG_INFO[IDX][2]
xlabel = PLOT_CONFIG_INFO[IDX][3]
ylabel = PLOT_CONFIG_INFO[IDX][4]
photo_name = gtitle +".png"
parsing_mode = PLOT_CONFIG_INFO[IDX][5]
################################
trip_time_ms_list = []
abnormal_cnt = 0
irq_count = 0
start_time = 0
end_time = 0

if parsing_mode == "SCHEDULE":
    for idx, element in enumerate(data):
        if element.find(search_string_1) > 0:
            temp_list = re.findall("\d+\.\d+", data[idx])
            end_time = float(temp_list[0])
            if start_time != 0:
                duration = (end_time - start_time) * 1000
                trip_time_ms_list.append(duration)
            start_time = end_time

if parsing_mode == "JITTER":
    PASER_START_IDX = 0
    for idx, element in enumerate(data):
      if idx < PASER_START_IDX:
          continue
      if element.find(search_string_1) > 0:
          temp_list = re.findall("\d+\.\d+", data[idx - PASER_START_IDX])
          start_time = float(temp_list[0])
          temp_list = re.findall("\d+\.\d+", data[idx+1])
          end_time = float(temp_list[0])
          duration = (end_time - start_time) * 1000
          if duration > 3:
              abnormal_cnt += 1;
              print "[%d]output.txt Line: %d, time duration:%f" % (abnormal_cnt, idx+1, duration)
          trip_time_ms_list.append(duration)
if parsing_mode == "PERIOD":
    temp_list_start = []
    temp_list_end = []
    for idx, element in enumerate(data):
        if element.find(search_string_1) > 0:
            temp_list_start = re.findall("\d+\.\d+", data[idx])
            start_time = float(temp_list_start[0])
            continue
        if element.find(search_string_2) > 0:
            temp_list_end = re.findall("\d+\.\d+", data[idx])
            end_time = float(temp_list_end[0])
            duration = (end_time - start_time) * 1000
            trip_time_ms_list.append(duration)
if parsing_mode == "TRACE-CMD":
    temp_list_start = []
    temp_list_end = []
    flag = 0
    for idx, element in enumerate(data):
        if element.find(search_string_1) > 0 and flag == 0:
            flag = 1
            result = element.split(":")
            result = result[0].split(" ")
            temp_list_start = result[3]
            start_time = float(temp_list_start)
            continue
        if element.find(search_string_2) > 0 and flag == 1:
            flag = 0
            result = element.split(":")
            result = result[0].split(" ")
            temp_list_end = result[3]
            end_time = float(temp_list_end)
            duration = (end_time - start_time) / 1000000.0
            trip_time_ms_list.append(duration)

print("|Round trip count | %d |" % (len(trip_time_ms_list)))
print("|Avg| %f ms |" % (sum(trip_time_ms_list)/len(trip_time_ms_list)))
print("|Max| %f ms |" % (max(trip_time_ms_list)))
print("|Min| %f ms |" % (min(trip_time_ms_list)))
print("|sigma| %f  |" % (numpy.std(trip_time_ms_list)))
time_stamp = 0
with open("plot_data_input.txt", "w") as file_out:
    for idx in range(len(trip_time_ms_list)):
        time_stamp += (1 / float(SAMPLE_HZ))
        file_out.write(str(time_stamp) + " " + str(trip_time_ms_list[idx]) +"\n")

data_string = " count=" + "\\n" + str(len(trip_time_ms_list)) + "\\n"
data_string += " Max=" + "\\n" + str(max(trip_time_ms_list)) + "\\n"
data_string += " Min=" + "\\n" + str(min(trip_time_ms_list)) + "\\n"
data_string += " Avg=" + "\\n" + str(sum(trip_time_ms_list)/len(trip_time_ms_list)) + "\\n"
data_string += " sigma=" + "\\n" + str(numpy.std(trip_time_ms_list)) + "\\n"

print "Please use the following command to plot:"
print "gnuplot -e 'in=\"plot_data_input.txt\";out=\"%s\";gtitle=\"%s\"; \
data_string=\"%s\";xlabel=\"%s\" ;ylabel=\"%s\"' plot.gp" \
% (photo_name,gtitle, data_string, xlabel, ylabel)