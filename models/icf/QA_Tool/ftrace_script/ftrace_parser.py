#!/usr/bin/env python
import re
import numpy
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
IRQ_NUM_PER_RX_CAN_PACKET = 7
SYSCALL_NUM_PER_ROUND_TRIP = 7
SERIAL_PORT_START_TX_CNT = 172
file_input = open("logg.txt","r")
file_out = open("output.txt","w+")
#plot_file = open("plot_data_input.txt", "w+")
logger_size = counting_actual_size(file_input, file_out)
file_input.close()
file_out.close()


with open("output.txt", "r") as file_out:
    data = file_out.readlines()

#### GNU PLOT Setting: User defined #### 
search_string = "sys_enter: NR 502"
gtitle= "[HIL] EGSE Rateble_X (Simulation Time: 200 secs)"
xlabel = "Timeframe (1000 HZ)"
ylabel = "Schedule Period (ms)"
photo_name = gtitle +".png"
################################
trip_time_ms_list = []
abnormal_cnt = 0
irq_count = 0
start_time = 0
end_time = 0

for idx, element in enumerate(data):
    if element.find(search_string) > 0:
        temp_list = re.findall("\d+\.\d+", data[idx])
        end_time = float(temp_list[0])
        if start_time != 0:
            duration = (end_time - start_time) * 1000
            trip_time_ms_list.append(duration)
        start_time = end_time

# for idx, element in enumerate(data):
#   if element.find("irq_handler_entry: irq=16 name=pcan") > 0:
#       irq_count += 1
#       if irq_count == 1:
#           temp_list = re.findall("\d+\.\d+", data[idx])
#           start_time = float(temp_list[0])
#       if irq_count == 7:
#           temp_list = re.findall("\d+\.\d+", data[idx + 172])
#           end_time = float(temp_list[0])
#           duration = (end_time - start_time) * 1000
#           if duration > 3:
#               abnormal_cnt += 1;
#               print "[%d]output.txt Line: %d, time duration:%f" % (abnormal_cnt, idx+1, duration)
#           trip_time_ms_list.append(duration)
#           irq_count = 0

# PASER_START_IDX = 2
# for idx, element in enumerate(data):
#   if idx < PASER_START_IDX:
#       continue
#   if element.find("sys_enter: NR 511") > 0:
#       temp_list = re.findall("\d+\.\d+", data[idx - PASER_START_IDX])
#       start_time = float(temp_list[0])
#       temp_list = re.findall("\d+\.\d+", data[idx])
#       end_time = float(temp_list[0])
#       duration = (end_time - start_time) * 1000
#       if duration > 3:
#           abnormal_cnt += 1;
#           print "[%d]output.txt Line: %d, time duration:%f" % (abnormal_cnt, idx+1, duration)
#       trip_time_ms_list.append(duration)

print("|Round trip count | %d |" % (len(trip_time_ms_list)))
print("|Avg| %f ms |" % (sum(trip_time_ms_list)/len(trip_time_ms_list)))
print("|Max| %f ms |" % (max(trip_time_ms_list)))
print("|Min| %f ms |" % (min(trip_time_ms_list)))
print("|sigma| %f  |" % (numpy.std(trip_time_ms_list)))

with open("plot_data_input.txt", "w") as file_out:
    for idx in range(len(trip_time_ms_list)):
        file_out.write(str(idx+1) + " " + str(trip_time_ms_list[idx]) +"\n")

data_string = " count=" + "\\n" + str(len(trip_time_ms_list)) + "\\n"
data_string += " Max=" + "\\n" + str(max(trip_time_ms_list)) + "\\n"
data_string += " Min=" + "\\n" + str(min(trip_time_ms_list)) + "\\n"
data_string += " Avg=" + "\\n" + str(sum(trip_time_ms_list)/len(trip_time_ms_list)) + "\\n"
data_string += " sigma=" + "\\n" + str(numpy.std(trip_time_ms_list)) + "\\n"

print "Please use the following command to plot:"
print "gnuplot -e 'in=\"plot_data_input.txt\";out=\"%s\";gtitle=\"%s\"; \
data_string=\"%s\";xlabel=\"%s\" ;ylabel=\"%s\"' plot.gp" \
% (photo_name,gtitle, data_string, xlabel, ylabel)