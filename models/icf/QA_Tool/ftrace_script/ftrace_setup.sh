#!/bin/sh

# Init the tracing
mount -t debugfs nodev /sys/kernel/debug
sudo echo 0 > /sys/kernel/debug/tracing/tracing_on
echo 0 > /sys/kernel/debug/tracing/trace
echo 65535 > /sys/kernel/debug/tracing/buffer_size_kb
echo "mono" > /sys/kernel/debug/tracing/trace_clock


##### Function Tracer #####
echo function > /sys/kernel/debug/tracing/current_tracer
#echo :mod:adv17v35x > /sys/kernel/debug/tracing/set_ftrace_filter
#echo :mod:bio1737 > /sys/kernel/debug/tracing/set_ftrace_filter
#echo :mod:pcan > /sys/kernel/debug/tracing/set_ftrace_filter
#echo serialadv_start_tx > /sys/kernel/debug/tracing/set_ftrace_filter
#echo pcan_pci_irqhandler >> /sys/kernel/debug/tracing/set_ftrace_filter
##### Empty Function #####
echo bnep_net_setup > /sys/kernel/debug/tracing/set_ftrace_filter
cat /sys/kernel/debug/tracing/set_ftrace_filter

##### Event tracer #####
#echo nop > /sys/kernel/debug/tracing/current_tracer
#echo "irq==16" > /sys/kernel/debug/tracing/events/irq/irq_handler_entry/filter
#echo irq_handler_entry > set_event

##### System call #####
echo "id>=500" > /sys/kernel/debug/tracing/events/raw_syscalls/sys_enter/filter
echo sys_enter > set_event

##### UDP tracer #####
# echo 5566 > set_ftrace_pid
# echo 5566 > set_event_pid
# echo "id>=500" > /sys/kernel/debug/tracing/events/raw_syscalls/sys_enter/filter
# echo sys_enter > /sys/kernel/debug/tracing/set_event
# echo net_dev_xmit >> /sys/kernel/debug/tracing/set_event
# echo udp_sendmsg > /sys/kernel/debug/tracing/set_ftrace_filter
# echo function > /sys/kernel/debug/tracing/current_tracer

##### Net event #####
# echo "net:*" > /sys/kernel/debug/tracing/set_event
# echo function > /sys/kernel/debug/tracing/current_tracer

##### Use trace-cmd #####
# sudo trace-cmd record -p nop -e irq_handler_entry -f "irq==16" -e sys_enter -f "id>=500"
# trace-cmd report

# sh -c 'sudo echo $$ > /sys/kernel/debug/tracing/set_ftrace_pid;sudo echo 1 > /sys/kernel/debug/tracing/tracing_on; sudo ./readCan'
