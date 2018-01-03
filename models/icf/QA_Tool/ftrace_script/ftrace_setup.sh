#!/bin/sh

# Init the tracing
mount -t debugfs nodev /sys/kernel/debug
sudo echo 0 > /sys/kernel/debug/tracing/tracing_on
echo 0 > /sys/kernel/debug/tracing/trace
echo 65535 > /sys/kernel/debug/tracing/buffer_size_kb
echo "mono" > /sys/kernel/debug/tracing/trace_clock

# sh -c 'sudo echo $$ > /sys/kernel/debug/tracing/set_ftrace_pid;sudo echo 1 > /sys/kernel/debug/tracing/tracing_on; sudo ./readCan'
#echo "32640" > /sys/kernel/debug/tracing/set_ftrace_pid

##### Function Tracer #####
echo function > /sys/kernel/debug/tracing/current_tracer
#echo :mod:adv17v35x > /sys/kernel/debug/tracing/set_ftrace_filter
#echo :mod:pcan > /sys/kernel/debug/tracing/set_ftrace_filter
#echo serialadv_start_tx > /sys/kernel/debug/tracing/set_ftrace_filter
#echo pcan_pci_irqhandler >> /sys/kernel/debug/tracing/set_ftrace_filter
##### Empty Function #####
echo bnep_net_setup > /sys/kernel/debug/tracing/set_ftrace_filter
cat /sys/kernel/debug/tracing/set_ftrace_filter

##### Event tracer #####
##IRQ PCAN
#echo nop > /sys/kernel/debug/tracing/current_tracer
#echo "irq==16" > /sys/kernel/debug/tracing/events/irq/irq_handler_entry/filter
#echo 1 > /sys/kernel/debug/tracing/events/irq/irq_handler_entry/enable

##### System call #####
echo "id>=500" > /sys/kernel/debug/tracing/events/raw_syscalls/sys_enter/filter
echo 1 > /sys/kernel/debug/tracing/events/raw_syscalls/sys_enter/enable

#Use trace-cmd
#sudo trace-cmd record -p nop -e irq_handler_entry -f "irq==16" -e sys_enter -f "id>=500"
#trace-cmd report

##### Socket CAN Interface Enable #####
# sudo ip link set can0 up type can bitrate 125000
# sudo ip link set can1 up type can bitrate 125000

##### CPU Isolation #####
# sudo vim /etc/default/grub ===> add GRUB_CMDLINE_LINUX_DEFAULT="isolcpus=1"
# sudo update-grub
# sudo reboot

##### Loogin Permission #####
# sudo vim /etc/ssh/sshd_config ===> modify "PermitRootLogin yes"