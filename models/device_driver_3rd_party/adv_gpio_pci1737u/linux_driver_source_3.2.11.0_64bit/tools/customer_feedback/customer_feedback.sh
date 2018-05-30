#/bin/bash

[ "$UID" -ne 0 ] && echo "Run this script with root privileges" && exit 1

CURR=${PWD}
output_file=${CURR}/DAQNavi_customer_feedback.txt

if [ -f $output_file ];then
	rm $output_file
fi

echo Support mail: support@advantech.com.tw >>  $output_file

echo *****************************System infomation*******************************  >> $output_file

arch=`uname -m | sed 's/^[ \t]*//'`
echo "Architecture = ${arch}" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file

model_name=`cat /proc/cpuinfo | grep "model name" | awk -F ":" '{print $2}' | head -n 1 | sed 's/^[ \t]*//'`
echo "Processors = $model_name" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
cpu_mhz=`cat /proc/cpuinfo | grep "cpu MHz" | awk -F ":" '{print $2}' | head -n 1 | sed 's/^[ \t]*//'`
echo "CPU MHz = $cpu_mhz MHz" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
cpu_core_number=`cat /proc/cpuinfo | grep "processor" | awk -F ":" '{print $2}' | wc -l | sed 's/^[ \t]*//'`
echo "CPU Core Number = $cpu_core_number" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
total_memory=`cat /proc/meminfo | grep "MemTotal" | awk -F ":" '{print $2}' | sed 's/^[ \t]*//'`
echo "Total Memory = $total_memory" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
free_memory=`cat /proc/meminfo | grep "MemFree" | awk -F ":" '{print $2}' | sed 's/^[ \t]*//'`
echo "Free Memory = $free_memory" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
df -h | while read disk_info
do
	echo $disk_info >> $output_file
done

echo -----------------------------------------------------------------------------  >>  $output_file
kernel_ver=`uname -r | sed 's/^[ \t]*//'`
echo "Kernel version = ${kernel_ver}" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
gcc_ver=`gcc --version | grep gcc | sed 's/^[ \t]*//'`
echo "Gcc version = $gcc_ver" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
ldd_ver=`ldd --version | grep ldd | sed 's/^[ \t]*//'`
echo "Glibc version = $ldd_ver" >> $output_file

echo -----------------------------------------------------------------------------  >>  $output_file
distribution=`more /etc/issue | head -n 1 | sed 's/^[ \t]*//'`
echo "Distribution = $distribution" >>  $output_file
echo -----------------------------------------------------------------------------  >>  $output_file

comedi_flag=`lsmod  | grep comedi | sed 's/^[ \t]*//'`
if [ "$comedi_flag" = "" ];then
	echo "Comedi driver[Not Exist]"  >> $output_file
else
	echo "Comedi driver[Exist]" >> $output_file
fi
echo "" >>  $output_file
echo *****************************DAQNavi infomation******************************* >> $output_file

if [ -d /sys/class/daq ];then
	echo -e Device number "\t" Description "\t" Hardware version "\t" library version >> $output_file
echo -----------------------------------------------------------------------------  >>  $output_file
	for daq_dev_dir in `ls /sys/class/daq`
	do
		dev_desc=`more /sys/class/daq/$daq_dev_dir/desc`
		dev_num=`echo $daq_dev_dir | cut -b4-6`
		if [ "$dev_num" = "255" ];then
			continue
		fi
		if [ "${arch}" = "x86_64" ];then
			if [ -f ${CURR}/getDeviceVersion_64 ];then
				hw_ver=`${CURR}/getDeviceVersion_64 $dev_num hw`
				so_ver=`${CURR}/getDeviceVersion_64 $dev_num so`
			elif [ -f /usr/bin/getDeviceVersion_64 ];then
                                hw_ver=`getDeviceVersion_64 $dev_num hw`
                                so_ver=`getDeviceVersion_64 $dev_num so`
			else
				hw_ver=""
				so_ver=""
			fi
		else
                        if [ -f ${CURR}/getDeviceVersion_32 ];then
                                hw_ver=`${CURR}/getDeviceVersion_32 $dev_num hw`
                                so_ver=`${CURR}/getDeviceVersion_32 $dev_num so`
                        elif [ -f /usr/bin/getDeviceVersion_64 ];then
                                hw_ver=`getDeviceVersion_32 $dev_num hw`
                                so_ver=`getDeviceVersion_32 $dev_num so`
                        else                                     
                                hw_ver=""
                                so_ver=""
                        fi
		fi
		echo -e $dev_num "\t\t" $dev_desc "\t" $hw_ver "\t" $so_ver >> $output_file
echo -----------------------------------------------------------------------------  >>  $output_file
	done
fi
