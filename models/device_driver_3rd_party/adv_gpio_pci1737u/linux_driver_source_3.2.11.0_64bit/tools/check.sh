#!/bin/sh   


checkKernel="Fail" 
kernel=`uname -r` 

checkKernelRet=`find /usr/src -iname *${kernel}*`
if [ "$checkKernelRet" = "" ]; then
echo "Please install the kernel source code manually!"
else
echo "Check Kernel OK!"
checkKernel="Pass"
fi


checkGcc="Fail"
checkGccRet=`ls /usr/bin | grep gcc`
if [ "$checkGccRet" != "" ]; then
checkGcc="Pass"
fi
checkGccRet=`ls /sbin | grep gcc`
if [ "$checkGccRet" != "" ]; then
checkGcc="Pass"
fi
checkGccRet=`ls /bin | grep gcc`
if [ "$checkGccRet" != "" ]; then
checkGcc="Pass"
fi

if [ "$checkGcc" = "Pass" ]; then
echo "Check gcc OK!"
else
echo "Please install gcc manually!"
fi




