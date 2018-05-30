#!/bin/bash
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DRVER_SOURCE=linux_driver_source_3.2.11.0_64bit
PRODUCT=pci1737_pci1739
#PRODUCT=pci1730_pcm3730i
# Build kernel module
cd  $SCRIPT_PATH/$DRVER_SOURCE/drivers/driver_base/src/lnx_ko
make clean
# Build the PCI-1737U  module image

cd $SCRIPT_PATH/$DRVER_SOURCE/drivers/$PRODUCT/src/lnx_ko
make clean
rm -rf $SCRIPT_PATH/$DRVER_SOURCE/drivers/lib/Module.symvers
rm -rf $SCRIPT_PATH/$DRVER_SOURCE/drivers/bin/*.ko
rm -rf $SCRIPT_PATH/$DRVER_SOURCE/examples/C++_Console/DI_DIInterrupt/DIInterrupt