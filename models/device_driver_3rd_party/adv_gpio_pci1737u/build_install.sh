#!/bin/bash
# $ sudo vim /etc/modprobe.d/blacklist.conf
# Add the following at the end of file

SCRIPT_PATH=$PWD
DRVER_SOURCE=linux_driver_source_3.2.11.0_64bit
PRODUCT=pci1737_pci1739
#PRODUCT=pci1730_pcm3730i
#HARDWARE_CARDS=bio1730
HARDWARE_CARDS=bio1737
BLACKLIST=$(grep "blacklist adv_pci_dio" /etc/modprobe.d/blacklist.conf)
AUTOLOAD_DRIVERLIST=$(grep "biokernbase" /etc/modules)

sudo cp /etc/modprobe.d/blacklist.conf  /etc/modprobe.d/blacklist.conf.bak
if [ -z "$BLACKLIST" ]; then
    echo "blacklist adv_pci_dio" | sudo tee -a /etc/modprobe.d/blacklist.conf
    echo "blacklist comedi" | sudo tee -a /etc/modprobe.d/blacklist.conf
    echo "blacklist adv_pci1710" | sudo tee -a /etc/modprobe.d/blacklist.conf
fi
#echo "blacklist bio1730" | sudo tee -a /etc/modprobe.d/blacklist.conf
#
# Build kernel module
cd  $SCRIPT_PATH/$DRVER_SOURCE/drivers/driver_base/src/lnx_ko
make
sudo make install
# Build the PCI-1737U  module image
cd $SCRIPT_PATH/$DRVER_SOURCE/drivers/$PRODUCT/src/lnx_ko
cp $SCRIPT_PATH/$DRVER_SOURCE/drivers/lib/Module.symvers $SCRIPT_PATH/$DRVER_SOURCE/drivers/$PRODUCT/src/lnx_ko
make
sudo make install
# Auto load the drver
if [ -z "$AUTOLOAD_DRIVERLIST" ]; then
    sudo echo "loop" | sudo tee -a /etc/modules
    sudo echo "biokernbase" | sudo tee -a /etc/modules
    sudo echo "$HARDWARE_CARDS" | sudo tee -a /etc/modules
fi
# Copy all dynamic lib to /usr/lib
sudo cp $SCRIPT_PATH/$DRVER_SOURCE/libs/* /usr/lib/

# Copy the header file to the system directory.
sudo cp $SCRIPT_PATH/$DRVER_SOURCE/inc/bdaqctrl.h /usr/include/
sudo mkdir -p /usr/share/advantech
sudo cp $SCRIPT_PATH/$DRVER_SOURCE/inc/Automation.BDaq.jar /usr/share/advantech/
sudo chmod 444 /usr/share/advantech/Automation.BDaq.jar
