#!/bin/bash
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DRVER_SOURCE=peak-linux-driver-8.5.1
AUTOLOAD_DRIVERLIST=$(grep "biokernbase" /etc/modules)
sudo apt-get install can-utils libpopt-dev
cd $SCRIPT_PATH/$DRVER_SOURCE
make NET=NETDEV_SUPPORT
sudo make install
sudo modprobe pcan
sudo echo "pcan" | sudo tee -a /etc/modules
sudo echo "can_dev" | sudo tee -a /etc/modules


