#!/bin/bash
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DRVER_SOURCE=adv_17V35X
AUTOLOAD_DRIVERLIST=$(grep "biokernbase" /etc/modules)
cd $SCRIPT_PATH/$DRVER_SOURCE
make
sudo make install
sudo echo "adv17v35x" | sudo tee -a /etc/modules
sudo touch /etc/udev/rules.d/rs422.rules
sudo echo 'KERNEL=="ttyAP[0-9]*",MODE="0666"' | sudo tee -a /etc/udev/rules.d/rs422.rules

