#!/bin/bash 
xterm -e ./readRS422 /dev/ttyAP0 &
xterm -e ./readRS422 /dev/ttyAP1 &
xterm -e ./readRS422 /dev/ttyAP2 &
xterm -e ./readRS422 /dev/ttyAP7 &
xterm -e ./readRS422 /dev/ttyAP4 &
xterm -e ./readRS422 /dev/ttyAP5 &
xterm -e ./readRS422 /dev/ttyAP6 
