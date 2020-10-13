#!/bin/bash
echo "USB-CAN Udev Rule"

echo "Check them using the command : lsusb | grep Microchip"
echo "Start to copy udev rules to  /etc/udev/rules.d/"

sudo cp `rospack find huaxun_radar`/scripts/udev/99-can2usb.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Finish "
