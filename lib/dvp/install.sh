#!/bin/bash

# This is the main installer for dvp2 Linux
# Run this script as root (superuser).
# (C)DO3THINK / www.do3think.com - 2016
#


CURDIR=`pwd`
echo "Your current directory is $CURDIR. This is where the dvp2 software will be installed..."
A=`whoami`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   exit 1;
fi

create_udev_rules() {
	echo "# do3think dvp2 usb camera driver " > /etc/udev/rules.d/88-dsusb.rules
	echo 'KERNEL=="*", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ACTION=="add", ATTR{idVendor}=="04b4", MODE="666", TAG="dsusb_dev"' >> /etc/udev/rules.d/88-dsusb.rules
	echo 'KERNEL=="*", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ACTION=="remove"' >> /etc/udev/rules.d/88-dsusb.rules
}

extract_qt(){
    tar xf Qt-5.7.0-aarch64.tar.gz -C /usr/local/
}

extract_qt
create_udev_rules
