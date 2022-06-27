#load the c++ lib

#set the defualt IPAddr
#ifconfig eth0 10.60.2.222

#Set CAN0
#bring down the device
#set the bitrate 500K == 1000K
#bring up the device

ip link set can0 type can bitrate 1000000
ip link set can0 up

#Set CAN1
ip link set can1 type can bitrate 1000000
ip link set can1 up

# Excute the program
./build/src/ccr_quick_detection/ccr_core



