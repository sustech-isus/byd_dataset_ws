
socket can

install driver; ~/Download/socketcan_kvaser_pciefd
```
make
sudo make install
sudo make load
```
note the linuxcan kvaser driver must be removed.

ifconfig show can interfaces.

config and test
```
sudo ip link set can2 type can bitrate 500000 dbitrate 1000000 fd on
sudo ip link set can2 up
candump can2
```
