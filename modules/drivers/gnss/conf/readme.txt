first check imutoantoffset saved in device, method below:
screen /dev/ttyUSB2

log imutoantoffsets

---- GKV ----

sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

ll /sys/class/tty/ttyUSB*

export GLOG_alsologtostderr=1

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

cyber/tools/cyber_launch/cyber_launch.py start /apollo/modules/drivers/gnss/launch/gnss.launch

cyber/tools/cyber_launch/cyber_launch.py start /apollo/modules/localization/launch/rtk_localization.launch