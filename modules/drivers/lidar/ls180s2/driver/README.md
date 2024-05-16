# Ls180s2 lidar driver

Lidar configuration is located in `/apollo/modules/drivers/lidar/conf/lidar_config.bp.txt` in the section `ls180s2`. Also do not forget to change lidar driver brand to `LS180S2` in this configuration file.

.dag file is located in `/apollo/modules/drivers/lidar/dag/ls180s2_lidar.dag`.

.launch file is located in `modules/drivers/lidar/launch/driver.dag`. Do not forget change `dag_conf` parameter to `/apollo/modules/drivers/lidar/dag/ls180s2_lidar.dag` value.

To launch module run: `cyber/tools/cyber_launch/cyber_launch.py start /apollo/modules/drivers/lidar/launch/driver.launch`.
