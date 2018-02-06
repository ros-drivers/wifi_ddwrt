# wifi_ddwrt

The main documentation for the wifi_ddwrt package is the [wifi_ddwrt](http://wiki.ros.org/wifi_ddwrt) page on the ROS wiki.

For Hydro use:

    git clone -b hydro-devel https://github.com/ros-drivers/wifi_ddwrt.git

For Indigo and newer versions of ROS, use the hydro-devel branch:

    git clone -b indigo-devel https://github.com/ros-drivers/wifi_ddwrt.git

For Groovy and older version of ROS, use the master branch:

    git clone -b master https://github.com/ros-drivers/wifi_ddwrt.git


## Modifications

The main modifications where made to the ddwrt.py file making it compatible with more devices and change the way information
is published using diagnostic messages instead of PR2 messages.

The other nodes are untouch in terms of functionality. The only thing that was done was a small fix removing a warning
related to the Publishers under Indigo



# Usage

- Edit the config.yaml inside the config folder with your router (AP) info
- Then execute the following command:
    roslaunch wifi_ddwrt wifi_ddwrt.launch


### Output (diagnostic aggregator)

```
Full Name: /Other/wifi_ddwrt: wifi_ap
Component: wifi_ddwrt: wifi_ap
Hardware ID: RouterXPTO
Level: OK
Message: AP AGV_24 [192.168.1.1] working OK

essid: XPTO_24
channel: 5
rate: 288.9 Mb/s
tx_power: 20 dBm
ath1: [{'macaddr': 'AA:AA:AA:AA:AA:AA', 'signal': -76, 'nt_devices': {'bytes': '98906417', 'frame': '0', 'drop': '0', 'packets': '228663', 'fifo': '0', 'multicast': '0', 'colls': '0', 'compressed': '0', 'carrier': '0', 'errs': '0'}, 'noise': -98, 'snr': 22, 'quality': 48}, {'macaddr': 'BB:BB:BB:BB:BB:BB', 'signal': -78, 'nt_devices': {'bytes': '98908498', 'frame': '0', 'drop': '0', 'packets': '228675', 'fifo': '0', 'multicast': '0', 'colls': '0', 'compressed': '0', 'carrier': '0', 'errs': '0'}, 'noise': -98, 'snr': 20, 'quality': 44}]
ath2: [{'macaddr': 'AA:AA:AA:AA:AA:AA', 'signal': -76, 'nt_devices': {'bytes': '98906697', 'frame': '0', 'drop': '0', 'packets': '228663', 'fifo': '0', 'multicast': '0', 'colls': '0', 'compressed': '0', 'carrier': '0', 'errs': '0'}, 'noise': -96, 'snr': 23, 'quality': 48}, {'macaddr': 'BB:BB:BB:BB:BB:BB', 'signal': -78, 'nt_devices': {'bytes': '98908498', 'frame': '0', 'drop': '0', 'packets': '223075', 'fifo': '0', 'multicast': '0', 'colls': '0', 'compressed': '0', 'carrier': '0', 'errs': '0'}, 'noise': -96, 'snr': 20, 'quality': 43}]
```
