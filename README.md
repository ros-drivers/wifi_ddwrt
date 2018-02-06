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

