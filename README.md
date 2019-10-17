# ROS driver for ublox ZED-F9P receiver

This is for the ArduSimple ZED-F9P boards. (Mainly affects which port on the board is being used.) I am connecting the F9P board to a Linux laptop with USB cable. (MovingBaseline requires connecting a single wire between two receiver boards.)

I copied the ublox ROS driver from https://github.com/bao-eng/ublox

Looking at my initial commit comments I really didn't change anything to get it to work.

I added ROS topic RTCM correction support so the board produces an RTK fix. (The correction data is sent through the ROS driver node.) The RTCM server is here: https://github.com/ros-agriculture/ntrip_ros A similar server (may be better than mine) is here: https://github.com/dayjaby/ntrip_ros

I added launch files and yaml files to allow running two F9P boards at the same time. This allows operation in MovingBaseline mode. 

**I need to add the uBlox config files which get manually loaded on to the boards through uBlox u-center software.**

Below is original README from bao-eng that I started with:

=======================================================================

# ROS driver for ublox ZED-F9P receiver

Just quick hardcode to publish some UBX messages to ROS.
Disabled configuration of the receiver via yaml. Reciever should be configured via u-center software.
Hardcoded to work as HPG Rover device.
NavRELPOSNED.msg updated to match u-blox 9 protocol version 27.1

## Options

zed-f9p.yaml (only for seting up device connection and published messages)

## Launch

```roslaunch ublox_gps ublox_zed-f9p.launch```

