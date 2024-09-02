# NTURT NMEA NAVSAT Driver

Author: [CHYang25](https://github.com/CHYang25)

### Description

Please refer to the link https://www.waveshare.net/wiki/LC29H(XX)_GPS/RTK_HAT for most of the information. This package is mainly derived from [ntrip rover sample code](https://www.waveshare.net/w/upload/1/14/Lc29h_gps_rtk_hat_code.zip) and [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver/tree/269d288475db2a97221445ea1e9104c80aeed00f). Ntrip feature is added to the original nmea_navsat_driver module, which publishes nmea messages to ros2 topics.

### Ntrip 
Ntrip (Networked Transport of RTCM via Internet Protocol). The Networked Transport of RTCM via Internet Protocol (NTRIP) is a protocol for streaming differential GPS (DGPS) corrections over the Internet for real-time kinematic positioning. (From wikipedia)
1. Ntrip Caster: [rtk2go](http://rtk2go.com/), [rtk2go_how_to_connect](http://rtk2go.com/how-to-connect/), [rtk2go_mountpoint_list](http://www.rtk2go.com:2101/SNIP::STATUS#single). Ntrip caster collects data from Ntrip base server and send the RTCM correction message to Ntrip clients.
2. Ntrip Server: Serve as a base with a permanent reference point. In Taiwan, use the ``Navi-test`` mountpoint as Ntrip server (or we shall setup our own server).
3. Ntrip Client: The rpi device on the car with the lc29h_gps_rtk_hat will be the ntrip client. The code added to the original nmea_navsat_driver is mainly about this characeter.

Specific Ntrip metadata:
1. username: any valid email address.
2. password: nturt2023
3. mountpoint: Navi-test
4. Ntrip Caster IP (rtk2go): 3.143.243.81
5. Ntrip Caster Port: 2101

### Package Description
1. ``./lc29h_gps_rtk_hat_code``: reference code directory for NTRIP feature. It's not part of the execution.
2. ``./config/nmea_ntrip_driver.yaml``: the configuration for ntrip module.
3. ``./launch/nmea_ntrip/driver.launch.py``: the launch file for ntrip module.
4. ``./src/libnmea_navsat_driver/nodes/nmea_ntrip_driver.py``: ntrip module main function.
5. ``./src/libnmea_navsat_driver/driver.py``: driver module file with ``NtripClient`` class, which is derived from ``./lc29h_gps_rtk_hat_code/python/rtk_rover/main.py``

### Dependency
```
pip install -t requirements.txt
```

### Execution
To execute this node, receiving ntrip-corrected nmea messages and publishing to the /fix and /vel topics:
```
ros2 run nturt_nmea_navsat_driver nmea_ntrip_driver
```
If verbose debug information is needed:
```
ros2 run nturt_nmea_navsat_driver nmea_ntrip_driver --ros-args --log-level debug
```

### references:
- **rtkhat (cheaper model): https://www.waveshare.net/wiki/LC29H(XX)_GPS/RTK_HAT**
- rtcm msm: https://genesys-offenburg.de/support/application-aids/gnss-basics/the-rtcm-multiple-signal-messages-msm/
- snip: https://www.use-snip.com/about-snip/?gad_source=1&gclid=CjwKCAjwps-zBhAiEiwALwsVYSfkNkqc-UDDzF13CKCMwh2ZGY-UVrZodJCpEW_hZvUB1Ww-aXpZWhoCbkEQAvD_BwE
- rtk2go: http://rtk2go.com/
- rtk hackmd: https://hackmd.io/ESTtRuTTRGyngEU4xf0vkw
- openocd: https://openocd.org/
- remote debugging on rpi: https://forums.raspberrypi.com/viewtopic.php?t=300039
- openocd hackmd: https://hackmd.io/@QuantumSpawner/Hk_E8OK0h
- elf file: https://en.wikipedia.org/wiki/Executable_and_Linkable_Format
- tailscale console: https://login.tailscale.com/admin/users
- rtk2go caster mountpoint list: http://www.rtk2go.com:2101/SNIP::STATUS#single
- nmea message format: https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html

