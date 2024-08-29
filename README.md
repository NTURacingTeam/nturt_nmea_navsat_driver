## ntrip

Please refer to the link https://www.waveshare.net/wiki/LC29H(XX)_GPS/RTK_HAT for most of the information.

lc29h_gps_rtk_hat_code: the directory for ntrip rtcm message receiving, from rtk2go.com registered server

``./lc29h_gps_rtk_hat_code`` is reference code directory for NTRIP feature. It's not part of the execution.

### progress
#### rtkRover
Not done:
coordinate converter main.py would convert the received message and convert it into refined position. It's developed for chinese users, so we gotta modify ```./lc29h_gps_rtk_hat_code/python/coordinate_converter```

Integrate the code into ours

#### rtkBase
Already finished: registered rtk2go ntrip caster
```
nturt - username
nturt2023 - password
3.143.243.81 - rtk2go server IP address
2101 - rtk2go server port
T430_32 - mountpoint
```

Not done:
Would configure a dedicated rpi @ the new warehouse. 
code: https://github.com/Stefal/rtkbase

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
