# HighSpeedPerception

Hardware and software for a high-speed vision-enabled millirobot

## Instructions

### Getting WiFi image server working

After flashing the esp32s3 Sense with CameraWebServer, the following text should appear in the Serial Monitor:

```
load:0x403cc700,len:0x2a68
entry 0x403c98d4
Powering on
MAC: AA:AA:AA:AA:AA:AA
...............................
```

Go to: https://clearpass-portal1.cit.buffalo.edu/guest/mac_create.php

Register the MAC address for the microrobot.
These device registrations will last for one year.

## Debugging

### Permission Error 13 while flashing the board

Error message form:

```
Sketch uses 2933733 bytes (87%) of program storage space. Maximum is 3342336 bytes.
Global variables use 74008 bytes (22%) of dynamic memory, leaving 253672 bytes for local variables. Maximum is 327680 bytes.
esptool.py v4.5.1
Serial port COM9
Connecting...
Chip is ESP32-S3 (revision v0.1)
Features: WiFi, BLE
Crystal is 40MHz
.
.
.
Compressed 2934096 bytes to 2061845...
Writing at 0x00010000... (0 %)
Writing at 0x0001437f... (1 %)
.
.
.
Writing at 0x000f400a... (38 %)
Writing at 0x000f8811... (39 %)

A serial exception error occurred: Cannot configure port, something went wrong. Original message: PermissionError(13, 'The device does not recognize the command.', None, 22)
Note: This error originates from pySerial. It is likely not a problem with esptool, but with the hardware connection or drivers.
For troubleshooting steps visit: https://docs.espressif.com/projects/esptool/en/latest/troubleshooting.html
Failed uploading: uploading error: exit status 1
```

Resolution:  
[Xiao Instructions](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#bootloader-mode)

1. Disconnect the board from the computer.
   - Recommended to disconnect computer from USB cable as the B button on the board is small.
1. Hold down the B button on the board.
1. While continuing to hold down the button, reconnect the cable connecting the board and computer.
1. Attempt to flash again.

This procedure has so far been successful in resolving all instances of this issue.
