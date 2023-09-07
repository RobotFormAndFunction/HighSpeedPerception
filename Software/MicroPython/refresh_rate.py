### MODIFIED FROM: https://github.com/Seeed-Studio/wiki-documents/files/12375085/XIAO.ESP32S3.Micropython.zip
### STREAMING_CLIENT.PY

# Windows Setup instructions: https://wiki.seeedstudio.com/XIAO_ESP32S3_Micropython/

# run this on ESP32 Camera

import esp
import camera
from time import sleep, ticks_ms

# https://github.com/lemariva/micropython-camera-driver
# https://github.com/shariltumin/esp32-cam-micropython-2022
## ESP32-CAM (default configuration) - https://bit.ly/2Ndn8tN
cam = camera.init()
print("Camera ready?: ", cam)

while not cam:
    sleep(2)
    cam = camera.init()
    print("Camera ready?: ", cam)

if cam:
    # set preffered camera setting
    camera.framesize(10)     # frame size 800X600 (1.33 aspect ratio)
    camera.contrast(2)       # increase contrast
    camera.speffect(2)       # jpeg grayscale

    while True:
        pic=camera.capture()
        if pic:
            print("Picture taken. time:", ticks_ms(), len(pic))
        else:
            print("failure")