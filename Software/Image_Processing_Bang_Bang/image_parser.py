### Show all images saved to the SD card

import os
import cv2
import numpy as np

camera_dims = 240,320

base_dir = 'D:\\'
files = os.listdir(base_dir)
files = [f for f in files if ".bytes" in f]
times = [f.split(".")[0][len("image"):] for f in files]
files = [f"{base_dir}{f}" for f in files]

# print(files)
# print(times)
# exit()

for f,t in zip(files, times):
    # read into NP example: https://www.geeksforgeeks.org/reading-binary-files-in-python/
    with open(f, 'rb') as byteBuff:
        bytes = np.fromfile(byteBuff, dtype=np.uint8)
        img0 = bytes.reshape(camera_dims)
        cv2.imshow(f"Image at time t={t}: ", cv2.resize(img0,     (400,400), interpolation=cv2.INTER_NEAREST))
cv2.waitKey(0)
cv2.destroyAllWindows()
