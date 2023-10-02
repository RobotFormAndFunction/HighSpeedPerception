### Show all images saved to the SD card

# Test for non-empty: (format-hex D:\image112782.U  )|?{$_ -notlike "*$("00 "*16)*"}

import os
import cv2
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


# Configuration segment copied from MatPlotLib documentation:
# https://matplotlib.org/stable/users/explain/colors/colormap-manipulation.html#creating-listed-colormaps
top = mpl.colormaps['Oranges_r'].resampled(128)
bottom = mpl.colormaps['Blues'].resampled(128)

newcolors = np.vstack((top(np.linspace(0, 1, 128)),
                       bottom(np.linspace(0, 1, 128))))
non_zero_cmap = ListedColormap(newcolors, name='OrangeBlue')
# End of config

# Plotting function modified from MatPlotLib plot_examples:
# https://matplotlib.org/stable/users/explain/colors/colormap-manipulation.html#creating-listed-colormaps
def plot_u(U):
    fig, ax = plt.subplots()
    ax.invert_yaxis()
    psm = ax.pcolormesh(U, cmap=non_zero_cmap, vmin=-10, vmax=10)
    fig.colorbar(psm, ax=ax)



def plotUV(U,V):
    # print(U,V)
    # https://matplotlib.org/stable/gallery/images_contours_and_fields/quiver_simple_demo.html#sphx-glr-gallery-images-contours-and-fields-quiver-simple-demo-py
    fig, ax = plt.subplots()
    q = ax.quiver(np.arange(U.shape[0]), -np.arange(U.shape[1]), U, V, scale=50)
    plt.show()


camera_dim_options = [
    (240,320),
    (96,96)
]
base_dir = 'D:\\'
files = os.listdir(base_dir)
files = [f for f in files if ".bytes" in f]    # look for just files with .bytes suffix
times = [f.split(".")[0][len("image"):] for f in files]   # Extract timestamps for output in cv2 windows
files = [f"{base_dir}{f}" for f in files]      # Prepend 'D:\' to file paths for accessing

# print(files)
# print(times)
# exit()

i=0
for f,t in zip(files, times):
    print(f"Reading file {f}")
    i+=1
    if i>10:
        break
    # read into NP example: https://www.geeksforgeeks.org/reading-binary-files-in-python/
    with open(f, 'rb') as byteBuff:
        bytes = np.fromfile(byteBuff, dtype=np.uint8)

        # Determine image dimensions based on file size
        camera_dims = (0,0)
        for d in camera_dim_options:
            if d[0]*d[1] == len(bytes):
                camera_dims = d
                break
        # Transform 1D byte array into 2d image grid of bytes
        img0 = bytes.reshape(camera_dims)
        cv2.imshow(f"Image at time t={t}: ", cv2.resize(img0,     (400,400), interpolation=cv2.INTER_NEAREST))

    with open(f.replace("bytes", "U"), "rb") as uByteBuff:
        imgU = np.fromfile(uByteBuff, dtype=np.int8).reshape(camera_dims)
        plot_u(imgU)
    with open(f.replace("bytes", "V"), "rb") as vByteBuff:
        imgV = np.fromfile(vByteBuff, dtype=np.int8).reshape(camera_dims)
    
    plotUV(imgU, imgV)
        
cv2.waitKey(0)
cv2.destroyAllWindows()
