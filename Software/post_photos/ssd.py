import cv2
import numpy as np
import math

import matplotlib.pyplot as plt

def plotUV(U,V):
    # print(U,V)
    # https://matplotlib.org/stable/gallery/images_contours_and_fields/quiver_simple_demo.html#sphx-glr-gallery-images-contours-and-fields-quiver-simple-demo-py
    fig, ax = plt.subplots()
    q = ax.quiver(np.arange(U.shape[0]), np.arange(U.shape[1]), U, V)
    plt.show()



def get_grid_ssds(img0, img1):
    Y,X = img0.shape
    patch_size = 5   # 5x5 pixel patches of the image
    U = np.zeros((Y//patch_size, X//patch_size))
    V = np.zeros((Y//patch_size, X//patch_size))

    for y in range(0, Y-patch_size+1, patch_size):
        for x in range(0, X-patch_size+1, patch_size):
            patch0 = img0[y:y+patch_size, x:x+patch_size]
            # print("Patch 0:", patch0.shape)

            min_diff = 2**20  # some large number
            u = min_diff
            v = min_diff
            # Scan for best match around the source point

    return U, V


def get_best_offset_for(patch1, img2, x,y,patch_size,search_area=8):
    Y,X = img2.shape
    best_u = 0
    best_v = 0
    min_diff = 2**20  # some large number

    for y2 in range(-search_area, search_area):
        for x2 in range(-search_area, search_area):
            _y = y+y2
            _x = x+x2
            if not (0 <= _y < Y-patch_size and 0 <= _x < X-patch_size):
                # print(f"Skipping ({x2},{y2})")
                continue
            # else:
                # print(f"Processing ({x2},{y2})")
            patch2 = img2[_y:_y+patch_size, _x:_x+patch_size]
            if patch2.shape != patch1.shape:
                # print("Mismatch in shapes:")
                # print(patch1.shape)
                # print(patch2.shape)
                continue
            diff = ssd(patch1, patch2)
            if diff < min_diff:
                min_diff = diff
                best_u = x2
                best_v = y2
    
    return (best_u, best_v)


# Calculate the sum of squared differences between the two patches
def ssd(patch1, patch2) -> tuple:
    # print("Comparing\n", patch1.shape, "\nvs\n", patch2.shape)
    diff = 0
    for l1, l2 in zip(patch1, patch2):
        for p1, p2 in zip(l1, l2):
            diff += (float(p1)-float(p2))**2
    return diff

img0 = cv2.imread('frame1.jpg',0)
img1 = cv2.imread('frame2.jpg',0)


# print(type(img))
# U,V = get_grid_ssds(img0, img1)
# plotUV(U,V)

X=Y=96
s=5
U = np.zeros((Y//s, X//s))
V = np.zeros((Y//s, X//s))

for y in range(0, Y-s+1, s):
    for x in range(0, X-s+1, s):
        patch0 = img0[y:y+s, x:x+s]

        u,v = get_best_offset_for(patch0, img1, x,y,s)
        U[y//s][x//s] = u
        V[y//s][x//s] = v

        patch1 = img1[y+v:y+v+s, x+u:x+u+s]

cv2.imshow(f"Image at time t=0: ", cv2.resize(img0, (400,400), interpolation=cv2.INTER_NEAREST))
cv2.imshow(f"Image at time t=1: ", cv2.resize(img1, (400,400), interpolation=cv2.INTER_NEAREST))
plotUV(U,V)
cv2.waitKey(0)
cv2.destroyAllWindows()
