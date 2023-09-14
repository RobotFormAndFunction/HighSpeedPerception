import cv2
import numpy as np
import math

import matplotlib.pyplot as plt


# Based on: https://www.youtube.com/watch?v=IjPLZ3hjU1A&list=PL2zRqk16wsdoYzrWStffqBAoUY8XdvatV&index=3
def partial_d(video, x, y, t):
    i_x = (float(video[t][y][x+1]) + float(video[t][y+1][x+1]) + float(video[t+1][y][x+1]) + float(video[t+1][y+1][x+1])) - \
          (float(video[t][y][x  ]) + float(video[t][y+1][x  ]) + float(video[t+1][y][x  ]) + float(video[t+1][y+1][x  ]))
    i_y = (float(video[t][y+1][x]) + float(video[t][y+1][x+1]) + float(video[t+1][y+1][x]) + float(video[t+1][y+1][x+1])) - \
          (float(video[t][y  ][x]) + float(video[t][y  ][x+1]) + float(video[t+1][y  ][x]) + float(video[t+1][y  ][x+1]))
    i_t = (float(video[t+1][y][x]) + float(video[t+1][y+1][x]) + float(video[t+1][y][x+1]) + float(video[t+1][y+1][x+1])) - \
          (float(video[t  ][y][x]) + float(video[t  ][y+1][x]) + float(video[t  ][y][x+1]) + float(video[t  ][y+1][x+1]))

    return (i_x/4, i_y/4, i_t/4)

def getMask(vid):
    # Returns an (Y,X) bit mask for the pixels that are the most likely to contain feature edges worth tracking
    T,Y,X = vid.shape

    window_size = 3
    os = window_size//2 ## offset

    epsilon = 1e-5
    IXes = np.zeros((Y,X))
    IYes = np.zeros((Y,X))
    ICs  = np.zeros((Y,X))
    mask = np.zeros((Y,X))

    for y in range(Y-window_size):
        for x in range(X-window_size):
            M = np.zeros((2,2))
            for _x in range(x, x+window_size):
                for _y in range(y, y+window_size):
                    ix,iy,_ = partial_d(vid,x,y,t=0)
                    M[0][0] += ix**2
                    M[0][1] += ix*iy
                    M[1][0] += ix*iy
                    M[1][1] += iy**2
            eival, eivec = np.linalg.eig(M)
            # print(eival)
            IXes[y,x] = eival[0]
            IYes[y,x] = eival[1]
            ICs [y,x] = IXes[y,x] + IYes[y,x] + IXes[y,x] * IYes[y,x]   # ends up as all zeroes if we do the proper I_X * I_Y as specified in the Harris corner detector

            # TODO: optimize runtime to reduce redundant calculations of each derivative

    # Normalize ranges to max 250 for comprehensibility
    mx = ICs.max()
    IXes *= 250/mx
    IYes *= 250/mx
    ICs  *= 250/mx


    # ignore all pixels below 50 for the sake of faster compute of flow
    for y in range(Y-window_size):
        for x in range(X-window_size):
            mask[y,x] = np.sign(ICs[y,x] - 50)

    return mask

def plotUV(U,V):
    # print(U,V)
    # https://matplotlib.org/stable/gallery/images_contours_and_fields/quiver_simple_demo.html#sphx-glr-gallery-images-contours-and-fields-quiver-simple-demo-py
    fig, ax = plt.subplots()
    q = ax.quiver(np.arange(U.shape[0])*5, np.arange(U.shape[1])*5, U, V, scale=25)
    plt.show()

def get_best_offset_for(patch1, img2, x, y, patch_size, search_area=5):
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

def ssd(patch1, patch2) -> tuple:
    # Calculate the sum of squared differences between the two patches
    # print("Comparing\n", patch1.shape, "\nvs\n", patch2.shape)
    diff = 0
    for l1, l2 in zip(patch1, patch2):
        for p1, p2 in zip(l1, l2):
            diff += (float(p1)-float(p2))**2
    return diff


img0 = cv2.imread('frame1.jpg',0)
img1 = cv2.imread('frame2.jpg',0)
vid = np.stack([img0,img1])
mask = getMask(vid)


X=Y=96
s=5
U = np.zeros((Y//s, X//s))
V = np.zeros((Y//s, X//s))

for y in range(0, Y-s+1):
    for x in range(0, X-s+1):
        if mask[y,x] < 1: continue # skip pixels that are unlikely to have strong correspondence

        patch0 = img0[y:y+s, x:x+s]

        u,v = get_best_offset_for(patch0, img1, x,y,s)
        U[y//s][x//s] = u
        V[y//s][x//s] = v

        patch1 = img1[y+v:y+v+s, x+u:x+u+s]

cv2.imshow(f"Image at time t=0: ", cv2.resize(img0,     (400,400), interpolation=cv2.INTER_NEAREST))
cv2.imshow(f"Image at time t=1: ", cv2.resize(img1,     (400,400), interpolation=cv2.INTER_NEAREST))
cv2.imshow(f"Corner mask t=0,1  ", cv2.resize(mask*250, (400,400), interpolation=cv2.INTER_NEAREST))
plotUV(U,V)
cv2.waitKey(0)
cv2.destroyAllWindows()
