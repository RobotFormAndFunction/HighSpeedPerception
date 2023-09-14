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
            ICs [y,x] = IXes[y,x] + IYes[y,x]

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


img0 = cv2.imread('frame1.jpg',0)
img1 = cv2.imread('frame2.jpg',0)
vid = np.stack([img0,img1])
print(vid[0].shape)
mask = getMask(vid)

cv2.imshow(f"Image at time t=0: ", cv2.resize(img0, (400,400), interpolation=cv2.INTER_NEAREST))
cv2.imshow(f"Image at time t=1: ", cv2.resize(img1, (400,400), interpolation=cv2.INTER_NEAREST))
cv2.imshow(f"Edge Mask:     ", cv2.resize(mask*250, (400,400), interpolation=cv2.INTER_NEAREST))

cv2.waitKey(0)
cv2.destroyAllWindows()
