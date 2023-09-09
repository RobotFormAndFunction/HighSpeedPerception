import cv2
import numpy as np
import math


def compute_patch_uv(video, W_x, W_y, W_t, W_l):
    # W_x: Starting index within W along the X axis
    # W_y: Starting index within W along the Y axis
    # W_t: Starting time step for the computation
    # W_l: Number of elements in W along the X and Y axes.  Assumed to be a square
    # Computes (u,v)--> for an image patch

    partials = []  # Initialize partial derivatives grid of triples

    Ixx = 0
    Ixy = 0
    Iyy = 0
    Ixt = 0
    Iyt = 0
    
    # Compute all necessary partials and sums
    for x in range(W_x, W_x+W_l):
        partials.append([])
        for y in range(W_y, W_y+W_l):
            p = partial_d(video, x, y, W_t)
            ix, iy, it = p
            partials[-1].append(p)
            Ixx += ix * ix 
            Ixy += ix * iy
            Iyy += iy * iy
            Ixt += ix * it
            Iyt += iy * it
            
    A = np.array([
        [Ixx, Ixy],
        [Ixy, Iyy]
    ])

    ATB = np.array([
        -Ixt,
        -Iyt
    ])

    ATA = np.dot(A.T, A)

    ATA_inv = np.linalg.inv(ATA)

    U = np.dot(ATA_inv, ATB)

    return U


# Based on: https://www.youtube.com/watch?v=IjPLZ3hjU1A&list=PL2zRqk16wsdoYzrWStffqBAoUY8XdvatV&index=3
def partial_d(video, x, y, t):
    i_x = video[t][y][x+1] + video[t][y+1][x+1] + video[t+1][y][x+1] + video[t+1][y+1][x+1] - \
         (video[t][y][x]   + video[t][y+1][x]   + video[t+1][y][x]   + video[t+1][y+1][x])
    
    i_y = video[t][y+1][x] + video[t][y+1][x+1] + video[t+1][y+1][x] + video[t+1][y+1][x+1] - \
         (video[t][y][x]   + video[t][y][x+1]   + video[t+1][y][x]   + video[t+1][y][x+1])

    i_t = video[t+1][y][x] + video[t+1][y+1][x] + video[t+1][y][x+1] + video[t+1][y+1][x+1] - \
         (video[t][y][x]   + video[t][y+1][x]   + video[t][y][x+1]   + video[t][y+1][x+1])

    return (i_x/4, i_y/4, i_t/4)

# Based on FPoCV OF Constraint Equation
# vector u = u_n (normal) + u_p (parallel)
def get_u_n(ix,iy,it):
    magnitude = np.abs(it) / euclidian(ix,iy)
    return (ix * magnitude, iy*magnitude)

def euclidian(x,y):
    return (x**2 + y**2)**0.5

def downsample(img):
    # Downscales images in 2x2 grid pattern
    gs = 2 # Grid size of single axis (2x2)
    y2,x2 = img.shape
    y1 = math.ceil(y2/gs)
    x1 = math.ceil(x2/gs)

    # smaller image
    simg = np.zeros((y1, x1), dtype=np.uint8)
    for y in range(y1):
        for x in range(x1):
            for xi in range(gs):
                for yi in range(gs):
                    simg[y][x] += img[gs*y+yi][gs*x+xi] / (gs**2)

    # print(img.shape)
    # print(type(img[0][0]))
    # print(img)

    # print(simg.shape)
    # print(simg)

    return simg










img0 = cv2.imread('frame1.png',0)
img1 = cv2.imread('frame2.png',0)

video = np.stack([
    img0, img1
])

f = video[0]

for i in range(4):
    cv2.imshow(f"frame 1, /{2**i}", f)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    f = downsample(f)

# print(type(img))
print(video.shape)

