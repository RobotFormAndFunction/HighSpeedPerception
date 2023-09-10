import cv2
import numpy as np
import math


# Computes (u,v)--> for an image patch
def compute_patch_uv(video, W_x, W_y, W_t, W_l):
    # W_x: Starting index within W along the X axis
    # W_y: Starting index within W along the Y axis
    # W_t: Starting time step for the computation
    # W_l: Number of elements in W along the X and Y axes.  Assumed to be a square

    # partials = []  # Initialize partial derivatives grid of triples

    Ixx = 0
    Ixy = 0
    Iyy = 0
    Ixt = 0
    Iyt = 0
    
    # Compute all necessary partials and sums
    for x in range(W_x, W_x+W_l):
        # partials.append([])
        for y in range(W_y, W_y+W_l):
            p = partial_d(video, x, y, W_t)
            ix, iy, it = p
            # partials[-1].append(p)
            Ixx += ix * ix 
            Ixy += ix * iy
            Iyy += iy * iy
            Ixt += ix * it
            Iyt += iy * it
            
    A = np.array([
        [Ixx, Ixy],
        [Ixy, Iyy]
    ])
    # print("A:", A)

    ATB = np.array([
        -Ixt,
        -Iyt
    ])


    ATA = np.dot(A.T, A)
    # print("ATA:", ATA)

    det = np.linalg.det(ATA)

    if det != 0:
        ATA_inv = np.linalg.inv(ATA)
        U = np.dot(ATA_inv, ATB)
    else:
        U = np.zeros((2,))   # Determinant is 0 -> no optical flow in the region
    return U


# Based on: https://www.youtube.com/watch?v=IjPLZ3hjU1A&list=PL2zRqk16wsdoYzrWStffqBAoUY8XdvatV&index=3
def partial_d(video, x, y, t):
    i_x = (float(video[t][y][x+1]) + float(video[t][y+1][x+1]) + float(video[t+1][y][x+1]) + float(video[t+1][y+1][x+1])) - \
          (float(video[t][y][x  ]) + float(video[t][y+1][x  ]) + float(video[t+1][y][x  ]) + float(video[t+1][y+1][x  ]))
    i_y = (float(video[t][y+1][x]) + float(video[t][y+1][x+1]) + float(video[t+1][y+1][x]) + float(video[t+1][y+1][x+1])) - \
          (float(video[t][y  ][x]) + float(video[t][y  ][x+1]) + float(video[t+1][y  ][x]) + float(video[t+1][y  ][x+1]))
    i_t = (float(video[t+1][y][x]) + float(video[t+1][y+1][x]) + float(video[t+1][y][x+1]) + float(video[t+1][y+1][x+1])) - \
          (float(video[t  ][y][x]) + float(video[t  ][y+1][x]) + float(video[t  ][y][x+1]) + float(video[t  ][y+1][x+1]))

    return (i_x/4, i_y/4, i_t/4)

# Based on FPoCV OF Constraint Equation
# vector u = u_n (normal) + u_p (parallel)
def get_u_n(ix,iy,it):
    magnitude = np.abs(it) / euclidian(ix,iy)
    return (ix * magnitude, iy*magnitude)

def euclidian(x,y):
    return (x**2 + y**2)**0.5

def downsample(img, gs=2):
    # img = numpy 2d array of pixels representing a greyscale image
    # gs = Grid size of single axis (2x2)
    # Downscales images in 2x2 grid pattern
    y2,x2 = img.shape
    y1 = math.ceil(y2/gs)
    x1 = math.ceil(x2/gs)

    # smaller image
    simg = np.zeros((y1, x1), dtype=np.uint8)
    for y in range(y2):
        for x in range(x2):
            simg[y//gs][x//gs] += img[y][x] / gs**2

    return simg


def compute_image_uv(img1, img2):
    # Input: 2x (NxM) images
    # Output: 2x NxM vectors u,v indicating x and y optical flow between frames 
    # Computes the optical flow UV difference between pixels in two images just at that scale of the pyramid
    Y,X = img1.shape

    video = np.stack([img1, img2])

    u = np.zeros((Y,X))
    v = np.zeros((Y,X))

    # -1 avoids going outside of the bounding boxes
    for y in range(Y-1):
        for x in range(X-1): 
            U = compute_patch_uv(video, x, y, W_t=0, W_l=1)
            u[y][x] = U[0]
            v[y][x] = U[1]

    return u,v

def computeFlow(frame1, frame2):
    # start with the coarsest settings and work upwards
    Y,X = frame1.shape
    
    # The maximum amount of divisions that can be done before 
    # the image dims are reduced to an elem of {2x2, 2xN, Nx2}
    max_divs = int(math.log2(min(X,Y))) # - 2   # -1 avoids actually operating on a 2x2 or 4x4 unit.  Minimum size is thus 8x8 = 64 pixels

    video = np.stack([
        frame1, frame2
    ])

    subsamples1 = [frame1]
    subsamples2 = [frame2]

    for i in range(max_divs):
        cv2.imshow(f"frame 1, /{2**i}", subsamples1[-1])
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        subsamples1.append(downsample(subsamples1[-1]))
        subsamples2.append(downsample(subsamples2[-1]))     
        print("Created frame of shape: ", subsamples1[-1].shape)

    # Cumulative vector offsets in each direction
    u_c = np.zeros((Y,X))
    v_c = np.zeros(subsamples1[0].shape)

    # Starting with the smallest image, compute optical flow
    for i in range(max_divs, -1, -1):
        u,v = compute_image_uv(subsamples1[i], subsamples2[i])
        # print("U", u.shape)
        # print("V", v.shape)

        # After computing the u,v at a higher level, use that information to modify the next layer down
        sf = 2**i # scale factor (the factor by which source u,v need to be expanded ot the cumulative u,v)

        for x in range(X):
            for y in range(Y):
                u_c[y][x] += u[y//sf][x//sf]    # Pull from the new u,v data to update global U,V vectors
                v_c[y][x] += v[y//sf][x//sf]    # Pull from the new u,v data to update global U,V vectors

        # Use the latest u,v information to offset parts of the next order down's image to try to get finer correlations from there
        pass



img0 = cv2.imread('frame1.jpg',0)
img1 = cv2.imread('frame2.jpg',0)


# print(type(img))
computeFlow(img0, img1)
