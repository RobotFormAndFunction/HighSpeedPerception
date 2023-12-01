#include "MatrixMath.h"
#include "esp_camera.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"


#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM


//macro used to fill up an array with zeros
#define FILLARRAY(a, n) \
  do { \
    for (int _i = 0; _i < sizeof(a) / sizeof(a[0]); _i++) { \
      for (int _j = 0; _j < sizeof(a[0]) / sizeof(a[0][0]); _j++) { \
        a[_i][_j] = n; \
      } \
    } \
  } while (0)

#include "camera_pins.h"

const int IMAGE_WIDTH = 320; // set the camera properties to this size in the configure file
const int IMAGE_HEIGHT = 240; // set the camera properties to this size

// Two-dimensional array to hold the pixel values
uint8_t image2D[IMAGE_HEIGHT][IMAGE_WIDTH]; //two array to make a binary image file
uint8_t image2D_old[IMAGE_HEIGHT][IMAGE_WIDTH]; //two array to make a binary image file

// Declare 2D arrays for the x and y gradients
const int GRADIENT_WINDOW_HEIGHT = IMAGE_HEIGHT / 20;  // 5% of image height
const int GRADIENT_WINDOW_WIDTH = IMAGE_WIDTH;

// Declare 2D arrays for the x and y gradients
uint8_t Ix1[GRADIENT_WINDOW_HEIGHT][GRADIENT_WINDOW_WIDTH];
uint8_t Iy1[GRADIENT_WINDOW_HEIGHT][GRADIENT_WINDOW_WIDTH];
uint8_t It[GRADIENT_WINDOW_HEIGHT][GRADIENT_WINDOW_WIDTH];


const int windowSize = 3; //used for Lucas-Kanade optical flow algorithm
const int opticalFlowWindow = IMAGE_HEIGHT/20; //setting this up to only loo at at a 5% window in the middle

// Preallocate the optical flow vectors (2D arrays for each channel)
double flow_x[GRADIENT_WINDOW_HEIGHT][GRADIENT_WINDOW_WIDTH];
double flow_y[GRADIENT_WINDOW_HEIGHT][GRADIENT_WINDOW_WIDTH];

const int mat_row_size=(windowSize*2+1)*(windowSize*2+1);

double lambda = 1e-6;

mtx_type A[mat_row_size][2];
mtx_type At[2][mat_row_size];
mtx_type b[mat_row_size][1];
mtx_type AtA[2][2];
mtx_type PI_A[2][2];
mtx_type v[2][1];




void setup() {
  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  camera_config_t config; //setting up configuration 
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA; //changed to qvga for (320 x 240)
  config.pixel_format = PIXFORMAT_GRAYSCALE; // changed to grayscale
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12; //this can be adjusted to create lower or higher quality images
  config.fb_count = 1;

  // camera initialize, will need to remove some of these things for the robot itself
  esp_err_t err = esp_camera_init(&config);

// set up two arrays for two images with 0s
  FILLARRAY(image2D, 0);
  FILLARRAY(image2D_old, 0);

}

void loop(){
  // setting up a pointer to the frame buffer
  camera_fb_t * fb = NULL;
  
  // Take Picture with camera and put in buffer
  fb = esp_camera_fb_get();

  // If something exists in the frame buffer, then put it in the two-d array
  if (fb) {
    // Transfer pixel data from the image buffer to the 2D array
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
      for (int col = 0; col < IMAGE_WIDTH; col++) {
        int index = (row * IMAGE_WIDTH) + col; // Calculate the index in the 1D buffer
        image2D[row][col] = fb->buf[index];   // Copy the pixel value to the 2D array
      }
    }

  // Release the image buffer
  esp_camera_fb_return(fb);
  }

  // now to do optical flow between two images
  // First compute the spatial gradient of the old image
  computeGradient_oldimage();
  computeTimeGradient();
  computeOpticalFlow();

  // Store the new image as the old image
  for (int row = 0; row < IMAGE_HEIGHT; row++) {
    for (int col = 0; col < IMAGE_WIDTH; col++) {
      image2D_old[row][col] = image2D[row][col];
    }
  }

  //implement control based on flow_x
for(int i = 0; i < IMAGE_WIDTH; i++)
{
  Serial.print(flow_x[i][1]);
}
Serial.println(" ");

  
}



void computeGradient_oldimage() {
  // Compute gradients in the x and y directions within the optical flow window
  int y_start = IMAGE_HEIGHT/2 - GRADIENT_WINDOW_HEIGHT/2; // Start from 45% of the image height
  int y_end = IMAGE_HEIGHT/2 + GRADIENT_WINDOW_HEIGHT/2;   // End at 55% of the image height
  
  for (int y = y_start; y < y_end; y++) {
    for (int x = 0; x < IMAGE_WIDTH; x++) {
      // Calculate the x-gradient
      if (x == 0) {
        Ix1[y][x] = image2D_old[y][x + 1] - image2D_old[y][x]; // Forward difference
      } else if (x == IMAGE_WIDTH - 1) {
        Ix1[y][x] = image2D_old[y][x] - image2D_old[y][x - 1]; // Backward difference
      } else {
        Ix1[y][x] = (image2D_old[y][x + 1] - image2D_old[y][x - 1]) / 2; // Central difference
      }

      // Calculate the y-gradient
      if (y == y_start) {
        Iy1[y][x] = image2D_old[y + 1][x] - image2D_old[y][x]; // Forward difference
      } else if (y == y_end - 1) {
        Iy1[y][x] = image2D_old[y][x] - image2D_old[y - 1][x]; // Backward difference
      } else {
        Iy1[y][x] = (image2D_old[y + 1][x] - image2D_old[y - 1][x]) / 2; // Central difference
      }
    }
  }
}


void computeTimeGradient() {
  // Compute temporal gradient
  for (int y = IMAGE_HEIGHT/2 - GRADIENT_WINDOW_HEIGHT/2; y < IMAGE_HEIGHT/2 + GRADIENT_WINDOW_HEIGHT/2; y++) {
    for (int x = 0; x < IMAGE_WIDTH; x++) {
      int index = (y * IMAGE_WIDTH) + x;
      // Calculate the temporal gradient (difference between current and old frames)
      It[y][x] = image2D[y][x] - image2D_old[y][x];
    }
  }
}

//following Lucas-Kanade optical flow algorithm
void computeOpticalFlow(){
    int A_row = 0; //initial row counter
    // Compute the optical flow for each pixel
    for (int y = windowSize + 1; y < IMAGE_HEIGHT - windowSize; y++) {
        for (int x = windowSize + 1; x < IMAGE_WIDTH - windowSize; x++) {
            // Extract the window around the current pixel in both images
            int window_start_y = y - windowSize;
            int window_end_y = y + windowSize;
            int window_start_x = x - windowSize;
            int window_end_x = x + windowSize;

            // Fill the matrix A with gradient values and vector b with temporal gradient values
            for (int window_y = window_start_y; window_y <= window_end_y; window_y++) {
                for (int window_x = window_start_x; window_x <= window_end_x; window_x++) {
                    A[A_row][0] = Ix1[window_y][window_x];
                    A[A_row][1] = Iy1[window_y][window_x];
                    b[A_row][0] = -1*It[window_y][window_x];                 
                    A_row++;
                }
            }
            // Compute the Moore-Penrose pseudo-inverse of A with regularization this is matlab script that was translated A_pseudo_inverse = inv((A' * A + lambda * eye(size(A, 2)))) * A';
            Matrix.Transpose((mtx_type*)A, mat_row_size, 2, (mtx_type*)At);
            Matrix.Multiply((mtx_type*)At, (mtx_type*)A, mat_row_size, 2, mat_row_size, (mtx_type*)AtA);
        
            AtA[0][0] += lambda; //add regularization to avoid singularities
            AtA[1][1] += lambda;
            
            Matrix.Invert((mtx_type*)AtA, 2);
            Matrix.Multiply((mtx_type*)AtA, (mtx_type*)At, 2, 2, mat_row_size, (mtx_type*)PI_A);
            Matrix.Multiply((mtx_type*)PI_A, (mtx_type*)b, 2, mat_row_size, 2, (mtx_type*)v);

            // Store the optical flow vector components in the preallocated arrays
            flow_x[y][x] = v[0][0];
            flow_y[y][x] = v[1][0];
        }
    }
}
