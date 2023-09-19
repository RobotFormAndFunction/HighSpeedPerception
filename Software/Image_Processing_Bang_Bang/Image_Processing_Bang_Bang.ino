#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "img_converters.h" // see https://github.com/espressif/esp32-camera/blob/master/conversions/include/img_converters.h
#include <BasicLinearAlgebra.h>

// https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/References/References.ino
using namespace BLA;

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#define MAX_PIX_CORR 128   // MAXIMUM_PIXEL_CORRESPONDENCES - the upper bound on how many corresponding points to try to match between two subsequent frames
#include "camera_pins.h"

const int IMAGE_WIDTH = 320; // set the camera properties to this size in the configure file
const int IMAGE_HEIGHT = 240; // set the camera properties to this size

// two instances of Two-dimensional array to hold the pixel values at consecutive time points
BLA::Matrix<IMAGE_HEIGHT, IMAGE_WIDTH,uint8_t> frames[2];
bool t = 0;   // timestep of latest image to go into buffer
#define THRESHOLD 255/50 //integer threshold for corner pixel - at least a 50 out of 255, but inverted for faster compute
BLA::Matrix<IMAGE_HEIGHT, IMAGE_WIDTH,uint8_t> UVimage2D[2];  // 3D array tracking the U and V values for each pixel of incoming frames
#define U_idx 0
#define V_idx 1

// our call back to dump whatever we got in binary format, this is used with CoolTerm on my machine to capture an image
size_t jpgCallBack(void * arg, size_t index, const void* data, size_t len) {
  uint8_t* basePtr = (uint8_t*) data;
  for (size_t i = 0; i < len; i++) {
    Serial.write(basePtr[i]);
  }
  return 0;
}

uint8_t deltas[3]; // tracks x,y,z values returned by partialD, overwrtiting the same memory with each function call
uint8_t * partialD(uint8_t x, uint8_t y, bool t) {
  /**
  partial derivative over x,y,t between frames f1 and f2 at pixel coordinates (x,y)
  not divided by 4 as it should be to avoid loss of information with integer-type data storage
  */
  bool n = (1+t)%2;  
  deltas[0] /*x*/ = (frames[t](y,x+1) + frames[t](y+1,x+1) + frames[n](y,x+1) + frames[n](y+1,x+1)) - 
                    (frames[t](y,x)   + frames[t](y+1,x  ) + frames[n](y,x  ) + frames[n](y+1,x  ));
  deltas[1] /*y*/ = (frames[t](y+1,x) + frames[t](y+1,x+1) + frames[n](y+1,x) + frames[n](y+1,x+1)) - 
                    (frames[t](y,x)   + frames[t](y  ,x  ) + frames[n](y,  x) + frames[n](y+1,x  ));
  deltas[2] /*t*/ = (frames[n](y,x)   + frames[n](y+1,x  ) + frames[n](y,x+1) + frames[n](y+1,x+1)) - 
                    (frames[t](y,x)   + frames[t](y+1,x  ) + frames[t](y,x+1) + frames[t](y+1,x+1));

  return deltas;
}

uint8_t eigenvals[2]; // tracks eigenvalues of the matrix
uint8_t * get_eigenvals(Matrix<2,2> M) { 
  /** 
  Loads the eigenvalues for a 2x2 symmetric matrix into eigenvals[2].
  This will provide the wrong answer for all other matrices
  */
    uint8_t a = M(0,0);
    uint8_t b = M(0,1);
    uint8_t c = M(1,1);
    uint8_t base = a+c;
    uint8_t radical = (a-c)*(a-c) + (4*b*b);
    eigenvals[0] = (base + sqrt(radical)) / 2;
    eigenvals[1] = (base - sqrt(radical)) / 2;
    return eigenvals;
}

SparseMatrix<IMAGE_HEIGHT, IMAGE_WIDTH, uint8_t, MAX_PIX_CORR> corners_map;
Matrix      <IMAGE_HEIGHT, IMAGE_WIDTH, uint8_t> corners;
uint8_t * cornerDetect(bool t) {
    Serial.println("Beginning corner detection");
    // t: the starting frame
    // Computes the pixels in the image that have high dx and dy gradients, making them likely corners
    #define sl 3 // side length - how many pixels (one axis length) to include in the summation for corner detection
    // uint8_t os = window_size/2; // offset - halved to shift iteration point toward the center of the patch
    corners.Fill(0);

    Serial.println("Created corners matrix");
    Serial.print("Processing row...");
    for (int row = 0; row < IMAGE_HEIGHT - sl; row++) {
        Serial.print(row);
        Serial.print(" ");
        if(row%32 == 0) {Serial.println("");}
        for (int col = 0; col < IMAGE_WIDTH - sl; col++) {
            Matrix<2,2> M;
            M.Fill(0);
            // RefMatrix<BLA::Matrix<IMAGE_HEIGHT, IMAGE_WIDTH> sl,sl> patch0(video[t].Submatrix<sl,sl>(row,col)); 

            for (int y = row; y < row + sl; y++) {
                for (int x = col; x < col + sl; x++) {
                    uint8_t *grad = partialD(x, y, t);  // I_x, I_y, I_t
                    M(0,0) += grad[0] * grad[0];              // I_x ^2
                    M(0,1) += grad[0] * grad[1];              // I_x * I_y
                    M(1,0) += grad[1] * grad[0];              // I_x * I_y
                    M(1,1) += grad[1] * grad[1];              // I_y ^2
                }
            }

            // determine eigenvalues from matrix M
            get_eigenvals(M);
            uint8_t eg = eigenvals[0]; // λ_1 - the greater eigenvalue
            uint8_t el = eigenvals[1]; // λ_2 - the smaller eigenvalue

            // use the eigenvalues to determine partial derivatives
            corners(row, col) = eg + el + eg*el;  //  eg*el = likely a corner.  If we don't get any corners, the edges will have to do.
        }
    }

    Serial.println("\nComputed corner eigenvalues");

    // Generate the bit mask for pixels that are corners
    // find the maximum of the matrix
    uint8_t mx = corners(0,0); //maximum value of the matrix
    for (int row = 0; row < IMAGE_HEIGHT - sl; row++) {
        for (int col = 0; col < IMAGE_WIDTH - sl; col++) {
            if(mx < corners(row,col)) {mx = corners(row,col);}
        }
    }

    Serial.println("Computed maximum corner value");

    // assume anything that is 20%+ of the maximum is a corner
    uint8_t entities = 0;
    for (int row = 0; row < IMAGE_HEIGHT - sl; row++) {
        for (int col = 0; col < IMAGE_WIDTH - sl; col++) {
            // point / mx -> [0,1]  
            // THRESHOLD -> 255/50 -> ~5, 
            // so p/mx > 0.2 should be non-zero
            if (((THRESHOLD * corners(row,col)) / mx) > 0) {
                corners_map(row,col) = 1;
                entities++;
            };
        }
    }
    Serial.printf("Completed mask generation of corners_map with %d corners.", entities);
}
void clearCorners(){
    corners_map.Fill(0);
}


void computeUV(){
    Serial.println("Beginning UV Computation");
    corners_map.Fill(0);    // clearCorners();
    cornerDetect(t);
    // From the image data buffer, extract the U and V values at corners using SSD
    // iterator over sparse matrix elements based on: https://github.com/tomstewart89/BasicLinearAlgebra/blob/94f2bdf8c245cefc66842a7386940a045e7ef29f/test/test_linear_algebra.cpp#L159
    Serial << "This frame has " << corners_map.Size << " corner pixels.\n";
    for(uint8_t i = 0; i < corners_map.Size; i++) {
        const auto &corner = corners_map.table[i];
        uint8_t r = corner.row;
        uint8_t c = corner.col;
        // compute SSD over a couple of surrounding candidate (u,v) tuples
    }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  while(!Serial); // When the serial monitor is turned on, the program starts to execute

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
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

}

// we have 80ms (12.5 fps) between camera frames to compute and act on the information

void loop(){
  // setting up a pointer to the frame buffer
  camera_fb_t * fb = NULL;
  
  // Take Picture with camera and put in buffer
  fb = esp_camera_fb_get();

  if (!fb) {
    return;
    Serial.println("Camera capture failed");
  }

  int leftSum = 0;
  int centerSum = 0;
  int rightSum = 0;


  if (fb) {
    Serial.print("t:");
    Serial.println(millis());
    Serial.print("Camera buffer length:");
    Serial.println(fb->len);

    // Transfer pixel data from the image buffer to the 2D array
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
      for (int col = 0; col < IMAGE_WIDTH; col++) {
        int index = (row * IMAGE_WIDTH) + col; // Calculate the index in the 1D buffer
        frames[t](row, col) = fb->buf[index];    // Copy the pixel value to the 2D array and put a 1 if above threshold, otherwise 0
      }
    }
    // Release the image buffer
    esp_camera_fb_return(fb);

    // compute the consequences
    t = (t+1)%2;
    computeUV();
  }



  if(centerSum >= leftSum && centerSum >= rightSum){
    Serial.println("both motors on");
    }
  if(centerSum <= leftSum && leftSum >= rightSum){
    Serial.println("left motor on");
    }
  if(rightSum >= leftSum && leftSum <= rightSum){
    Serial.println("right motor on");
    }
  else {Serial.println("both motors on");}

}
