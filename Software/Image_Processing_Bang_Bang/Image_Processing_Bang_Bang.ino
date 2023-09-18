#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "img_converters.h" // see https://github.com/espressif/esp32-camera/blob/master/conversions/include/img_converters.h

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

const int IMAGE_WIDTH = 320; // set the camera properties to this size in the configure file
const int IMAGE_HEIGHT = 240; // set the camera properties to this size

// two instances of Two-dimensional array to hold the pixel values at consecutive time points
uint8_t image2D[2][IMAGE_HEIGHT][IMAGE_WIDTH]; //3 array to make a binary image file
uint8_t t = 0;   // timestep of latest image to go into buffer
#define THRESHOLD 150 //decimal threshold for white pixel
uint8_t UVimage2D[IMAGE_HEIGHT][IMAGE_WIDTH][2]; // 3D array tracking the U and V values for each pixel of incoming frames


// our call back to dump whatever we got in binary format, this is used with CoolTerm on my machine to capture an image
size_t jpgCallBack(void * arg, size_t index, const void* data, size_t len) {
  uint8_t* basePtr = (uint8_t*) data;
  for (size_t i = 0; i < len; i++) {
    Serial.write(basePtr[i]);
  }
  return 0;
}

float eigenvals[2];
float * calculateEigenvals2x2(M00, M01, M10, M11){
  
}

uint8_t deltas[3]; // tracks x,y,z values returned by partialD, overwrtiting the same memory with each function call
uint8_t * partialD(uint8_t** f1,uint8_t** f2, uint8_t x, uint8_t y) {
  /**
  partial derivative over x,y,t between frames f1 and f2 at pixel coordinates (x,y)
  */
  deltas[0] /*x*/ = (f1[y][x+1] + f1[y+1][x+1] + f2[y][x+1] + f2[y+1][x+1]) - 
                    (f1[y][x  ] + f1[y+1][x  ] + f2[y][x  ] + f2[y+1][x  ]);
  deltas[1] /*y*/ = (f1[y+1][x] + f1[y+1][x+1] + f2[y+1][x] + f2[y+1][x+1]) - 
                    (f1[y  ][x] + f1[y  ][x  ] + f2[y  ][x] + f2[y+1][x  ]);
  deltas[2] /*t*/ = (f2[y ][x ] + f2[y+1][x  ] + f2[y][x+1] + f2[y+1][x+1]) - 
                    (f1[y ][x ] + f1[y+1][x  ] + f1[y][x+1] + f1[y+1][x+1]);

  return deltas;
}

uint8_t * cornerDetect(uint8_t** f1,uint8_t** f2) {
    // Computes the pixels in the image that have high dx and dy gradients, making them likely corners
    uint8_t window_size = 3;   // how many pixels (one axis length) to include in the summation for corner detection
    // uint8_t os = window_size/2; // offset - halved to shift iteration point toward the center of the patch

    for (int row = 0; row < IMAGE_HEIGHT - window_size; row++) {
        for (int col = 0; col < IMAGE_WIDTH - window_size; col++) {
            uint8_t M0, M1, M2; // track the sums: M0 -> Ix^2, M1 -> IxIy, M2 -> Iy^2
            M0 = M1 = M2 = 0;
            for (int y = row; y < row + window_size; y++) {
              for (int x = col; x < col + window_size; x++) {
                uint8_t *grad = partialD(f1, f2, x, y);  // I_x, I_y, I_t
                M0 += grad[0] * grad[0];  // I_x ^2
                M1 += grad[0] * grad[1];  // I_x * I_y
                M2 += grad[1] * grad[1];  // I_y ^2
              }
            }

            // determine eigenvalues from matrix M

            // use the eigenvalues to determine partial derivatives
            // todo
        }
    }

    // Generate the bit mask for pixels that are corners
    // todo
}

void computeUV(){
  // From the image2D data buffer, extract the U and V values at corners using SSD
  // todo
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
        image2D[t][row][col] = fb->buf[index];    // Copy the pixel value to the 2D array and put a 1 if above threshold, otherwise 0
        computeUV();
      }
    }
    t = (t+1)%2;
    // Release the image buffer
    esp_camera_fb_return(fb);
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
