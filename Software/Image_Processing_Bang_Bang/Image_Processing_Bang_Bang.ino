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

// Two-dimensional array to hold the pixel values
uint8_t image2D[IMAGE_HEIGHT][IMAGE_WIDTH]; //two array to make a binary image file
#define THRESHOLD 150 //decimal threshold for white pixel

// our call back to dump whatever we got in binary format, this is used with CoolTerm on my machine to capture an image
size_t jpgCallBack(void * arg, size_t index, const void* data, size_t len)
{
  uint8_t* basePtr = (uint8_t*) data;
  for (size_t i = 0; i < len; i++) {
    Serial.write(basePtr[i]);
  }
  return 0;
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

void loop(){
  // setting up a pointer to the frame buffer
  camera_fb_t * fb = NULL;
  
  // Take Picture with camera and put in buffer
  fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  int leftSum = 0;
  int centerSum = 0;
  int rightSum = 0;


  if (fb) {
    // Transfer pixel data from the image buffer to the 2D array
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
      for (int col = 0; col < IMAGE_WIDTH; col++) {
        int index = (row * IMAGE_WIDTH) + col; // Calculate the index in the 1D buffer
        image2D[row][col] = (fb->buf[index] > THRESHOLD) ? 1 : 0;;    // Copy the pixel value to the 2D array and put a 1 if above threshold, otherwise 0

        //dividing image up into thirds, left, center, and right
        if (col < IMAGE_WIDTH/3){
          leftSum += image2D[row][col];
        }
        if (col > IMAGE_WIDTH/3 && col < 2*IMAGE_WIDTH/3){
          centerSum += image2D[row][col];
        }
        if (col > 2*IMAGE_WIDTH/3 && col < IMAGE_WIDTH){
          rightSum += image2D[row][col];
        }
      }
    }

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
