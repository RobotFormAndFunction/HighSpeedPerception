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
#include "camera_pins.h"

bool sd_sign = false;              // Check sd status

const int IMAGE_WIDTH = 96; // set the camera properties to this size in the configure file
const int IMAGE_HEIGHT = 96; // set the camera properties to this size

// two instances of Two-dimensional array to hold the pixel values at consecutive time points
BLA::Matrix<IMAGE_HEIGHT, IMAGE_WIDTH,uint8_t> frames[2];
bool t = 0;   // timestep of latest image to go into buffer
#define THRESHOLD 255/150 //integer threshold for corner pixel - at least a 50 out of 255, but inverted for faster compute
// BLA::Matrix<IMAGE_HEIGHT, IMAGE_WIDTH,uint8_t> UVimage2D[2];  // 3D array tracking the U and V values for each pixel of incoming frames
// #define U_idx 0
// #define V_idx 1

#define MAX_PIX_CORR 128   // MAXIMUM_PIXEL_CORRESPONDENCES - the upper bound on how many corresponding points to try to match between two subsequent frames
// correspondence point arrays represent the 5 values describing the points in the correspondence array
uint8_t corr_X[MAX_PIX_CORR];
uint8_t corr_Y[MAX_PIX_CORR];
int8_t corr_U[MAX_PIX_CORR];
int8_t corr_V[MAX_PIX_CORR];
bool   corr_en[MAX_PIX_CORR];
uint8_t currentFrameCorners = 0;

#define corner_sl 5 // side length - how many pixels (one axis length) to include in the summation for corner detection
#define SSD_MATCH_WINDOW 16    // correspondence search window - how far to look in each direction for a correspondence match
#define SSD_PATCH_SIZE 7

// our call back to dump whatever we got in binary format, this is used with CoolTerm on my machine to capture an image
size_t jpgCallBack(void * arg, size_t index, const void* data, size_t len) {
  uint8_t* basePtr = (uint8_t*) data;
  for (size_t i = 0; i < len; i++) {
    Serial.write(basePtr[i]);
  }
  return 0;
}

// based on take_photos writeFile
void photo_save() {
    char filename[32];
    char fileU[32];
    char fileV[32];
    sprintf(filename, "/image%d.bytes", millis());
    sprintf(fileU, "/image%d.U", millis());
    sprintf(fileV, "/image%d.V", millis());
    timeLog("File write starting");
    Serial.printf("Beginning of writing image %s\n", filename);
    File file = SD.open(filename, FILE_WRITE);
    
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
      for (int col = 0; col < IMAGE_WIDTH; col++) {
        file.write(frames[t](row,col));
      }
    }

    timeLog("Image written");
    file.close();

    timeLog("Beginning of writing UV images");
    File file_U = SD.open(fileU, FILE_WRITE);
    File file_V = SD.open(fileV, FILE_WRITE);
    int corr_i = 0;
    
    for(int img_i = 0; img_i < IMAGE_HEIGHT * IMAGE_WIDTH; img_i++) {
        int y = img_i / IMAGE_WIDTH;
        int x = img_i % IMAGE_WIDTH;

        if(corr_i < MAX_PIX_CORR && corr_en[corr_i] && corr_X[corr_i] == x && corr_Y[corr_i] == y) {
            file_U.write(corr_U[corr_i]);
            file_V.write(corr_V[corr_i]);
            corr_i++;
        } else {
            file_U.write(0);
            file_V.write(0);
        }
    }

    timeLog("U-V Images written with correspondence count:");
    Serial.printf("              %d.\n", corr_i);
    file_U.close();
    file_V.close();

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
void cornerDetect(bool t) {
    timeLog("Beginning corner detection");
    // t: the starting frame
    // Computes the pixels in the image that have high dx and dy gradients, making them likely corners
    // uint8_t os = window_size/2; // offset - halved to shift iteration point toward the center of the patch
    corners.Fill(0);

    timeLog("Processing rows...");
    for (int row = 0; row < IMAGE_HEIGHT - corner_sl; row++) {
        for (int col = 0; col < IMAGE_WIDTH - corner_sl; col++) {
            Matrix<2,2> M;
            M.Fill(0);

            for (int y = row; y < row + corner_sl; y++) {
                for (int x = col; x < col + corner_sl; x++) {
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

    timeLog("Computed corner eigenvalues");

    // Generate the bit mask for pixels that are corners
    // find the maximum of the matrix
    uint8_t mx = corners(0,0); //maximum value of the matrix
    for (int row = 0; row < IMAGE_HEIGHT - corner_sl; row++) {
        for (int col = 0; col < IMAGE_WIDTH - corner_sl; col++) {
            if(mx < corners(row,col)) {mx = corners(row,col);}
        }
    }

    timeLog("Computed maximum corner value");

    // assume anything that is 20%+ of the maximum is a corner
    uint8_t ent = 0;
    for (int row = 0; row < IMAGE_HEIGHT - corner_sl; row++) {
        for (int col = 0; col < IMAGE_WIDTH - corner_sl; col++) {
            // point / mx -> [0,1]  
            // THRESHOLD -> 255/50 -> ~5, 
            // so p/mx > 0.2 should be non-zero
            int eval = (THRESHOLD * corners(row, col)) / mx;
            if (eval > 0) {
                // Serial.printf("Corner at Row,Col (%d,%d)\twith evaluated threshold: %d.\n", row, col, eval);
                // corners_map(row,col) = 1;
                corr_en[ent] = 1;
                corr_Y[ent] = row;
                corr_X[ent] = col;
                ent++;
            };
            // end the cycle
            if(ent == MAX_PIX_CORR) {
                row = IMAGE_HEIGHT;
                col = IMAGE_HEIGHT;
                break;
            }
        }
    }
    if(ent < MAX_PIX_CORR) {
        corr_en[ent]=0; // disable the next element in the array to truncate the list for this frame
    }
    timeLog("Completed mask generation of corners_map");
    Serial.printf("               with %d corners.\n", ent);
    currentFrameCorners = ent;

}

void timeLog(char *data) {
    Serial.printf("%d - %s.\n", millis(), data);
}

int ssd (bool t0, bool t1, int x0, int y0, int x1, int y1){
  /**
  t0 - time buffer index of first frame
  t1 - time buffer index of second frame
  x0,y0 - image coordinates of top left corner of patch t0
  x1,y1 - image coordinates of top left corner of patch t1
  */
  int diff = 0;
  for(int i = 0; i < SSD_MATCH_WINDOW; i++){
    for(int j = 0; j < SSD_MATCH_WINDOW; j++){
      int d = frames[t0](y0+i,x0+j) - frames[t1](y1+i,x1+j); // compute difference between each 2 corresponding pixels in the patch
      diff += d*d; // square that and add it to the sum to get SSD
    }
  }

  return diff;
}

void computeUV(){
    timeLog("Beginning UV Computation");
    corr_en[0] = 0;  // "clear" all corners by setting the first one as disabled // corners_map.Fill(0);    // clearCorners();
    cornerDetect(t);
    timeLog("Completed Corner Detection");
    // From the image data buffer, extract the U and V values at corners using SSD
    // iterator over sparse matrix elements based on: https://github.com/tomstewart89/BasicLinearAlgebra/blob/94f2bdf8c245cefc66842a7386940a045e7ef29f/test/test_linear_algebra.cpp#L159
    for(uint8_t i = 0; i < currentFrameCorners; i++) {
        uint8_t y = corr_Y[i];
        uint8_t x = corr_X[i];
        int best_u = 0x69;
        int best_v = 0x69;
        int best_diff = 2<<20; // huge number to ensure it is overwritten immediately
        // compute SSD over a couple of surrounding candidate (u,v) tuples
        for(int u = -SSD_MATCH_WINDOW; u<SSD_MATCH_WINDOW; u++) {
          for(int v = -SSD_MATCH_WINDOW; v<SSD_MATCH_WINDOW; v++) {
            int diff = ssd(!t, t, x, y, x+u, y+v);
            // Serial.printf("nssd: %d.  ob_ssd: %d", diff, best_diff);
            if (diff < best_diff){
              best_u = u;
              best_v = v;
              best_diff = diff;
            }
            if (diff == 0) break; // if you found a perfect match (which should rarely if ever happen) don't keep searching
          }
        }
        

        corr_U[i] = best_u;
        corr_V[i] = best_v;
        // Serial.printf("\nAdded corner: (%d,%d)\td:(%d,%d),\ti:%d\n", x,y, corr_U[i], corr_V[i], i);
    }

    // timeLog("Completed UV Segment");
    // Serial.printf("X,Y,U,V tuples: ");
    // for(uint8_t i = 0; i < currentFrameCorners; i++) {
        // Serial.printf("(%d,%d, %d,%d)  ",corr_X[i], corr_Y[i], corr_U[i], corr_V[i]);
    // }
}

int configureSD(){
    // 0 -> good
    // non-zero -> error
    // Initialize SD card
    if(!SD.begin(21)){
        Serial.println("Card Mount Failed");
        return 1;
    }
    uint8_t cardType = SD.cardType();

    // Determine if the type of SD card is available
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return 2;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    sd_sign = true; // sd initialization check passes
    return 0;
}

int initCamera() {
  // 0 -> success
  // non-zero -> error
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
  config.frame_size = FRAMESIZE_96X96; // _QVGA -> (320 x 240)   _96X96 -> (96 x 96)
  config.pixel_format = PIXFORMAT_GRAYSCALE; // changed to grayscale
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12; //this can be adjusted to create lower or higher quality images
  config.fb_count = 1;

  // camera initialize, will need to remove some of these things for the robot itself
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return err;
  }
  return 0;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  while(!Serial); // When the serial monitor is turned on, the program starts to execute

  if(initCamera()) return;
  if(configureSD()) return;

  // initialize all correspondence points to disabled
  for(int i=0; i < MAX_PIX_CORR; i++) {
      corr_en[i] = 0;
  }
}

// we have 80ms (12.5 fps) between camera frames to compute and act on the information

void loop(){
  // setting up a pointer to the frame buffer
  camera_fb_t * fb = NULL;
  
  Serial.printf("\n\n\t\tCYCLE START\t %d\n", millis());
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
    timeLog("Frame buffer verified");
    // Serial.printf("Camera buffer length: %d\n", fb->len);

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
    computeUV();
    
    // Uncomment for testing, comment out for high speed performance without SD card
    photo_save();

    t = (t+1)%2;
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
