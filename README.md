# People-counter-with-ESP32-CAM
/*
 * Cor_Cam.ino
 *
 *  Created on: Nov 29, 2020
 *      modified on: Dec 29, 2020
 *      Authors: Anna Werner & Amina Chida
 *      version: v1.4 
 *  
 *  description:
 *  This code aims to count the number of people inside a builing using the ESP-Cam32 
 *  
 *  
 *  notifications:
 *  
 */
 
/* **************************************************************************** */

#define CAMERA_MODEL_AI_THINKER // define the type of the ESP camera 
// include the libraries 
#include "esp_camera.h"
#include "camera_pins.h"

#define FRAME_SIZE FRAMESIZE_QVGA // define the type of the frame size
#define WIDTH 320 //in pixels 
#define HEIGHT 240 //in pixels 
#define BLOCK_SIZE 10
#define W (WIDTH / BLOCK_SIZE) // in blocks 
#define H (HEIGHT / BLOCK_SIZE) //in blocks 
// define the scaling factor  which means the percent threshold above which we say that the block is actually changed
#define BLOCK_DIFF_THRESHOLD 0.2
//define the number of blocks above which an image is changed 
#define IMAGE_DIFF_THRESHOLD 0.2
#define DEBUG 1

/*initialize the variables needed in the program
*/
uint16_t prev_frame[H][W] = { 0 }; // the previous frame 
uint16_t current_frame[H][W] = { 0 };
int list[2]={0,0} ; // this list allows us to select only one direction when sereval directions are detected for just one person entering or getting out of the building 
int counter = -1 ; 

/* define the used funtions 
 */
bool setup_camera(framesize_t);
bool capture_still();
int motion_detect();
void update_frame();
void print_frame(uint16_t frame[H][W]);
bool direction_detection ();
int freq(uint16_t frame[H][W],uint16_t a);


/**
 *
 */
void setup() {
    Serial.begin(115200);
    Serial.println(setup_camera(FRAME_SIZE) ? "OK" : "ERR INIT");
    pinMode(12, OUTPUT);  // initialize the GPIO12 pin as an output
    pinMode(4, OUTPUT);  // initialize the GPIO4 pin as an output
}

/**
 This is the main program where all the functions are called 
 */
 
void loop() {
    /* verify that there is no error in detecting the image from the camera */
    if (!capture_still()) {
        Serial.println("Failed capture");      
        return;
    }
// the following allows to count the number of people according to the direction of their movement 
   switch (motion_detect()){
    case 0 : 
      Serial.println("no motion");
      list[1]=0;
      digitalWrite(12, LOW);
      digitalWrite(4, LOW);
      break;  
    case 1 : 
      Serial.println("get in "); // when a person gets in that means that he is coming from the left side( the motion is detected from the left side of the photo) 
      list[1]= 1;
      digitalWrite(12, HIGH);
      digitalWrite(4, LOW);
      break; 
    case -1 : 
      Serial.println("get out"); // when a person gets out that means that he is coming from the right side( the motion is detected from the right side of the photo) 
      list[1]= -1 ;
      digitalWrite(12, LOW);
      digitalWrite(4, HIGH);
      break;       
   }
 // if two directions or more are detected simultaneously (within milliseconds) only the first one will be selected     
    if ((list[0]==0) && (list[1]==1)){
       counter= counter+1 ; // if the person is getting in the building +1 is added 
    }else if ((list[0]==0) && (list[1]==-1)){
        counter=counter-1 ; //if the person is getting out the building 1 is substracted
       }
       
    Serial.print(counter) ; 
    list[0]= list[1]; //the current value will be the preview value
    Serial.println("=================");
    update_frame();
      
}


/**
 * Camera setup
 */
bool setup_camera(framesize_t frameSize) {
    camera_config_t config;

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
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = frameSize;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    bool ok = esp_camera_init(&config) == ESP_OK;

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, frameSize);

    return ok;
}

/**
 * Capture image and do down-sampling, this means that the image is divided into blocks instead of pixels in order to reduce noise 
 * A block is a 10x10 pixels
 */
bool capture_still() {
    camera_fb_t *frame_buffer = esp_camera_fb_get();

    if (!frame_buffer)
        return false;

    // set all 0s in current frame
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] = 0;


    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];

        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if DEBUG
    //Serial.println("Current frame:");
    //print_frame(current_frame);
    //Serial.println("Preview frame:");
    //print_frame(prev_frame);
    //Serial.println("---------------");
#endif

    return true;
}
/* this function is used to determin the direction of the movement by dividing the direction matrix 
 * calculated in motion_detect() function into one right side matrix and one left side matrix,
 * the matrix with the higher frequency in "99" shows the direction from which the movement is coming 
 */
bool direction_detection(uint16_t frame[H][W]){
  // initialize the two direction matrices 
  uint16_t direc_left[H][W] = { 0 };
  uint16_t direc_right[H][W] = { 0 };
  
  for (int y = 0; y < H ; y++) {
      for (int x = 0; x < W; x++){
        if(x < W/2 ) {                   
          direc_left[y][x] = frame[y][x];
          direc_right[y][x] = { 0 };
        } 
        else {
           direc_right[y][x] = frame[y][x];
           direc_left[y][x] = { 0 };
        }
      } 
      }
      //print_frame(direc_left);
      //Serial.println("---------------");
      //print_frame(direc_right);
      Serial.println("....................");
  if (freq(direc_right,{99}) > freq(direc_left,{99})) {
    //Serial.println("Rechts Richtung");
    return false;
    } else if (freq(direc_right,{99}) < freq(direc_left,{99})) { 
      //Serial.println("Links Richtung");
      return true; 
    }
    //else {
      //Serial.println("error keine Richtung detektiert ");
    //} 
   
}


/**
 * Compute the number of different blocks
 * If there are enough, then motion happened
 */

int motion_detect(){
    uint16_t changes = {0}; //initialize the number of changed blocks 
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE); // set the number of blocks in the picture
    uint16_t direc[H][W] = { 0 };
    // set the direction matrix to all 0s
    for (int y = 0; y < H; y++)
       { for (int x = 0; x < W; x++)
       {direc[y][x] = {0};}}
    // compare the blocks of the current frame with the blocks of the previous frame and calculate the delta factor 
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float current = current_frame[y][x]; 
            float prev = prev_frame[y][x];
            float delta = abs(current - prev) / prev;
    
            if (delta >= BLOCK_DIFF_THRESHOLD) { 
#if DEBUG
                  
                //Serial.print("diff\t");
                //Serial.print(y);
                //Serial.print('\t');
                //Serial.println(x);
                //delay(100);
                
                changes += 1;  //if the block has changed considerably then add one to the number of changed blocks 
                direc[y][x] = {99} ; // write 99 in the direction matrix refering to the changed block 
 #endif               
            }
    }
    }
     //Serial.print("Changed ");
     //Serial.print(changes);

     if ((1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD){
      if  (direction_detection(direc)) {
          return 1;  // 1 means there is motion from the leftside ==> someone is getting in
      } else {
        return -1 ; //-1 means there is motion from the rightside ==> someone is getting out  
      }  
    } else {
      return 0; // there is no motion 
    }
}




/**
 * Copy current frame to previous
 */
void update_frame() { 
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            prev_frame[y][x] = current_frame[y][x];
        }
    }
}
/**
 * calculate the frequency of a number in a HxW matrix 
 */
int freq(uint16_t matrix[H][W],uint16_t a) {
   int freq = 0 ; 
   for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            if (matrix[y][x] == a) {
              freq = freq +1 ;
            }
        }
    }
    return freq ;

}

/**
 * For serial debugging ürint the frame 
 */
void print_frame(uint16_t frame[H][W]) {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }

        Serial.println();
    }
}
