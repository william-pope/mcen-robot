#include <Servo.h>
#include "arduinoFFT.h"
#include <Pixy2.h>
#include <NewPing.h>
#include <SparkFun_TB6612.h>

// motors
// m1
#define LTS 2 //Left top motor
#define LT1 4 //B01
#define LT2 3 //B02

// m2
#define RTS 7 //Right top motor
#define RT1 5 //A01
#define RT2 6 //A02

// m3
#define LBS 13 //Left bottom motor
#define LB1 11 //B11
#define LB2 12 //B12

// m4
#define RBS 8  //Right bottom motor
#define RB1 10 //A11
#define RB2 9  //A12

#define STNDBY 22 //Connected to VCC

/* m1 - left front
 * m2 - right front
 * m3 - left back
 * m4 - right back
 */

const int direction_m1 = 1;
const int direction_m2 = -1;
const int direction_m3 = 1;
const int direction_m4 = -1;

Motor m1 = Motor(LT1, LT2, LTS, direction_m1, STNDBY);
Motor m2 = Motor(RT1, RT2, RTS, direction_m2, STNDBY);
Motor m3 = Motor(LB1, LB2, LBS, direction_m3, STNDBY);
Motor m4 = Motor(RB1, RB2, RBS, direction_m4, STNDBY);

// servo
Servo servo;
int servo_pos;

// ultrasonic
#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 30 // Maximum distance (in cm) to ping.

float factor = sqrt(1.00 + 25.00 / 273.15) / 60.368; // Speed of sound calculation based on temperature.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(31, 33, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(31, 32, MAX_DISTANCE)
};

// pixy
Pixy2 pixy;

// microphone
#define MIC_PIN 0

#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 5000 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
 
arduinoFFT FFT = arduinoFFT();

bool mic_enable = true;
 
unsigned long microSeconds;
int startFreq = 2000;

double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values
 
unsigned int samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds 

// initialize buffers
#define ACT_REQ_LENGTH 9
#define OBS_RPL_LENGTH 25

byte act_req_buffer[ACT_REQ_LENGTH];
byte obs_rpl_buffer[OBS_RPL_LENGTH];

byte req_id[1];


void setup() {
  // serial
  Serial.begin(115200);
 
  // servo
  servo.attach(45);
  servo.write(50);
  delay(3000);

  // pixy
  pixy.init();
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytes(req_id, 1); 

    if (req_id[0] == 0xaa) {
      Serial.readBytes(act_req_buffer, ACT_REQ_LENGTH);
      act_request(act_req_buffer);  
    } 
    else if (req_id[0] == 0xff) {
      obs_request_offense();
    }
    else if (req_id[0] == 0xdd) {
      obs_request_defense();
    }
  }

  // TO-DO: add timer to shut down motors if no messages received from Pi
}

void act_request(byte act_req_buffer[ACT_REQ_LENGTH]) {    
  int p_m1 = (act_req_buffer[1] << 8) + act_req_buffer[0];
  int p_m2 = (act_req_buffer[3] << 8) + act_req_buffer[2];
  int p_m3 = (act_req_buffer[5] << 8) + act_req_buffer[4];
  int p_m4 = (act_req_buffer[7] << 8) + act_req_buffer[6];
  bool shoot = act_req_buffer[8];

  setMotors(p_m1, p_m2, p_m3, p_m4);
  setServo(shoot);

  Serial.write(0x1a);
}

// ISSUE: returns all zeros, make sure bit-shifting is correct
//  - functions return properly, bit shifting works properly
//  - issue is with internal sensor calls

void obs_request_offense() {
  // ultra sonic sensor 1
  int d_r0 = getRange(0);
  obs_rpl_buffer[0] = d_r0;
  obs_rpl_buffer[1] = d_r0 >> 8;

  // ultra sonic sensor 2
  int d_r1 = getRange(1);
  obs_rpl_buffer[2] = d_r1;
  obs_rpl_buffer[3] = d_r1 >> 8;

  int* coords_ptr;
  
  // pixy - ball
  coords_ptr = getPixy(1);
  obs_rpl_buffer[4] = coords_ptr[0];
  obs_rpl_buffer[5] = coords_ptr[0] >> 8;
  obs_rpl_buffer[6] = coords_ptr[1];
  obs_rpl_buffer[7] = coords_ptr[1] >> 8;

  // pixy - left post
  coords_ptr = getPixy(2);
  obs_rpl_buffer[8] = coords_ptr[0];
  obs_rpl_buffer[9] = coords_ptr[0] >> 8;
  obs_rpl_buffer[10] = coords_ptr[1];
  obs_rpl_buffer[11] = coords_ptr[1] >> 8;

  // pixy - right post
  coords_ptr = getPixy(3);
  obs_rpl_buffer[12] = coords_ptr[0];
  obs_rpl_buffer[13] = coords_ptr[0] >> 8;
  obs_rpl_buffer[14] = coords_ptr[1];
  obs_rpl_buffer[15] = coords_ptr[1] >> 8;

  // pixy - robot yellow
  coords_ptr = getPixy(4);
  obs_rpl_buffer[16] = coords_ptr[0];
  obs_rpl_buffer[17] = coords_ptr[0] >> 8;
  obs_rpl_buffer[18] = coords_ptr[1];
  obs_rpl_buffer[19] = coords_ptr[1] >> 8;

  // pixy - robot blue
  coords_ptr = getPixy(5);
  obs_rpl_buffer[20] = coords_ptr[0];
  obs_rpl_buffer[21] = coords_ptr[0] >> 8;
  obs_rpl_buffer[22] = coords_ptr[1];
  obs_rpl_buffer[23] = coords_ptr[1] >> 8;


  // start tone
  bool s_t;
  if (mic_enable == true) {
    s_t = getMic();
  } 
  else if (mic_enable == false) {
    s_t = 0x01;
  }
  
  obs_rpl_buffer[24] = s_t;
  
  Serial.write(obs_rpl_buffer, OBS_RPL_LENGTH);
}

void obs_request_defense() {
  // set range measurements to zero
  for (int i=0; i<4; i++) {
    obs_rpl_buffer[i] = 0x00;
  }

  int* coords_ptr;
  
  // pixy - ball
  coords_ptr = getPixy(1);
  obs_rpl_buffer[4] = coords_ptr[0];
  obs_rpl_buffer[5] = coords_ptr[0] >> 8;
  obs_rpl_buffer[6] = coords_ptr[1];
  obs_rpl_buffer[7] = coords_ptr[1] >> 8;

  // set other pixy objects to zero
  for (int i=8; i<24; i++) {
    obs_rpl_buffer[i] = 0x00;
  }

  // start tone
  bool s_t;
  if (mic_enable == true) {
    s_t = getMic();
  } 
  else if (mic_enable == false) {
    s_t = 0x01;
  }
  
  obs_rpl_buffer[24] = s_t;
  
  Serial.write(obs_rpl_buffer, OBS_RPL_LENGTH);
}

void setMotors(int p_m1, int p_m2, int p_m3, int p_m4) {
  m1.drive(p_m1);
  m2.drive(p_m2);
  m3.drive(p_m3);
  m4.drive(p_m4);
}

void setServo(bool shoot) {
  if (shoot == 1) {
    servo.write(110);
  } 
  else {
    servo.write(-10);
  }
}

int getRange(uint8_t ID) {
  int distance = int(sonar[ID].ping_median(5) * factor * 10); // [mm]
  delay(50);  
  
  return distance;
}

int* getPixy(int sig) {  

  int m; //int result value 
  int block_sig; //signature of block

  static int coords[2];

  pixy.ccc.getBlocks();

  int m_x = 0;
  int m_y = 0;
  
  if (pixy.ccc.numBlocks) {
    for (int i=0; i<pixy.ccc.numBlocks; i++) {
      block_sig = pixy.ccc.blocks[i].m_signature;
      
      if (block_sig == sig) {
          m_x = pixy.ccc.blocks[i].m_x;
          m_y = pixy.ccc.blocks[i].m_y;
      }
    }

    coords[0] = m_x;    // (?): should this be inside >>if (block_sig == sig) ?
    coords[1] = m_y;
  }
  
  return coords;
}

bool getMic() {
  bool start_tone;

  for (int q=0; q<10; q++) {
    for (int i=0; i<SAMPLES; i++) {
      microSeconds = micros();    // returns the number of microseconds since the Arduino board began running the current script. 
   
      vReal[i] = analogRead(MIC_PIN); // reads the value from analog pin 0 (A0), quantize it and save it as a real term.
      vImag[i] = 0; // makes imaginary term 0 always
  
      /*remaining wait time between samples if necessary*/
      while(micros() < (microSeconds + samplingPeriod)) {
        //do nothing
      }
    }
  
    /*Perform FFT on samples*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  
    /*Find peak frequency and print peak*/
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  
    if (peak > startFreq) {
      start_tone = 1;
      mic_enable = false;
      return start_tone;
    }
    else {
      start_tone = 0;
    }
  }

  // ISSUE: always returns 1
  
  return start_tone;
}
