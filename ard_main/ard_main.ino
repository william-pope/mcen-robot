#include <Servo.h>
#include "arduinoFFT.h"
#include <Pixy2.h>
#include <NewPing.h>
#include <SparkFun_TB6612.h>

// MOTORS
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

const int offsetRB = -1;
const int offsetLB = 1;
const int offsetRT = 1;
const int offsetLT = 1;

Motor RB = Motor(RB1, RB2, RBS, offsetRB, STNDBY);
Motor LB = Motor(LB1, LB2, LBS, offsetLB, STNDBY);
Motor RT = Motor(RT1, RT2, RTS, offsetRT, STNDBY);
Motor LT = Motor(LT1, LT2, LTS, offsetLT, STNDBY);

// SERVO
Servo servo;
int servo_pos;

// USS
#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 30 // Maximum distance (in cm) to ping.

float factor = sqrt(1.00 + 25.00 / 273.15) / 60.368; // Speed of sound calculation based on temperature.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
NewPing(46, 22, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
NewPing(46, 53, MAX_DISTANCE)};

// PIXY
Pixy2 pixy;

// MIC
bool getStart();

#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 5000 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
 
arduinoFFT FFT = arduinoFFT();
 
unsigned long microSeconds;
int startFreq = 2000;

double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values
 
unsigned int samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds 

// Initialize buffers

#define ACT_REQ_LENGTH 9
#define OBS_RPL_LENGTH 25

byte act_req_buffer[ACT_REQ_LENGTH];
byte obs_rpl_buffer[OBS_RPL_LENGTH];

void setup() 
{
  // SERIAL
  Serial.begin(115200);
 
  // SERVO
  servo.attach(45);
  servo.write(-10);

  pixy.init();
}

byte req_id[1];

void loop() {  
  if (Serial.available() > 0) {
    Serial.readBytes(req_id, 1); 

    if (req_id[0] == 0xaa) {
      Serial.readBytes(act_req_buffer, ACT_REQ_LENGTH);
      act_request(act_req_buffer);  
    } 
    else if (req_id[0] == 0xbb) {
      obs_request();
    }
  }

//  delay(100);
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
void obs_request() {
  // ultra sonic sensor 1
  int d_r0 = getRange(0);
  obs_rpl_buffer[0] = d_r0;
  obs_rpl_buffer[1] = d_r0 >> 8;

  // ultra sonic sensor 2
  int d_r1 = getRange(1);
  obs_rpl_buffer[2] = d_r1;
  obs_rpl_buffer[3] = d_r1 >> 8;

  int m_x;
  int m_y;

  // pixy - ball position
  m_x = getPixy(1,0);
  obs_rpl_buffer[4] = m_x;
  obs_rpl_buffer[5] = m_x >> 8;
  
  m_y = getPixy(1,1);
  obs_rpl_buffer[6] = m_y;
  obs_rpl_buffer[7] = m_y >> 8;

  // pixy - left post
  m_x = getPixy(2,0);
  obs_rpl_buffer[8] = m_x;
  obs_rpl_buffer[9] = m_x >> 8;
  
  m_y = getPixy(2,1);
  obs_rpl_buffer[10] = m_y;
  obs_rpl_buffer[11] = m_y >> 8;

  // pixy - right post
  m_x = getPixy(3,0);
  obs_rpl_buffer[12] = m_x;
  obs_rpl_buffer[13] = m_x >> 8;
  
  m_y = getPixy(3,1);
  obs_rpl_buffer[14] = m_y;
  obs_rpl_buffer[15] = m_y >> 8;

  // pixy - gk yellow
  m_x = getPixy(4,0);
  obs_rpl_buffer[16] = m_x;
  obs_rpl_buffer[17] = m_x >> 8;
  
  m_y = getPixy(4,1);
  obs_rpl_buffer[18] = m_y;
  obs_rpl_buffer[19] = m_y >> 8;

  // pixy - gk blue
  m_x = getPixy(5,0);
  obs_rpl_buffer[20] = m_x;
  obs_rpl_buffer[21] = m_x >> 8;
  
//  m_y = getPixy(5,1);
  obs_rpl_buffer[22] = m_y;
  obs_rpl_buffer[23] = m_y >> 8;

  // start tone
  bool s_t;
  s_t = getStart();
  obs_rpl_buffer[24] = s_t;
  
  Serial.write(obs_rpl_buffer, OBS_RPL_LENGTH);
}

void setMotors(int p_m1,int p_m2,int p_m3,int p_m4) {
  RB.drive(p_m1);
  LB.drive(p_m2);
  RT.drive(p_m3);
  LT.drive(p_m3);
}

void setServo(bool shoot) {
  if (shoot == 1) {
    servo.write(110);
  } 
  else {
    servo.write(-10);
  }
}

int getRange(int ID) {
  //sending data in mm
  int distance = sonar[ID].ping_median(5) * factor * 10; 
  delay(60);  
  
  return distance;
}

int getPixy(int SIG, int ID) //ID - 0:x and 1:y, Sig - signature of blocl
{
//  pixy.init();  
  
  int i; //block number
  int k; //while loop for averagingb
  int m; //int result value 
  int getSig; //signature of block
  int temp_m; //for averaging
  int result; //return result

  for (int k=0; k<5; k++) {
    pixy.ccc.getBlocks();
    
    if (pixy.ccc.numBlocks) {
      for (i=0; i<pixy.ccc.numBlocks; i++) {
        int getSig = pixy.ccc.blocks[i].m_signature;
        
        if (SIG == getSig) {
          if (ID == 0) {
            int m = pixy.ccc.blocks[i].m_x;
          }
          else if (ID == 1) {
            int m = pixy.ccc.blocks[i].m_y;
          }
        }
        else {
          int m = 0;
        }
      }
    }
    temp_m = temp_m + m;
//    delay(10);
  }
    result = temp_m/5;
    return result;
}



bool getStart() 
{
  bool startTone;
  
  for(int i=0; i<SAMPLES; i++)
  {
      microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
   
      vReal[i] = analogRead(0); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
      vImag[i] = 0; //Makes imaginary term 0 always

      /*remaining wait time between samples if necessary*/
      while(micros() < (microSeconds + samplingPeriod))
      {
        //do nothing
      }
  }

  /*Perform FFT on samples*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  /*Find peak frequency and print peak*/
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  if (peak>startFreq) 
  {
    bool startTone = 1;
  }
  else 
  {
    bool startTone = 0;
  }
//  delay(100); // Delay, then loop again
  
  return startTone;
}
