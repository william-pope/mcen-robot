#include <Wire.h>
#include "arduinoFFT.h"
#include <Pixy2.h>
#include <NewPing.h>
#include <SparkFun_TB6612.h>




// I2C COMM

#define I2C_MEGA_ADDR 0x1a

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


// USS

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 30 // Maximum distance (in cm) to ping.

// SOLENOID

#define SolPin 36

// other

// MOTORS


Motor RB = Motor(RB1, RB2, RBS, offsetRB, STNDBY);
Motor LB = Motor(LB1, LB2, LBS, offsetLB, STNDBY);
Motor RT = Motor(RT1, RT2, RTS, offsetRT, STNDBY);
Motor LT = Motor(LT1, LT2, LTS, offsetLT, STNDBY);
 
// USS
float factor = sqrt(1.00 + 25.00 / 273.15) / 60.368; // Speed of sound calculation based on temperature.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
NewPing(46, 48, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
NewPing(46, 44, MAX_DISTANCE)};


// PIXY
Pixy2 pixy;

byte cmd_buffer[9];
byte data_buffer[9];



void setup() 
{
 
  // SOLENOID
  pinMode(SolPin, OUTPUT);

  // I2C
  Wire.begin(I2C_MEGA_ADDR);
  Wire.onReceive(receiveData);
//  Wire.onRequest(send/Data);

  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:  
}

void receiveData(int byte_count) {
  Serial.println("receiveData()");
  for (int i=0; i<byte_count; i++) {
    cmd_buffer[i] = Wire.read();
//    Serial.println(i2c_data, HEX);
  }
  int p_m1 = (cmd_buffer[3] << 8) + cmd_buffer[2];
  int p_m2 = (cmd_buffer[5] << 8) + cmd_buffer[4];
  int p_m3 = (cmd_buffer[7] << 8) + cmd_buffer[6];
  int p_m4 = (cmd_buffer[9] << 8) + cmd_buffer[8];
  bool solState = cmd_buffer[10];

//  Serial.println(p_m1);
  setMotors(p_m1, p_m2, p_m3, p_m4);
//  setSolenoid(solState);
}

void sendData() {
  Serial.println("sendData()");
  // ultra sonic sensor 1
  int d_r0 = getRange(0);
  data_buffer[0] = d_r0;
  data_buffer[1] = d_r0 << 8;

  Serial.println("gate 1");
  // ultra sonic sensor 2
//  int d_r1 = getRange(1);/
  int d_r1 = 230;
  data_buffer[2] = d_r1;
  data_buffer[3] = d_r1 << 8;

  Serial.println("gate 2");
  // pixy - ball position
  int m1_x = getPixy(0);
  data_buffer[4] = m1_x << 8;
  data_buffer[5] = m1_x;
  int m1_y = getPixy(1);
  data_buffer[6] = m1_y << 8;
  data_buffer[7] = m1_y;

  Serial.println("gate 3");

//  TO DO: add gk detection to pixy

//  // pixy - gk position
//  m_x = getPixy(0);
//  data_buffer[4] = m_x << 8;
//  data_buffer[5] = m_x;
//  m_y = getPixy(1);
//  data_buffer[6] = m_y << 8;
//  data_buffer[7] = m_y;

  // start tone
  bool s_t = getStart();
  data_buffer[8] = s_t;

  Wire.write(data_buffer, 9);
}

void setMotors(int p_m1,int p_m2,int p_m3,int p_m4) 
{

  
  // Probably need to make Motor variables (RB,LB,RT,LT) global
  RB.drive(p_m1);
  LB.drive(p_m2);
  RT.drive(p_m3);
  LT.drive(p_m3);

}

void setSolenoid(bool solState) 
{
  if (solState == 1) {
    digitalWrite(SolPin, HIGH);
  } else {
    digitalWrite(SolPin, LOW); 
  }
}

bool getStart() {

  bool startTone = 1;
  return startTone;
}

int getRange(int ID) {

  //sending data in mm
  int distance = sonar[ID].ping_median(5) * factor * 10; 
  delay(60);  
  return distance;
}

int getPixy(int ID) 
{
  pixy.init();

  int i; 
  int m;
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
//    for (i=0; i<pixy.ccc.numBlocks; i++)
//    {
      if (ID == 0) {
        int m = pixy.ccc.blocks[0].m_x;
      }
      else if (ID == 1) {
        int m = pixy.ccc.blocks[0].m_y;
      }
      else if (ID == 2) {
        int m = pixy.ccc.blocks[1].m_x;
      }
      else if (ID == 3) {
        int m = pixy.ccc.blocks[1].m_y;
      }
//    }
  }
    return m;
    delay(100);  

}
