#include <Wire.h>
#include "arduinoFFT.h"
#include <Pixy2.h>
#include <NewPing.h>
#include <SparkFun_TB6612.h>

#define I2C_MEGA_ADDR 0x1a

// MOTORS

#define RBS 10;    //Right bottom motor
#define RB1 50; //A11
#define RB2 52; //A12

// motor two
#define LBS 11;    //Left bottom motor
#define LB1 48; //B11
#define LB2 46; //B12

// motor three
#define RTS 9;    //Right top motor
#define RT1 7; //A01
#define RT2 8; //A02

// motor four
#define LTS 3;    //Left top motor
#define LT1 5; //B01
#define LT2 4; //B02

// USS

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 30 // Maximum distance (in cm) to ping.

// SOLENOID

#define SolPin 47; 

// other

byte cmd_buffer[9];
byte data_buffer[9];

void setup() 
{

  // USS
  float factor = sqrt(1.00 + 25.00 / 273.15) / 60.368; // Speed of sound calculation based on temperature.

  NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 51, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(2, 53, MAX_DISTANCE)};

  // SOLENOID
  pinMode(SolPin, OUTPUT);

  // I2C
  Wire.begin(I2C_MEGA_ADDR);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
}

void receiveData(int byte_count) {
  for (int i=0; i<byte_count; i++) {
    cmd_buffer[i] = Wire.read();
//    Serial.println(i2c_data, HEX);
  }
  int p_m1 = (cmd_buffer[3] << 8) + cmd_buffer[2];
  int p_m2 = (cmd_buffer[5] << 8) + cmd_buffer[4];
  int p_m3 = (cmd_buffer[7] << 8) + cmd_buffer[6];
  int p_m4 = (cmd_buffer[9] << 8) + cmd_buffer[8];
  bool SolState = cmd_buffer[10];

  
  setMotors(p_m1, p_m2, p_m3, p_m4);
  setSolenoid(SolState);
}

void sendData() {
  d_r0 = getRange(0);
  data_buffer[0] = d_r0 << 8;
  data_buffer[1] = d_r0;
  d_r1 = getRange(1);
  data_buffer[2] = d_r1 << 8;
  data_buffer[3] = d_r1;


  m_x = getPixy(0);
  data_buffer[4] = m_x << 8;
  data_buffer[5] = m_x;
  m_y = getPixy(1);
  data_buffer[6] = m_y << 8;
  data_buffer[7] = m_y;

  Wire.write(data_buffer);
}

void setMotors(int p_m1,int p_m2,int p_m3,int p_m4) 
{
  const int offsetRB = 1;
  const int offsetLB = 1;
  const int offsetRT = 1;
  const int offsetLT = 1;

  Motor RB = Motor(RB1, RB2, RBS, offsetRB);
  Motor LB = Motor(LB1, LB2, LBS, offsetLB);
  Motor RT = Motor(RT1, RT2, RTS, offsetRT);
  Motor LT = Motor(LT1, LT2, LTS, offsetLB);
   
  RB.drive(p_m1)
  LB.drive(p_m2)
  RT.drive(p_m3)
  LT.drive(p_m3)

}

void setSolenoid(bool SolState) 
{
  if (SolState == True) {
    digitalWrite(SolPin, HIGH);
  } else {
    digitalWrite(SolPin, LOW); 
  }
}

void StartTone() {

  data_buffer[0] = True;
}

int getRange(int ID) {

  //sending data in mm
  distance = sonar[ID].ping_median(5) * factor * 10; 
  delay(60);  
  return distance;
}

int getPixy(int ID) 
{
  Pixy2 pixy;
  pixy.init();
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    for (i=0; i<pixy.ccc.numBlocks; i++) 
    {
      if ID == 0 {
        pixy.ccc.blocks[i].m_x;
      }
      else {
        pixy.ccc.blocks[i].m_y;
      }
    }
  }
    delay(100);  

}
