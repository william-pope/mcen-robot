#include <Wire.h>

#define I2C_MEGA_ADDR 0x1a

byte cmd_buffer[9];
byte data_buffer[3];

void setup() {
  Wire.begin(I2C_MEGA_ADDR);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void receiveData(int n_bytes) {
  Serial.println("received cmd:");
  
  for (int i=0; i<n_bytes; i++) {
    cmd_buffer[i] = Wire.read();
  }

  int p_m1 = (cmd_buffer[3] << 8) + cmd_buffer[2];
  int p_m2 = (cmd_buffer[5] << 8) + cmd_buffer[4];
  int p_m3 = (cmd_buffer[7] << 8) + cmd_buffer[6];
  int p_m4 = (cmd_buffer[9] << 8) + cmd_buffer[8];

  bool o_sol = cmd_buffer[10];
  
  Serial.print("motor 1 pwm: ");
  Serial.println(p_m1);
  Serial.print("motor 2 pwm: ");
  Serial.println(p_m2);
  Serial.print("motor 3 pwm: ");
  Serial.println(p_m3);
  Serial.print("motor 4 pwm: ");
  Serial.println(p_m4);
  Serial.print("o_sol bool: ");
  Serial.println(o_sol);

//  setMotors(cmd_buffer)
//  setSolenoid(cmd_buffer[8])
}

void sendData() {
  Serial.println("received data request");
  
//  querySensors()/

  data_buffer[0] = 0xFF;
  data_buffer[1] = 0x05;
  data_buffer[2] = 0x09;
  
  Wire.write(data_buffer, 3);
}
