
byte read_buf[1];
byte write_buf[1];

void setup() {
  Serial.begin(115200);
}

void loop() {
//  if (Serial.available() > 0) {
//    Serial.read();
//  }

  write_buf[0] = 0x1a;
  Serial.write(write_buf, 1);

  Serial.println("hello");

  delay(1000);
}
