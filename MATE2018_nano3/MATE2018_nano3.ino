//mate 2018
//rov cam

//cam
const int campin[8] = {2, 3, 4, 5, 6, 7, 8, 9};

//-----------------------------
// Servo
//-----------------------------
#include <Servo.h>
Servo Scam[8];
byte cam_in[8] = {90, 90, 90, 90, 90, 90, 90, 90};
//-----------------------------

void setup() {
  // put your setup code here, to run once:
  //serial
  Serial.begin(115200);
  for (int i = 0; i < 8; i++)
    Scam[i].attach(campin[i]);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    byte c = Serial.read(); // get it
    //Serial.print(c); //debug
    switch (c) {
      case 0xC1:
        Serial.readBytes(cam_in, 8);
        for (int i = 0; i < 8; i++)
          Scam[i].write(cam_in[i]);
        break;
      case 0xD1:
        byte fake[7];
        Serial.readBytes(fake, 7);
        break;
      case 0xF1:
        Serial.write(0xF2);
        Serial.write(0xC1);
        break;
    }
  }
}

