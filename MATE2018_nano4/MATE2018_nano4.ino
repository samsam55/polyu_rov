#include <SoftwareSerial.h>
#include "A4988.h"

SoftwareSerial BT(12, 11); // RX | TX
int at = 13;
A4988 stepper1(5000, A0, A1, A2);
A4988 stepper2(5000, A3, A4, A5);
const int pin_motor[4][2] = {{2, 3}, {4, 5}, {8, 9}, {6, 7}};
//const int pin_stepper[3][3] = {{A0, A1, A2}, {A3, A4, A5}, {}}; //dir, step, enable
byte s_m[7] = {0, 0, 0, 0, 0, 0, 0};



void stepper(int i, byte dir) {
  if (dir == 0) {
    if (i == 0)
      stepper1.disable();
    else if (i == 1)
      stepper2.disable();
  }
  else if (dir == 1) {
    if (i == 0) {
      stepper1.enable();
      stepper1.move(5000);
    }
    else if (i == 1) {
      stepper2.enable();
      stepper2.move(5000);
    }
  }
  else if (dir == 2) {
    if (i == 0) {
      stepper1.enable();
      stepper1.move(-5000);
    }
    else if (i == 1) {
      stepper2.enable();
      stepper2.move(-5000);
    }
  }

}
void motor (int i, byte s) {

  if (s == 0)
    for (int  x = 0; x < 2; x++)
      digitalWrite(pin_motor[i][x], LOW);

  else if (s == 1) {
    digitalWrite(pin_motor[i][1], LOW);
    digitalWrite(pin_motor[i][0], HIGH);
  }

  else if (s == 2) {
    digitalWrite(pin_motor[i][0], LOW);
    digitalWrite(pin_motor[i][1], HIGH);
  }
}

void setup() {
  BT.begin(9600);
  Serial.begin(115200);
  for (int x = 0; x < 4; x++)
    for (int y = 0; y < 2; y++) {
      pinMode(pin_motor[x][y], OUTPUT);
      digitalWrite(pin_motor[x][y], LOW);
    }

  stepper1.begin(30, 16);
  stepper1.enable();
  stepper2.begin(30, 16);
  stepper2.enable();

}

void loop() {
  if (Serial.available()) {
    byte d = Serial.read();
    //Serial.write(d);

    switch (d) {
      case 0xD2:
        byte temp_s_v[2];
        Serial.readBytes(temp_s_v, 2);
        break;

      case 0xD1:
        byte temp_s_m[7];
        Serial.readBytes(temp_s_m, 7);
        int count;
        count = 0;
        for (int i = 0; i < 7; i++) {
          if (temp_s_m[i] != 0 && count == 0)
            count++;
          else temp_s_m[i] = 0;
          if (temp_s_m[i] != s_m[i]) {
            s_m[i] = temp_s_m[i];
            if (i < 2 ) {
              stepper(i, temp_s_m[i]);
            }
            else if (i > 1 && i < 6) {
              motor(i - 2, temp_s_m[i]);
            }
          }
        }
        break;
      case 0xC1:
        byte temp_cam[8];
        Serial.readBytes(temp_cam, 2);
        break;

      case 0xF1:
        Serial.write(0xF2);
        Serial.write(0xD1);
        break;
    }
  }
}
