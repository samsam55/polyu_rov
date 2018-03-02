#include <SoftwareSerial.h>


const int pin_motor[4][2] = {{2, 3}, {4, 5}, {8, 9}, {6, 7}};
const int pin_stepper[3][3] = {{A0, A1, A2}, {A3, A4, A5}, {}}; //dir, step, enable
byte s_m[4] = {0, 0, 0, 0};
byte s_s[3] = {0, 0, 0};
int stepperSpeed = 150;

void stepper(int i, byte dir) {
  if (dir == 0)
    digitalWrite(pin_stepper[i][2], HIGH);
  else if (dir == 1) {
    digitalWrite(pin_stepper[i][2], LOW);
    digitalWrite(pin_stepper[i][0], HIGH);
    for (int i = 0; i < 5000; i++) {
      digitalWrite(pin_stepper[i][1], HIGH);
      delayMicroseconds(stepperSpeed);
      digitalWrite(pin_stepper[1], LOW);
      delayMicroseconds(stepperSpeed);
    }
  }
  else if (dir == 2) {
    digitalWrite(pin_stepper[i][2], LOW);
    digitalWrite(pin_stepper[i][0], LOW);
    for (int i = 0; i < 5000; i++) {
      digitalWrite(pin_stepper[i][1], HIGH);
      delayMicroseconds(stepperSpeed);
      digitalWrite(pin_stepper[1], LOW);
      delayMicroseconds(stepperSpeed);
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
    digitalWrite(pin_motor[i][1], LOW);
    digitalWrite(pin_motor[i][1], HIGH);
  }
}

void setup() {

  Serial.begin(115200);
  for (int x = 0; x < 4; x++)
    for (int y = 0; y < 2; y++) {
      pinMode(pin_motor[x][y], OUTPUT);
      digitalWrite(pin_motor[x][y], LOW);
    }

  for (int x = 0; x < 3; x++)
    for (int y = 0; y < 3; y++) {
      pinMode(pin_stepper[x][y], OUTPUT);
      digitalWrite(pin_stepper[x][y], LOW);
    }
    for (int x = 0; x < 3; x++)
  digitalWrite(pin_stepper[x][2], HIGH);
  // put your setup code here, to run once:

}

void loop() {
  if (Serial.available()) {
    byte d = Serial.read();
    //Serial.write(d);

    switch (d) {
      case 0xD1:
        break;

      case 0xD2:
        byte temp_s_m[7];
        Serial.readBytes(temp_s_m, 7);
        int count;
        count = 0;
        for (int i = 0; i < 7; i++) {
          if (temp_s_m[i] != 0 && count == 0)
            count++;
          else temp_s_m[i] = 0;
          if (i < 3 && temp_s_m[i] != 0) {
            stepper(i, temp_s_m[i]);
          }
          else if (i > 2 && temp_s_m[i] != 0) {
            motor(i - 3, temp_s_m[i]);
          }
        }
        break;

      case 0xF1:
        Serial.write(0xF2);
        Serial.write(0xD1);
        break;
        // put your main code here, to run repeatedly:
    }
  }

}
