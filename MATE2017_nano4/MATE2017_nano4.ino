#include <SoftwareSerial.h>

SoftwareSerial BT(12, 11); // RX | TX
int at = 13;
const int pin_motor[4][2] = {{2, 3}, {4, 5}, {8, 9}, {6, 7}};
const int pin_stepper[3][3] = {{A0, A1, A2}, {A3, A4, A5}, {}}; //dir, step, enable
byte s_m[7] = {0, 0, 0, 0, 0, 0, 0};
byte s_s[2] = {0, 0};
int stepperSpeed = 150;

void disc() {
  digitalWrite(at, HIGH);
  delay(100);
  BT.write("AT\r\n");
  delay(100);
  BT.write("AT+DISC\r\n");
  delay(100);
  digitalWrite(at, LOW);
}

void stepper(int i, byte dir) {
  if (dir == 0)
    digitalWrite(pin_stepper[i][2], HIGH);
  else if (dir == 1) {
    digitalWrite(pin_stepper[i][2], LOW);
    digitalWrite(pin_stepper[i][0], HIGH);
    for (int x = 0; x < 5000; x++) {
      digitalWrite(pin_stepper[i][1], HIGH);
      delayMicroseconds(stepperSpeed);
      digitalWrite(pin_stepper[i][1], LOW);
      delayMicroseconds(stepperSpeed);
    }
  }
  else if (dir == 2) {
    digitalWrite(pin_stepper[i][2], LOW);
    digitalWrite(pin_stepper[i][0], LOW);
    for (int x = 0; x < 5000; x++) {
      digitalWrite(pin_stepper[i][1], HIGH);
      delayMicroseconds(stepperSpeed);
      digitalWrite(pin_stepper[i][1], LOW);
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
  BT.begin(9600);
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
        byte temp_s_v[2];
        Serial.readBytes(temp_s_v, 2);
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
          if (temp_s_m[i] != s_m[i]) {
            s_m[i] = temp_s_m[i];
            if (i < 2 ) {
              stepper(i, temp_s_m[i]);
            }
            else if (i > 1 && i < 7) {
              motor(i - 2, temp_s_m[i]);
            }
          }
        }
        break;

      case 0xF1:
        Serial.write(0xF2);
        Serial.write(0xD1);
        break;
    }
  }

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (BT.available() > 20) {
    //Serial.print(BT.readString());
    //Serial.print("receive");
    String a;
    a = BT.readString();
    //Serial.println(a);
    if (a.indexOf('7')) {
      int x = a.indexOf('7');
      if (a[x + 1] == '/n')
        a.remove(x, 1);
      String BTinput = a.substring(x + 1, x + 8);
      Serial.write(0xE1);
      Serial.print(BTinput);
    }
  }
  //] delay(10); // Delay for 10 ms so the next char has time to be received
}
