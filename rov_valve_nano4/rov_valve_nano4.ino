//mate 2016 internetional
//rov valve

//valve
const int pin_valve[3] = {2, 3, 4};
byte s_v[3] = {0, 0, 0};
//motor
const int pin_motor[4][2] = {{10, 11}, {12, 13}, {14, 15}, {16, 17}};
byte s_m[3] = {0, 0, 0};
void motor(int i, byte s) {
  if (s != s_m[i]) {
    s_m[i] = s;
    if (i == 0)
      for (int x = 0; x < 2; x++)
        digitalWrite(pin_motor[1][x], LOW);
    if (i == 1)
      for (int x = 0; x < 2; x++)
        digitalWrite(pin_motor[0][x], LOW);
    if (i == 2)
      for (int x = 0; x < 2; x++)
        digitalWrite(pin_motor[3][x], LOW);
    for (int x = 0; x < 2; x++)
      digitalWrite(pin_motor[i][x], LOW);
    switch (s) {
      case 1:
        if (i == 0)
          for (int x = 0; x < 2; x++)
            digitalWrite(pin_motor[1][x], HIGH);
        else if (i == 1)
          for (int x = 0; x < 2; x++)
            digitalWrite(pin_motor[0][x], HIGH);
        else if (i == 2)
          for (int x = 0; x < 2; x++)
            digitalWrite(pin_motor[3][x], HIGH);
        digitalWrite(pin_motor[i][0], LOW);
        digitalWrite(pin_motor[i][1], HIGH);
        break;
      case 2:
        if (i == 0)
          for (int x = 0; x < 2; x++)
            digitalWrite(pin_motor[1][x], HIGH);
        else if (i == 1)
          for (int x = 0; x < 2; x++)
            digitalWrite(pin_motor[0][x], HIGH);
        else if (i == 2)
          for (int x = 0; x < 2; x++)
            digitalWrite(pin_motor[3][x], HIGH);
        digitalWrite(pin_motor[i][0], HIGH);
        digitalWrite(pin_motor[i][1], LOW);
        break;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int x = 0; x < 4; x++)
    for (int y = 0; y < 2; y++) {
      pinMode(pin_motor[x][y], OUTPUT);
      digitalWrite(pin_motor[x][y], LOW);
    }
  for (int i = 0; i < 3; i++)
    pinMode(pin_valve[i], OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    byte c = Serial.read(); // get it
    //Serial.print(c); //debug
    switch (c) {
      //valve
      case 0xD1:
        Serial.readBytes(s_v, 3);
        for (int i = 0; i < 3; i++) {
          if (s_v[i]) digitalWrite(pin_valve[i], HIGH);
          else digitalWrite(pin_valve[i], LOW);
        }
        break;
      //arm
      case 0xD2:
        byte temp_s_m[3];
        Serial.readBytes(temp_s_m, 3);
        //protect
        if (temp_s_m[0] != 0) {
          temp_s_m[1] = 0;
          temp_s_m[2] = 0;
        }
        else if (temp_s_m[1] != 0)
          temp_s_m[2] = 0;
        for (int i = 0; i < 3; i++)
          motor(i, temp_s_m[i]);
        break;
      //id
      case 0xF1:
        Serial.write(0xF2);
        Serial.write(0xD1);
        break;
    }
  }
}
