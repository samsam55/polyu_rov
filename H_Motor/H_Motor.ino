#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "PID.h"

#define y_buffer_size 30
#define V_BLDC_1_pin 2  //23red
#define V_BLDC_2_pin 3  //23green
#define V_BLDC_3_pin 4  //14red
#define V_BLDC_4_pin 5  //14green

SoftwareSerial yPort(8, 9);
Servo V_BLDC_1, V_BLDC_2, V_BLDC_3, V_BLDC_4;

byte com_mag = 0, y_counter = 0;
boolean yaw_PID_OnOff = false, yaw_set = false;
short com_angle = 0, yaw_setPt, y, last_y, y_buffer[y_buffer_size];
short V_BLDC[4] = {1500, 1500, 1500, 1500}, Last_V_BLDC[4] = {1500, 1500, 1500, 1500};
float yaw_In, yaw_Out, yaw_Kp = 7, yaw_Ki = 0, yaw_Kd = 0;
unsigned long lastTime, yaw_setTime;

PID yaw_PID(&yaw_In, &yaw_Out, &yaw_setPt, yaw_Kp, yaw_Ki, yaw_Kd, true);

void setup() {
  //  EEPROM.put(0, yaw_Kp);
  //  EEPROM.put(4, yaw_Ki);
  //  EEPROM.put(8, yaw_Kd);
  EEPROM.get(0, yaw_Kp);
  EEPROM.get(4, yaw_Ki);
  EEPROM.get(8, yaw_Kd);
  yaw_PID.SetTunings(yaw_Kp, yaw_Ki, yaw_Kd);
  V_BLDC_1.attach(V_BLDC_1_pin);
  V_BLDC_2.attach(V_BLDC_2_pin);
  V_BLDC_3.attach(V_BLDC_3_pin);
  V_BLDC_4.attach(V_BLDC_4_pin);
  V_BLDC_1.writeMicroseconds(1500);
  V_BLDC_2.writeMicroseconds(1500);
  V_BLDC_3.writeMicroseconds(1500);
  V_BLDC_4.writeMicroseconds(1500);
  Serial.begin(115200);

  //  Serial.println(yaw_Kp);
  //  Serial.println(yaw_Ki);
  //  Serial.println(yaw_Kd);

  yPort.begin(57600);
  yaw_setPt = 0;
  yaw_PID.SetOutputLimits(-100, 100);//-150,150
}
void loop() {
  byte y_i = 0;
  //  byte s_i = 0;
  byte y_temp[2];
  //  byte s_temp[14];
  if (!digitalRead(V_BLDC_1_pin) && !digitalRead(V_BLDC_2_pin) && !digitalRead(V_BLDC_3_pin) && !digitalRead(V_BLDC_4_pin)) {
    yPort.listen();
    while (yPort.available()) {
      y_temp[y_i] = yPort.read();
      y_i++;
    }
  } else
    yPort.stopListening();

  if (Serial.available()) {
    byte type = Serial.read();
    //    Serial.println(type, HEX);
    if (type == 0xB1) {
      byte inputBytes[3];
      Serial.readBytes(inputBytes, 3);
      short temp_angle = inputBytes[1] << 8 | inputBytes[0]; //high_byte << 8 | low_byte;
      //      Serial.println(temp_angle);
      if (temp_angle >= -180 && temp_angle <= 180) {
        com_angle = temp_angle;
        if (inputBytes[2] >= 0 && inputBytes[2] <= 255) //(temp_mag >= 0 && temp_mag  <= 255)
          com_mag = inputBytes[2];//temp_mag;
      }
    } else if (type  == 0xB3) {
      byte inputBytes[2];
      Serial.readBytes(inputBytes, 2);
      short temp_setPt = inputBytes[1] << 8 | inputBytes[0]; //high_byte << 8 | low_byte;
      //      Serial.println(temp_setPt);
      if (temp_setPt >= -180 && temp_setPt <= 180) {
        yaw_setPt = temp_setPt;
        yaw_setTime = millis();
        yaw_set = true;
      }
    } else if (type == 0xB5) {
      float input_K[3];
      for (byte i = 0; i < 3; i++) {
        Serial.readBytes((char*)&input_K[i], sizeof(input_K[i]));
        //        Serial.println(input_K[i]);
      }
      yaw_PID.SetTunings(input_K[0], input_K[1], input_K[2]);
      for (byte i = 0; i < 3; i++)
        EEPROM.put(i * 4, input_K[i]);
      EEPROM.get(0, yaw_Kp);
      EEPROM.get(4, yaw_Ki);
      EEPROM.get(8, yaw_Kd);
      Serial.write(0xF5);
      byte Kp_Byte[4];
      *((float*)Kp_Byte) = yaw_Kp;
      Serial.write(Kp_Byte, 4);
      byte Ki_Byte[4];
      *((float*)Ki_Byte) = yaw_Ki;
      Serial.write(Ki_Byte, 4);
      byte Kd_Byte[4];
      *((float*)Kd_Byte) = yaw_Kd;
      Serial.write(Kd_Byte, 4);
    } else if (type == 0xF1) {
      Serial.write(0xF2);
      Serial.write(0xB1);
    } else if (type == 0xF4) {
      EEPROM.get(0, yaw_Kp);
      EEPROM.get(4, yaw_Ki);
      EEPROM.get(8, yaw_Kd);
      Serial.write(0xF5);
      byte Kp_Byte[4];
      *((float*)Kp_Byte) = yaw_Kp;
      Serial.write(Kp_Byte, 4);
      byte Ki_Byte[4];
      *((float*)Ki_Byte) = yaw_Ki;
      Serial.write(Ki_Byte, 4);
      byte Kd_Byte[4];
      *((float*)Kd_Byte) = yaw_Kd;
      Serial.write(Kd_Byte, 4);
    } else if (type == 0xFC) {
      Serial.write(0xE4);
      byte yB[2];
      yB[0] = y & 0xFF;
      yB[1] = y >> 8 & 0xFF;
      Serial.write(yB[0]);
      Serial.write(yB[1]);
    } else if (type == 0xFD) {
      Serial.write(0xE5);
      byte V_BLDC_B[8];
      for (byte i = 0; i < 4; i++) {
        V_BLDC_B[i * 2] = V_BLDC[i] & 0xFF;
        V_BLDC_B[i * 2 + 1] = V_BLDC[i] >> 8 & 0xFF;
        Serial.write(V_BLDC_B[i * 2]);
        Serial.write(V_BLDC_B[i * 2 + 1]);
      }
    } else if (type == 0xB7) {
      byte OnOff;
      Serial.readBytes(&OnOff, 1);
      if (OnOff == 0x01)
        yaw_PID_OnOff = true;
      else if (OnOff == 0x00)
        yaw_PID_OnOff = false;
      yaw_PID.ResetITerm();
      yaw_Out = 0;
    } else if (type == 0xFF) {
      Serial.write(0xE7);
      byte ySetPtB[2];
      ySetPtB[0] = yaw_setPt & 0xFF;
      ySetPtB[1] = yaw_setPt >> 8 & 0xFF;
      Serial.write(ySetPtB[0]);
      Serial.write(ySetPtB[1]);
    }
  }

  if (y_i == 2) {
    //    unsigned long start = micros();
    //    double timeChange = (start - lastTime);
    //    Serial.print(timeChange, 0);
    //    Serial.print(F("\t"));
    //    lastTime = start;

    y = y_temp[1] << 8 | y_temp[0];

    yaw_In = y;
    if (yaw_PID_OnOff)
      yaw_PID.Compute();
    else
      yaw_Out = 0;

    BLDC_filterOutput();

    V_BLDC[0] = constrain(V_BLDC[0], 1200, 1800);
    V_BLDC[1] = constrain(V_BLDC[1], 1200, 1800);
    V_BLDC[2] = constrain(V_BLDC[2], 1200, 1800);
    V_BLDC[3] = constrain(V_BLDC[3], 1200, 1800);


    V_BLDC_1.writeMicroseconds(V_BLDC[0]);
    V_BLDC_2.writeMicroseconds(V_BLDC[1]);
    V_BLDC_3.writeMicroseconds(V_BLDC[2]);
    V_BLDC_4.writeMicroseconds(V_BLDC[3]);

    //    Serial.print(V_BLDC[0]);
    //    Serial.print(F("\t"));
    //    Serial.print(V_BLDC[1]);
    //    Serial.print(F("\t"));
    //    Serial.print(V_BLDC[2]);
    //    Serial.print(F("\t"));
    //    Serial.print(V_BLDC[3]);
    //    Serial.println(F("\t"));

    //    for(byte i = 0; i < 4; i++)
    //      Last_V_BLDC[i] = V_BLDC[i];
    //    last_y = y;
  }
}
