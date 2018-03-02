#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "PID.h"
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>    //For I2C

#define V_BLDC_1_pin 2  //red
#define V_BLDC_2_pin 3  //green

SoftwareSerial prPort(8, 9);
Servo V_BLDC_1, V_BLDC_2;
MS5803 sensor(ADDRESS_HIGH);

boolean depth_PID_OnOff = true, pitch_PID_OnOff = true;
short depth_setPt, pitch_setPt, roll_setPt, pr[2], last_pr[2];
short V_BLDC[2] = {1500, 1500}, Last_V_BLDC[2] = {1500, 1500};
float error = 0.0, r = 1.706109235, q = 0.05, p = 1.0, k = 0.0, f_depth_cm = 0.0;
float depth_In, depth_Out, depth_Kp = 5, depth_Ki = 0, depth_Kd = 0;
float pitch_In, pitch_Out, pitch_Kp = 10, pitch_Ki = 0, pitch_Kd = 0;
unsigned long lastTime;

PID depth_PID(&depth_In, &depth_Out, &depth_setPt, depth_Kp, depth_Ki, depth_Kd, true);
PID pitch_PID(&pitch_In, &pitch_Out, &pitch_setPt, pitch_Kp, pitch_Ki, pitch_Kd, true);

//For I2C pressure
float setdep() {
  float initialpressure = 0;
  prPort.stopListening();
  sensor.getMeasurements(ADC_4096);
  initialpressure = sensor.getPressure(ADC_4096);
  return initialpressure;
}

float getdep() {
  float depth, pressure;
  prPort.stopListening();
  sensor.getMeasurements(ADC_4096);
  pressure = sensor.getPressure(ADC_4096);
  depth = (pressure - error) * 10 / 9.8099;       //9.8099 in Hong Kong, 9.7807 in singapore
  return depth;//depth in cm
}

void setup() {
  //  EEPROM.put(0, depth_Kp);
  //  EEPROM.put(4, depth_Ki);
  //  EEPROM.put(8, depth_Kd);
  //  EEPROM.put(12, pitch_Kp);
  //  EEPROM.put(16, pitch_Ki);
  //  EEPROM.put(20, pitch_Kd);
  EEPROM.get(0, depth_Kp);
  EEPROM.get(4, depth_Ki);
  EEPROM.get(8, depth_Kd);
  EEPROM.get(12, pitch_Kp);
  EEPROM.get(16, pitch_Ki);
  EEPROM.get(20, pitch_Kd);
  depth_PID.SetTunings(depth_Kp, depth_Ki, depth_Kd);
  pitch_PID.SetTunings(pitch_Kp, pitch_Ki, pitch_Kd);
  EEPROM.get(24, error);

  V_BLDC_1.attach(V_BLDC_1_pin);
  V_BLDC_2.attach(V_BLDC_2_pin);
  V_BLDC_1.writeMicroseconds(1500);
  V_BLDC_2.writeMicroseconds(1500);
  Serial.begin(115200);

  //  Serial.println(depth_Kp);
  //  Serial.println(depth_Ki);
  //  Serial.println(depth_Kd);
  //  Serial.println(pitch_Kp);
  //  Serial.println(pitch_Ki);
  //  Serial.println(pitch_Kd);

  prPort.begin(57600);
  depth_setPt = 0;
  pitch_setPt = 0;
  roll_setPt = 0;
  depth_PID.SetOutputLimits(-400, 400);
  pitch_PID.SetOutputLimits(-200, 200);
  //For I2C
  sensor.reset();
  sensor.begin();
  error = setdep();
  //Serial.println("setup done");
}
void loop() {
  byte pr_i = 0;
  //  byte s_i = 0;
  byte pr_temp[4];
  //  byte s_temp[14];
  if (!digitalRead(V_BLDC_1_pin) && !digitalRead(V_BLDC_2_pin)) {
    prPort.listen();
    while (prPort.available()) {
      pr_temp[pr_i] = prPort.read();
      pr_i++;
    }
  } else
    prPort.stopListening();

  if (Serial.available()) {
    byte type = Serial.read();
    //    Serial.println(type, HEX);
    if (type == 0xA3) {
      byte degree; // = Serial.read();
      Serial.readBytes(&degree, 1);
      short c_degree = degree - 128;
      //      Serial.println(c_degree);
      short temp_diff = c_degree - pitch_setPt;
      if (temp_diff < 20 && temp_diff > -20)
        pitch_setPt = c_degree;
    } else if (type == 0xA5) {
      byte degree; // = Serial.read();
      Serial.readBytes(&degree, 1);
      short c_degree = degree - 128;
      //      Serial.println(c_degree);
      short temp_diff = c_degree - roll_setPt;
      if (temp_diff < 20 && temp_diff > -20)
        roll_setPt = c_degree;
    } else if (type  == 0xA1) {
      byte inputBytes[2];
      Serial.readBytes(inputBytes, 2);
      short temp_setPt = inputBytes[1] << 8 | inputBytes[0]; // high_byte << 8 | low_byte;
      //      Serial.println(temp_setPt);
      short temp_diff = temp_setPt - depth_setPt;
      if (temp_diff < 200 && temp_diff > -200)
        depth_setPt = temp_setPt;
    } else if (type == 0xA7) {
      float input_K[3];
      for (byte i = 0; i < 3; i++) {
        Serial.readBytes((char*)&input_K[i], sizeof(input_K[i]));
        //        Serial.println(input_K[i]);
      }
      depth_PID.SetTunings(input_K[0], input_K[1], input_K[2]);
      for (byte i = 0; i < 3; i++)
        EEPROM.put(i * 4, input_K[i]);
      EEPROM.get(0, depth_Kp);
      EEPROM.get(4, depth_Ki);
      EEPROM.get(8, depth_Kd);
      Serial.write(0xF7);
      byte dKp_Byte[4];
      *((float*)dKp_Byte) = depth_Kp;
      Serial.write(dKp_Byte, 4);
      byte dKi_Byte[4];
      *((float*)dKi_Byte) = depth_Ki;
      Serial.write(dKi_Byte, 4);
      byte dKd_Byte[4];
      *((float*)dKd_Byte) = depth_Kd;
      Serial.write(dKd_Byte, 4);
    } else if (type == 0xA9) {
      float input_K[3];
      for (byte i = 0; i < 3; i++) {
        Serial.readBytes((char*)&input_K[i], sizeof(input_K[i]));
        //        Serial.println(input_K[i]);
      }
      pitch_PID.SetTunings(input_K[0], input_K[1], input_K[2]);
      for (byte i = 0; i < 3; i++)
        EEPROM.put((i * 4) + 12,  input_K[i]);
      EEPROM.get(12, pitch_Kp);
      EEPROM.get(16, pitch_Ki);
      EEPROM.get(20, pitch_Kd);
      Serial.write(0xF9);
      byte pKp_Byte[4];
      *((float*)pKp_Byte) = pitch_Kp;
      Serial.write(pKp_Byte, 4);
      byte pKi_Byte[4];
      *((float*)pKi_Byte) = pitch_Ki;
      Serial.write(pKi_Byte, 4);
      byte pKd_Byte[4];
      *((float*)pKd_Byte) = pitch_Kd;
      Serial.write(pKd_Byte, 4);
    } else if (type == 0xF1) {
      Serial.write(0xF2);
      Serial.write(0xA1);
    } else if (type == 0xF4) {
      EEPROM.get(0, depth_Kp);
      EEPROM.get(4, depth_Ki);
      EEPROM.get(8, depth_Kd);
      EEPROM.get(12, pitch_Kp);
      EEPROM.get(16, pitch_Ki);
      EEPROM.get(20, pitch_Kd);

      Serial.write(0xF7);
      byte dKp_Byte[4];
      *((float*)dKp_Byte) = depth_Kp;
      Serial.write(dKp_Byte, 4);
      byte dKi_Byte[4];
      *((float*)dKi_Byte) = depth_Ki;
      Serial.write(dKi_Byte, 4);
      byte dKd_Byte[4];
      *((float*)dKd_Byte) = depth_Kd;
      Serial.write(dKd_Byte, 4);

      Serial.write(0xF9);
      byte pKp_Byte[4];
      *((float*)pKp_Byte) = pitch_Kp;
      Serial.write(pKp_Byte, 4);
      byte pKi_Byte[4];
      *((float*)pKi_Byte) = pitch_Ki;
      Serial.write(pKi_Byte, 4);
      byte pKd_Byte[4];
      *((float*)pKd_Byte) = pitch_Kd;
      Serial.write(pKd_Byte, 4);
    } else if (type == 0xFA) {
      Serial.write(0xE2);
      byte prB[4];
      for (byte i = 0; i < 2; i++) {
        prB[i * 2] = pr[i] & 0xFF;
        prB[i * 2 + 1] = pr[i] >> 8 & 0xFF;
        Serial.write(prB[i * 2]);
        Serial.write(prB[i * 2 + 1]);
      }
    } else if (type == 0xFB) {
      Serial.write(0xE3);
      byte V_BLDC_B[4];
      for (byte i = 0; i < 2; i++) {
        V_BLDC_B[i * 2] = V_BLDC[i] & 0xFF;
        V_BLDC_B[i * 2 + 1] = V_BLDC[i] >> 8 & 0xFF;
        Serial.write(V_BLDC_B[i * 2]);
        Serial.write(V_BLDC_B[i * 2 + 1]);
      }
    } else if (type == 0xFE) {
      Serial.write(0xE6);
      byte f_depth_cm_B[2];
      short r_f_depth_cm;
      r_f_depth_cm = round(f_depth_cm);
      f_depth_cm_B[0] = r_f_depth_cm & 0xFF;
      f_depth_cm_B[1] = r_f_depth_cm >> 8 & 0xFF;
      Serial.write(f_depth_cm_B[0]);
      Serial.write(f_depth_cm_B[1]);
    } else if (type == 0xAB) {
      byte OnOff;
      Serial.readBytes(&OnOff, 1);
      if (OnOff == 0x01) {
        depth_PID_OnOff = true;
        //        digitalWrite(13,HIGH);
      }
      else if (OnOff == 0x00) {
        depth_PID_OnOff = false;
        //        digitalWrite(13,LOW);
      }
      depth_PID.ResetITerm();
      depth_Out = 0;
      //      Serial.write(0xEE);
    } else if (type == 0xAD) {
      byte OnOff;
      Serial.readBytes(&OnOff, 1);
      if (OnOff == 0x01)
        pitch_PID_OnOff = true;
      else if (OnOff == 0x00)
        pitch_PID_OnOff = false;
      pitch_PID.ResetITerm();
      pitch_Out = 0;
      //      Serial.write(0xEA);
    } else if (type == 0xAC) {
      //error = (analogRead(p_sen_pin) / 1023.0) - 0.04;
      error = setdep();
      EEPROM.put(24, error);
    }

  }

  if (pr_i == 4) {
    //    unsigned long start = micros();
    //    double timeChange = (start - lastTime);
    //    Serial.print(timeChange, 0);
    //    Serial.print(F("\t"));
    //    lastTime = start;

    pr[0] = pr_temp[1] << 8 | pr_temp[0];
    pr[1] = pr_temp[3] << 8 | pr_temp[2];

    short diffPr[2];
    for (byte i = 0; i < 2; i++) {
      diffPr[i] = abs(last_pr[i] - pr[i]);
      if (diffPr[i] <= 360 && diffPr[i] >= 0) {
        if ( diffPr[i] > 30 && diffPr[i] < 330 )
          pr[i] = last_pr[i];
      } else
        pr[i] = last_pr[i];
    }

    //    Serial.print(pr[0]);
    //    Serial.print(F("\t"));
    //    Serial.print(pr[1]);
    //    Serial.print(F("\t"));

    //short p_sen_Val = analogRead(p_sen_pin);
    //float depth_cm = (p_sen_Val / 1023.0 - error - 0.04) / 0.00369 * 10.1974;
    float depth_cm = getdep();
    KalmanUpdate(depth_cm);

    //    Serial.print(f_depth_cm);
    //    Serial.print(F("\t"));

    depth_In = f_depth_cm;
    if (depth_PID_OnOff)
      depth_PID.Compute();
    else
      depth_Out = 0;

    //    if ((pr[0] - pitch_setPt) < 3 && (pr[0] - pitch_setPt) > -3) pr[0] = 0;
    pitch_In = pr[0];
    if (pitch_PID_OnOff)
      pitch_PID.Compute();
    else
      pitch_Out = 0;

    //Serial.print(depth_Out);
    //Serial.print(F("\t"));
    //Serial.print(pitch_Out);
    //Serial.println(F("\t"));

    BLDC_filterOutput();

    //    if (V_BLDC[0] != Last_V_BLDC[0])
    //      V_BLDC_1.writeMicroseconds(V_BLDC[0]);
    //    if (V_BLDC[1] != Last_V_BLDC[1])
    //      V_BLDC_2.writeMicroseconds(V_BLDC[1]);

    V_BLDC[0] = constrain(V_BLDC[0], 1100, 1900);
    V_BLDC[1] = constrain(V_BLDC[1], 1100, 1900);

    V_BLDC_1.writeMicroseconds(V_BLDC[0]);
    V_BLDC_2.writeMicroseconds(V_BLDC[1]);
    /*
      Serial.print(V_BLDC[0]);
      Serial.print(F("\t"));
      Serial.print(V_BLDC[1]);
      Serial.println(F("\t"));
    */

    //    for(byte i = 0; i < 2; i++)
    //      Last_V_BLDC[i] = V_BLDC[i];
    for (byte i = 0; i < 2; i++)
      last_pr[i] = pr[i];
  }
}
