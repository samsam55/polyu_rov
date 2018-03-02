//mate 2016 internetional
//rov cam

//cam
const int campin[8] = {2, 3, 4, 5, 6, 7, 8, 9};
//sensor
const int pin_temp = 10;
long timer = 0;
long temp_delay = 0;
bool temp_flag = false;
int bit_delay=750; //9:94 10:188 11:375 12:750

//-----------------------------
// Servo
//-----------------------------
#include <Servo.h>
Servo Scam[8];
byte cam_in[8] = {90, 90, 90, 90, 90, 90, 90, 90};
//-----------------------------
// Temperture sensor
//-----------------------------
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(pin_temp);
DallasTemperature sensors(&oneWire);
//tempature in celsius
//float temp() {
 // sensors.requestTemperatures();
  //return sensors.getTempCByIndex(0);
//}

//function
void sensor(float Ftemp) {
  Serial.write(0xE1);
  byte Btemp[4];
  *((float*)Btemp) = Ftemp;
  Serial.write(Btemp, 4);
}

void setup() {
  // put your setup code here, to run once:
  //serial
  Serial.begin(115200);
  for (int i = 0; i < 8; i++)
    Scam[i].attach(campin[i]);
  //temperature  
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.setResolution(12);

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
      // case 0xC2:
      //   sensor();
      //   break;
      case 0xF1:
        Serial.write(0xF2);
        Serial.write(0xC1);
        break;
    }
  }
  if ((millis() - timer) > 1000 && temp_flag == 0) {
    sensors.requestTemperatures();
    temp_delay = millis();
    temp_flag = 1;
  }
  
  if ((millis() - temp_delay) > bit_delay && temp_flag == 1)
  {
    sensor(sensors.getTempCByIndex(0));
    temp_flag = 0;
    timer=millis();
  }
}

