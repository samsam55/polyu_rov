/*
 * 2018 sauvc arduino #3
 */
#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
const int mos_pin = 4;

void setup(void)
{
  Serial.begin(115200);
  //adc
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  ads.begin();
  //magnet
  pinMode(mos_pin, OUTPUT);
}

double voltage() {
  int16_t adc1;
  adc1 = ads.readADC_SingleEnded(1);
  return (adc1 * 0.000125 * 10.10101);  //0.000125 for 1x gain,10.10101 for voltage mutiplyer
}

double current() {
  int16_t adc0;
  adc0 = ads.readADC_SingleEnded(0);
  return (adc0 * 0.000125 * 18.0018); //0.000125 for 1x gain,18.0018 for current mutiplyer
}
void loop(void)
{
  if (Serial.available()) {
    byte d = Serial.read();
    //Serial.write(d);

    switch (d) {
      case 0xEA:
        float volt, cur;
        volt = (float)voltage();
        cur = (float)current();
        byte volt_byte[4], cur_byte[4];
        *((float *)volt_byte) = volt;
        *((float *)cur_byte) = cur;
        Serial.write(0xEA);
        Serial.write(volt_byte, 4);
        Serial.write(cur_byte, 4);
        break;

      case 0xC1:
        byte state;
        Serial.readBytes(&state, 1);
        if (state == 0x00)
          digitalWrite(mos_pin, LOW);
        else if (state == 0x01)
          digitalWrite(mos_pin, HIGH);
        break;

      case 0xF1:
        Serial.write(0xF2);
        Serial.write(0xC1);
        break;
        // put your main code here, to run repeatedly:
    }
  }
}
