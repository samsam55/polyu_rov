#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
  public:

    PID(float* Input, float* Output, short* Setpoint, float Kp, float Ki, float Kd, bool ControllerDirection) {
      myOutput = Output;
      myInput = Input;
      mySetpoint = Setpoint;
      SetOutputLimits(-10, 10);
      myDirection = ControllerDirection;
      SetTunings(Kp, Ki, Kd);
    }

    void Compute() {
      unsigned long now = millis();
      double timeChange = (now - lastTime) / 1000.0;
      double input = *myInput;
      double error;
      if ( (input >= 0 && *mySetpoint >= 0) || (input <= 0 && *mySetpoint <= 0) )
        error = *mySetpoint - input;
      else if (input > 0 && *mySetpoint < 0) {
        short couter_clockwise;
        couter_clockwise = *mySetpoint - input;
        if (couter_clockwise >= -180)
          error = couter_clockwise;
        else
          error = 360 + couter_clockwise;
      } else if (input < 0 && *mySetpoint > 0) {
        short clockwise;
        clockwise = *mySetpoint - input;
        if (clockwise <= 180)
          error = clockwise;
        else
          error = -360 + clockwise;
      }
      //      Serial.println(timeChange, 8);

      if (lastTime == 0) {
        lastTime = now;
        lastError = 0;
        *myOutput = 0;
        return;
      }

      if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
        lastTime = now;
        lastError = error;
        *myOutput = 0;
        return;
      }
      
      ITerm += (ki * error * timeChange);
      double DTerm = kd * (error - lastError) / timeChange;

      //      if (error < 2 && error > -2) {
      //        lastTime = now;
      //        lastInput = *myInput;
      //        *myOutput = ITerm;
      //        return;
      //      }

      double output = kp * error + ITerm + DTerm;

      //      Serial.print(input,1);
      //      Serial.print("\t");
      //      Serial.print(ITerm,4);
      //      Serial.print("\t");
      //      Serial.print(DTerm,4);
      //      Serial.print("\t");
      //      Serial.println(round(output));

      if (output > outMax) output = outMax;
      else if (output < outMin) output = outMin;
      *myOutput = output;

      lastError = error;
      lastTime = now;
    }

    void SetTunings(double Kp, double Ki, double Kd) {
      if (Kp < 0 || Ki < 0 || Kd < 0) return;

      kp = Kp;
      ki = Ki;
      kd = Kd;

      if (!myDirection) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
      }
    }

    void SetOutputLimits(double Min, double Max) {
      if (Min >= Max) return;

      outMin = Min;
      outMax = Max;

      if (*myOutput > outMax) *myOutput = outMax;
      else if (*myOutput < outMin) *myOutput = outMin;
    }

    void ResetITerm () {
      ITerm = 0;
    }

  private:
    double kp, ki, kd;
    bool myDirection;
    float *myInput, *myOutput;
    short *mySetpoint;
    unsigned long lastTime = 0;
    double ITerm, lastError, outMin, outMax;
};
#endif

