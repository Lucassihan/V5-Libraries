#include "movement_globals.h"
#include "vex.h"

class PID // versatile PID class
{
private:
  float count = 0;

public:
  bool scalePID = false;
  bool scalablePID = false;
  bool headingMode = false;
  bool newPidCycle = false;
  float turnKI;
  float kp, ki, kd;
  float fallbackKP, fallbackKD;
  float _0DegKP, _0DegKD;
  float _90DegKP, _90DegKD;
  float _180DegKP, _180DegKD;
  float _270DegKP, _270DegKD;
  float _360DegKP, _360DegKD;

  float clampOutput = 0;
  float clampOutputFor = 0;
  float cycles = 0;
  float timeSinceFirstCycle = 0;

  float maxintegral, integralbound, integralmargin;
  float cur, targ, error = 0, lasterror = 0, deltaerror, sumerror;
  float output, maxOutput, prevOutput;
  float minOutput;
  float slewAmount, slewMin;
  void reset() {
    cur = 0, targ = 0, error = 0, lasterror = 0, deltaerror = 0, sumerror = 0,
    slewAmount = 0, slewMin = 0, prevOutput = 0,
    clampOutput = 0, clampOutputFor = 0;
  }

  void setIntegral(float cap, float bound) {
    maxintegral = cap, integralbound = bound;
  }

  void setConstants(float KP, float KI, float KD) {
    kp = KP;
    ki = KI;
    kd = KD;
  }

  void setFallbackConstants(float KP, float KD) {
    fallbackKP = KP;
    fallbackKD = KD;
  }

  void setScalableConstants(float KI,
    float _0DEGKP, float _0DEGKD,
    float _90DEGKP, float _90DEGKD, 
    float _180DEGKP, float _180DEGKD,
    float _270DEGKP, float _270DEGKD,
    float _360DEGKP, float _360DEGKD) {
    turnKI = KI;

    _0DegKP = _0DEGKP;
    _0DegKD = _0DEGKD;

    _90DegKP = _90DEGKP;
    _90DegKD = _90DEGKD;

    _180DegKP = _180DEGKP;
    _180DegKD = _180DEGKD;

    _270DegKP = _270DEGKP;
    _270DegKD = _270DEGKD;

    _360DegKP = _360DEGKP;
    _360DegKD = _360DEGKD;
  }

  void scaleConstants(float error) {
    if (error >= 0 && error <= 45) {
      kp = _0DegKP;
      kd = _0DegKD;
    }

    if (error > 45 && error <= 135) {
      kp = _90DegKP;
      kd = _90DegKD;
    }

    if (error > 135 && error <= 225) {
      kp = _180DegKP;
      kd = _180DegKD;
    }

    if (error > 225 && error <= 315) {
      kp = _270DegKP;
      kd = _270DegKD;
    }

    if (error > 315) {
      kp = _360DegKP;
      kd = _360DegKD;
    }
  }

  float calculate(float inputError) { // PID calculation using the input as the error
    if (newPidCycle) {
      if (scalablePID && scalePID) {
        scaleConstants(fabs(inputError));
        ki = turnKI;
      } 
      if (scalablePID && not scalePID && headingMode) {
        ki = 0;
        kp = _0DegKP;
        kd = _0DegKD;
      }

      if (scalablePID && not scalePID && not headingMode) {
        ki = turnKI;
        kp = fallbackKP;
        kd = fallbackKD;
      }
    }

    error = inputError;
    deltaerror = error - lasterror;
    if (signum(deltaerror) == signum(error))
      deltaerror = 0; // if sign of deltaerror = sign error deltaerror = 0
    if (fabs(error) < integralbound)
      sumerror += error;
    if (signum(error) != signum(lasterror))
      sumerror = 0; // if sign error != sign lasterror sumerror = 0
    if (fabs(sumerror * ki) > maxintegral)
      sumerror = (maxintegral * signum(sumerror)) / ki;
    // printf(" E %5.3f ", sumerror * ki);
    // printf("\n");
    output = (error * kp) + (sumerror * ki) +
             (deltaerror * kd); // calculates PID output
    lasterror = error;

    if (slewAmount > 0) { // calculate slew if slew is enabled
      if (output > 0) {
        if (output > (prevOutput + slewAmount)) {
          output = prevOutput + slewAmount;
        }
      }
      if (output < 0) {
        if (output < (prevOutput - slewAmount)) {
          output = prevOutput - slewAmount;
        }
      }
    }

    if (fabs(output) > maxOutput) { // cap the output to the max allowed
      output = maxOutput * signum(output);
    }

    if (fabs(output) < minOutput) { // min output
      output = minOutput * signum(output);
    }

    if (newPidCycle) {
      cycles = 0;
      timeSinceFirstCycle = 0;
    } else {
      cycles += 1;
    }

    timeSinceFirstCycle = cycles * 10; //There is a 10ms delay between every movementloop call in the task

    if (timeSinceFirstCycle <= clampOutputFor) {
      output = clampOutput;
    }
    
    newPidCycle = false;
    prevOutput = output;
    return output;
  }
};

extern PID turnPID;
extern PID swingPID;
extern PID drivePID;
extern PID cataPID;
extern PID arcPID;