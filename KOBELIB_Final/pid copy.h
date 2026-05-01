#include "movement_globals.h"
#include "vex.h"


class PID // versatile PID class
{
private:
  float count = 0;

public:
  float kp, ki, kd;
  float maxintegral, integralbound, integralmargin;
  float cur, targ, error = 0, lasterror = 0, deltaerror, sumerror;
  float output, maxOutput, prevOutput;
  float minOutput;
  float slewAmount, slewMin;
  void reset() {
    cur = 0, targ = 0, error = 0, lasterror = 0, deltaerror = 0, sumerror = 0,
    slewAmount = 0, slewMin = 0, prevOutput = 0;
  }

  void setIntegral(float cap, float bound) {
    maxintegral = cap, integralbound = bound;
  }

  void setConstants(float KP, float KI, float KD) {
    kp = KP;
    ki = KI;
    kd = KD;
  }

  float calculate(float inputError) { // PID calculation using the input as the error
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

    // printf("kp %.3f \n", kp);
    // printf("E %.3f \n", error);
    // printf("O %.3f \n", output);
    prevOutput = output;
    return output;
  }
};

extern PID turnPID;
extern PID drivePID;
extern PID cataPID;
extern PID arcPID;