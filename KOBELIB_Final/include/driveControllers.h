#include "cmath"
#include "globals.h"
#include "vex.h"

void arcadeDriveFcn() {
  double turnImportance = 0.4; //0.2 on omnis, 0.13 on grips 
  
  fr.setStopping(coast);
  fl.setStopping(coast);
  mr.setStopping(coast);
  ml.setStopping(coast);
  br.setStopping(coast);
  bl.setStopping(coast);

  double turnVal = Controller.Axis1.position(percent);
  double forwardVal = Controller.Axis3.position(percent);

  //Expo curves, 0: y=x, 1: y=x^3. To see the curve, go to https://www.desmos.com/calculator/slehx2gwxs
  //Turn scaling, tune kTurn between 0 and 1
  double kTurn = 0;
  double turnNorm = turnVal / 100.0;
  turnVal = ((1 - kTurn) * turnNorm + kTurn * turnNorm * turnNorm * turnNorm) * 100.0;

  // Forward scaling
  double kForward = 0;
  double forwardNorm = forwardVal / 100.0;
  forwardVal = ((1 - kForward) * forwardNorm + kForward * forwardNorm * forwardNorm * forwardNorm) * 100.0;

  // Slew rate limiter — prevents tippy direction reversals on Axis3
  static double slewedForward = 0.0;
  double maxSlew = 6.0; // max % change per 10ms loop (~330ms to reverse fully)
  double slewDelta = forwardVal - slewedForward;
  if (slewDelta >  maxSlew) slewDelta =  maxSlew;
  if (slewDelta < -maxSlew) slewDelta = -maxSlew;
  slewedForward += slewDelta;
  forwardVal = slewedForward;

  // Convert to volts
  double turnVolts = turnVal * 0.127;
  double forwardVolts = forwardVal * 0.127 * (1 - (std::abs(turnVolts) / 127.0) * turnImportance);

  double leftVolts  = forwardVolts + turnVolts;
  double rightVolts = forwardVolts - turnVolts;

  fl.spin(forward, leftVolts, voltageUnits::volt);
  ml.spin(forward, leftVolts, voltageUnits::volt);
  bl.spin(forward, leftVolts, voltageUnits::volt);
  fr.spin(forward, rightVolts, voltageUnits::volt);
  mr.spin(forward, rightVolts, voltageUnits::volt);
  br.spin(forward, rightVolts, voltageUnits::volt);
}

// Curvature drive: You can speed up and slow down the chassis without changing what angle you are moving at
void curvatureDriveFcn() {
  fr.setStopping(coast);
  fl.setStopping(coast);
  mr.setStopping(coast);
  ml.setStopping(coast);
  br.setStopping(coast);
  bl.setStopping(coast);

  double throttle = Controller.Axis3.position(percent);
  double turn = Controller.Axis1.position(percent);

  // Deadband
  if (std::abs(throttle) < 5) throttle = 0;
  if (std::abs(turn) < 5) turn = 0;

  // Expo curves
  double kThrottle = 0;
  double tNorm = throttle / 100.0;
  throttle = ((1 - kThrottle) * tNorm + kThrottle * tNorm * tNorm * tNorm) * 100.0;

  // Inverted expo for turn: scales with throttle
  // At low throttle → linear turn. At full throttle → full inverse expo (small stick = big turn).
  double kTurnMax = 0.5; // max inverse expo strength at full throttle
  double kTurn = kTurnMax * (std::abs(throttle) / 100.0);
  double turnNorm = turn / 100.0;
  double cbrtTurn = copysign(cbrt(std::abs(turnNorm)), turnNorm);
  turn = ((1 - kTurn) * turnNorm + kTurn * cbrtTurn) * 100.0;

  double leftPct, rightPct;

  if (std::abs(throttle) < 5) {
    // In-place turning when not moving
    leftPct = turn;
    rightPct = -turn;
  } else {
    // Curvature: turn input controls radius of curvature, not turn rate
    // Flip curvature when going backward so turning direction matches arcade
    double curvature = turn / 100.0;
    if (throttle < 0) curvature = -curvature;
    leftPct = throttle * (1.0 + curvature);
    rightPct = throttle * (1.0 - curvature);
  }

  // Normalize so neither side exceeds 100%
  double maxVal = std::max(std::abs(leftPct), std::abs(rightPct));
  if (maxVal > 100.0) {
    leftPct = leftPct / maxVal * 100.0;
    rightPct = rightPct / maxVal * 100.0;
  }

  // Convert to volts
  double leftVolts = leftPct * 0.127;
  double rightVolts = rightPct * 0.127;

  fl.spin(forward, leftVolts, voltageUnits::volt);
  ml.spin(forward, leftVolts, voltageUnits::volt);
  bl.spin(forward, leftVolts, voltageUnits::volt);
  fr.spin(forward, rightVolts, voltageUnits::volt);
  mr.spin(forward, rightVolts, voltageUnits::volt);
  br.spin(forward, rightVolts, voltageUnits::volt);
}
