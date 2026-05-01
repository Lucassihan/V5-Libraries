#include "vex.h"
#include "movement.h"
#include "threads.h"
#include <climits>
#include <cstdlib>
#include <iterator>
#include <utility>
using namespace vex;
#include <cmath>
#include "math.h"
#include <vector>
#include "globals.h"

float endtime = 0;
float starttime = 0;

// ANGULAR PID CONSTANTS
float TKI = 0.01;

float _0DegTKP = 0.27;
float _0DegTKD = 1.4;

float _90DegTKP = 0.27;
float _90DegTKD = 1.46;

float _180DegTKP = 0.2;
float _180DegTKD = 1.2;

float _270DegTKP = 0.2;
float _270DegTKD = 1.38;

float _360DegTKP = 0.2;
float _360DegTKD = 1.38;

// SWING PID CONSTANTS
float SKI = 0.01;

float _0DegSKP = 0.35;
float _0DegSKD = 1.3;

float _90DegSKP = 0.3;
float _90DegSKD = 1.43;

float _180DegSKP = 0.31;
float _180DegSKD = 1.2;

float _270DegSKP = 0.2;
float _270DegSKD = 1.38;

float _360DegSKP = 0.2;
float _360DegSKD = 1.38;

// HEADING PID CONSTANTS
float HKP = _0DegTKP;
float HKD = _0DegTKD;

// LINEAR PID CONSTANTS
float YKP = 1.1; //1.3
float YKI = 0;
float YKD = 5.5; //10

// ARC CORRECTION CONSTANTS
float ARC_CT_KP = 1;  // cross-track P (distance from arc)
float ARC_HDG_KP = 0.6; // heading P (alignment with arc tangent)

// SLEW
float slew = 0;

//TRACK WIDTH - Important for arcs and BLRS Curvature calculations (which is the curveTo backend)
float trackwidth = 11.75;

//Default speed caps
float ymaxSpeed = 12.7;
float amaxSpeed = 12.7;
float hmaxSpeed = ymaxSpeed ;

//Drive minimum output
float drive_min_output = 0;

//Sweep turns
float sweep_ratio = 0; 
//A sweep ratio of 0.5 would mean the locked side goes half as fast as the turning side. 
// A sweep ratio of 1 would mean both sides output the same value.

//Straightline Jumpstarts
bool bangbangJumpstart = false;
double bangbangDist = 0.25;

//Smart turning early exits
bool enableTurningEarlyExits = false;
bool turningEarlyExits = false;
double turningEarlyExitLength = 0.9;

//Lateral movement early exits
double exitMovementIn = 0;

//Don't touch this stuff
int turnmargin = 65;
int turnstop = 5;
int track = 10;
float breakang = 5;

void autonStart() {
  chassis.resetData();
  chassis.ytarg = 0;
  chassis.xtarg = 0;
  chassis.htarg = 0;
  starttime = Brain.timer(msec);
}

void autonStop() { //Kobe used this, idk what its for ngl
    endtime = Brain.timer(msec) - starttime;
    delay(10000000);
}

void auto_settings() {
  autonStart();
  move.turndisablelength = 6; // After the robot is this distance away from the point, heading correction ends.
  move.bigTime = 300;
  turnPID.integralbound = 10;
  turnPID.maxintegral = 2;

  hookUp();

  antiJam = true;

  move.seamlessTransitions = false;

  turnPID.scalePID = true;
  drivePID.scalePID = false;

  turnPID.scalablePID = true;
  drivePID.scalablePID = false;

  move.ptpCosineScaling = true;

  autonSort = true;
  dropOdom();

  turnPID.setScalableConstants(
  TKI,
  _0DegTKP, _0DegTKD,
  _90DegTKP, _90DegTKD,
  _180DegTKP, _180DegTKD,
  _270DegTKP, _270DegTKD,
  _360DegTKP, _360DegTKD
);

  swingPID.integralbound = 10;
  swingPID.maxintegral = 2;
  swingPID.scalePID = true;
  swingPID.scalablePID = true;

  swingPID.setScalableConstants(
  SKI,
  _0DegSKP, _0DegSKD,
  _90DegSKP, _90DegSKD,
  _180DegSKP, _180DegSKD,
  _270DegSKP, _270DegSKD,
  _360DegSKP, _360DegSKD
);

}

float angleDiff(float target, float current) {
  float diff = target - current;

  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;

  return diff;
}                                                                                                                          

///////////////////////////////////////////////////////////////////////////////////////
//  __          __     _      _        _____  ______          _____ _____ _   _  _____
//  \ \        / /\   | |    | |      |  __ \|  ____|   /\   |  __ \_   _| \ | |/ ____|
//   \ \  /\  / /  \  | |    | |      | |__) | |__     /  \  | |  | || | |  \| | |  __ 
//    \ \/  \/ / /\ \ | |    | |      |  _  /|  __|   / /\ \ | |  | || | | . ` | | |_ |
//     \  /\  / ____ \| |____| |____  | | \ \| |____ / ____ \| |__| || |_| |\  | |__| |
//      \/  \/_/    \_\______|______| |_|  \_\______/_/    \_\_____/_____|_| \_|\_____|
///////////////////////////////////////////////////////////////////////////////////////

// Sensor offsets
double sensor1Offset = 18.97;
double sensor2Offset = 11.9;
double sensor3Offset = 19.1;
double sensor4Offset = 18.7; //CHANGE FOR SKILLS

void readWall(int wall, int sensor, bool bypassSettleDelay = false) {
  rightdrive.stop(brake);
  leftdrive.stop(brake);

  if (not bypassSettleDelay) {
    delay(10);
  }

  double distanceReading = 0;
  double sensorOffset = 0;

  if (sensor == 1) { distanceReading = DistSensor1.objectDistance(inches); sensorOffset = sensor1Offset; }
  else if (sensor == 2) { distanceReading = DistSensor2.objectDistance(inches); sensorOffset = sensor2Offset; }
  else if (sensor == 3) { distanceReading = DistSensor3.objectDistance(inches); sensorOffset = sensor3Offset; }
  else if (sensor == 4) { distanceReading = DistSensor4.objectDistance(inches); sensorOffset = sensor4Offset; }

  double relativeOffset = sensorOffset - distanceReading;

  // Walls 1 & 2 are positive edges (+48), walls 3 & 4 are negative edges (-48)
  double sign;
  if (wall <= 2) {
    sign = 1;
  } else {
    sign = -1;
  }

  double newPos = sign * (48 + relativeOffset);

  // Walls 1 & 3 set X, walls 2 & 4 set Y
  if (wall == 1 || wall == 3) {
    chassis.setPos(newPos, chassis.y, chassis.h);
  }
  
  if (wall == 2 || wall == 4) {
    chassis.setPos(chassis.x, newPos, chassis.h);
  }

}

/*
Note: walls look like this:

   2
3     1
   4

*/

////////////////////////////////////////////////////////////////////
//   ____          _   _  _____        ____          _   _  _____ 
//  |  _ \   /\   | \ | |/ ____|      |  _ \   /\   | \ | |/ ____|
//  | |_) | /  \  |  \| | |  __ ______| |_) | /  \  |  \| | |  __ 
//  |  _ < / /\ \ | . ` | | |_ |______|  _ < / /\ \ | . ` | | |_ |
//  | |_) / ____ \| |\  | |__| |      | |_) / ____ \| |\  | |__| |
//  |____/_/    \_\_| \_|\_____|      |____/_/    \_\_| \_|\_____|
////////////////////////////////////////////////////////////////////

void bangbangDrive(float dist, float breakWithin, bool coastExit = true, float coastFor = 10) {
  // Record starting position
  float startX = chassis.x;
  float startY = chassis.y;

  float targetDist = fabs(dist);
  bool backward = (dist < 0);

  float traveled = 0;
  float breakDist = targetDist * breakWithin;

  // Decide direction
  if (!backward) {
    move.voldrive(12.7, 12.7);
  } else {
    move.voldrive(-12.7, -12.7);
  }

  // Bang-bang loop
  while (traveled < (targetDist - breakDist)) {
    float dx = chassis.x - startX;
    float dy = chassis.y - startY;
    traveled = sqrt(dx * dx + dy * dy);
    delay(10);
  }

  // Brake hard
  if (coastExit) {
    leftdrive.stop(coast);
    rightdrive.stop(coast);
    delay(coastFor);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//    ____  _    _ _____ _____ _  __  ______ _    _ _   _  _____ _______ _____ ____  _   _  _____ 
//   / __ \| |  | |_   _/ ____| |/ / |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
//  | |  | | |  | | | || |    | ' /  | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___  
//  | |  | | |  | | | || |    |  <   |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \ 
//  | |__| | |__| |_| || |____| . \  | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |
//   \___\_\\____/|_____\_____|_|\_\ |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/ 
/////////////////////////////////////////////////////////////////////////////////////////////////////

float calculateTargetHeading(float targetX, float targetY, bool ifBackward, float fromPointX = 0, float fromPointY = 0) {
  float dx;
  float dy;

  if (fromPointX == 0 && fromPointY == 0) {
    dx = targetX - chassis.x;
    dy = targetY - chassis.y;
  } else {
    dx = targetX - fromPointX;
    dy = targetY - fromPointY;
  }

  // atan2 gives 0° = +X, 90° = +Y
  float headingRad = atan2(dy, dx);
  float headingDeg = headingRad * 180.0 / M_PI;

  // Rotate frame so 0° = +Y
  headingDeg = 90 - headingDeg;

  // Normalize to [0, 360)
  while (headingDeg < 0) headingDeg += 360;
  while (headingDeg >= 360) headingDeg -= 360;

  if (ifBackward) {
    headingDeg += 180;
    if (headingDeg >= 360) headingDeg -= 360;
  }

  return headingDeg;
}
                                               
//These exist for parameter autofill. Way quicker to write driveForward(24) than move.straighth(24, chassis.h, YKP, YKD, HKP, HKD, ymaxSpeed, hmaxSpeed, slew, 3);
//The root cause for the lack of autofill is because ClangD conflicts with the VexCode compiler, but ClangD is goated so just keep it.

//Quick forwards backward motions
void driveForward(float dist, float heading = chassis.h, float breakdist = 3, float yspeed = ymaxSpeed, float ykp = YKP, float ykd = YKD) {

  if (bangbangJumpstart) {
    bangbangDrive(dist * bangbangDist, 0.2, true);
    dist -= dist * bangbangDist;
  }

  if (exitMovementIn != 0) {
    move.straighth(dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, 0);
    delay(exitMovementIn);
    exitMovementIn = 0;
  }

  move.straighth(dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);

}

void driveBackward(float dist, float heading = chassis.h, float breakdist = 3, float yspeed = ymaxSpeed, float ykp = YKP, float ykd = YKD) {

  if (bangbangJumpstart) {
    bangbangDrive(-dist * bangbangDist, 0.2, true);
    dist -= dist * bangbangDist;
  }

  if (exitMovementIn != 0) {
    move.straighth(-dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, 0);
    delay(exitMovementIn);
    exitMovementIn = 0;
  }

  move.straighth(-dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);

}

// Easy turning motions
void face(float heading, float breakang = 8, int dir = 0) {

  move.face(heading, amaxSpeed, breakang, dir);

}

void facePoint(float x, float y, float breakang = 8, bool backward = false, int dir = 0) {

  move.facePoint(x, y, amaxSpeed, breakang, backward, dir);

}

void swingOnRight(float heading, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnRight(heading, amaxSpeed, breakang, dir);

}

void swingOnLeft(float heading, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnLeft(heading, amaxSpeed, breakang, dir);

}

void swingOnRightToPoint(float x, float y, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnRighttoPoint(x, y, amaxSpeed, breakang, backward, dir);

}

void swingOnLeftToPoint(float x, float y, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnLefttoPoint(x, y, amaxSpeed, breakang, backward, dir);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//    _____ _______ _____            _____ _____ _    _ _______ _      _____ _   _ _____ _   _  _____ 
//   / ____|__   __|  __ \     /\   |_   _/ ____| |  | |__   __| |    |_   _| \ | |_   _| \ | |/ ____|
//  | (___    | |  | |__) |   /  \    | || |  __| |__| |  | |  | |      | | |  \| | | | |  \| | |  __ 
//   \___ \   | |  |  _  /   / /\ \   | || | |_ |  __  |  | |  | |      | | | . ` | | | | . ` | | |_ |
//   ____) |  | |  | | \ \  / ____ \ _| || |__| | |  | |  | |  | |____ _| |_| |\  |_| |_| |\  | |__| |
//  |_____/   |_|  |_|  \_\/_/    \_\_____\_____|_|  |_|  |_|  |______|_____|_| \_|_____|_| \_|\_____|
/////////////////////////////////////////////////////////////////////////////////////////////////////////

float calculateStraightlineDist(float targetX, float targetY) {
  //Find distance using distance formula
  float x1 = chassis.x;
  float y1 = chassis.y;
  float dist = std::abs(std::sqrt((targetX-x1)*(targetX-x1)+(targetY-y1)*(targetY-y1)));

  return dist;
}

void straightTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed) {

  float distance = calculateStraightlineDist(x, y);
  float heading  = calculateTargetHeading(x, y, backward);

  if (backward) {
    if (bangbangJumpstart) {
    bangbangDrive(-distance * bangbangDist, 0.2, true);

    distance = calculateStraightlineDist(x, y);

    }
    
    move.straighth(-distance, heading, YKP, YKD, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);
  }

  if (not backward) {
    if (bangbangJumpstart) {
    bangbangDrive(distance * bangbangDist, 0.2, true);

    distance = calculateStraightlineDist(x, y);
    
    }

    move.straighth(distance, heading, YKP, YKD, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);
  }

}

void straightlineTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed, float ykp = YKP, float ykd = YKD) {

  facePoint(x, y, breakang, backward);
  straightTo(x, y, backward, breakdist, breakang, yspeed, hspeed);

}

void swingOnLeftAndStraightTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed, float ykp = YKP, float ykd = YKD) {

  swingOnLeftToPoint(x, y, breakang, backward);
  straightTo(x, y, backward, breakdist, breakang, yspeed, hspeed);

}

void swingOnRightAndStraightTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed, float ykp = YKP, float ykd = YKD) {

  swingOnRightToPoint(x, y, breakang, backward);
  straightTo(x, y, backward, breakdist, breakang, yspeed, hspeed);

}

////////////////////////////////////////////////////////////////////////////////////////////////
//    _____  ____ _______ ____    ______ _    _ _   _  _____ _______ _____ ____  _   _  _____ 
//   / ____|/ __ \__   __/ __ \  |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
//  | |  __| |  | | | | | |  | | | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___  
//  | | |_ | |  | | | | | |  | | |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \ 
//  | |__| | |__| | | | | |__| | | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |
//   \_____|\____/  |_|  \____/  |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/ 
////////////////////////////////////////////////////////////////////////////////////////////////
enum TurnType {spot, leftswing, rightswing, noturn};

void moveTo(float x, float y, bool backward = false, float breaklength = 8, int dir = 0, float turnmargin = 60, TurnType turnType = spot, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {
  if (backward) {
    move.bto(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin, 0, dir);
  }

  if (not backward) {
    if (turnType == spot) {
      move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin, 0, dir);
    }

    if (turnType == leftswing) {
      move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin, 1, dir);
    }

    if (turnType == rightswing) {
      move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin, 2, dir);
    }
  }

}

void goTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 3, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {

  facePoint(x, y, breakang, backward);
  moveTo(x, y, backward, breaklength, 0, breakang, spot, yspeed, hspeed, ykP, hkP);


}

void swingOnLeftAndGoTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 10, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {
  move.swingOnLefttoPoint(x, y, amaxSpeed, breakang, backward);

  if (backward) {
    move.bto(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

  if (not backward) {
    move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

}

void swingOnRightAndGoTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 10, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {
  move.swingOnRighttoPoint(x, y, amaxSpeed, breakang, backward);

  if (backward) {
    move.bto(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

  if (not backward) {
    move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

}

///////////////////////////////////////////////////////////////////////
//   ____   ____   ____  __  __ ______ _____            _   _  _____
//  |  _ \ / __ \ / __ \|  \/  |  ____|  __ \     /\   | \ | |/ ____|
//  | |_) | |  | | |  | | \  / | |__  | |__) |   /  \  |  \| | |  __
//  |  _ <| |  | | |  | | |\/| |  __| |  _  /   / /\ \ | . ` | | |_ |
//  | |_) | |__| | |__| | |  | | |____| | \ \  / ____ \| |\  | |__| |
//  |____/ \____/ \____/|_|  |_|______|_|  \_\/_/    \_\_| \_|\_____|
///////////////////////////////////////////////////////////////////////

void boomerangTo(
    float x, float y, float target_heading,
    bool backward = false,
    float breaklength = 3,
    float lead = 0.6, // Carrot point pullback = lead * distance + setback
    float setback = 5, //Setback is a minimum distance from the target point at which the robot will start to prioritize heading like crazy
    float min_voltage = 0, //For exiting at high speed and motion chaining
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3, //The robot switches to final pose correction when within 3 inches of the target. If it stays within 3 inches for 200ms AND heading is within 5 degrees, the movement exits.
    float timeout = 0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  move.boomerang(x, y, target_heading, backward,
                 drive_kp, drive_kd,
                 heading_kp, heading_kd,
                 drive_max, heading_max,
                 lead, min_voltage, settle_error,
                 setback, slew, timeout,
                 breaklength);
}

void boomerangToFacePoint(
    float x, float y, 
    float faceX, float faceY,
    bool backward = false,
    float breaklength = 3,
    float lead = 0.6, // Carrot point pullback = lead * distance + setback
    float setback = 0, //Setback is a minimum distance from the target point at which the robot will start to prioritize heading like crazy
    float min_voltage = 0, //For exiting at high speed and motion chaining
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3, //The robot switches to final pose correction when within 3 inches of the target. If it stays within 3 inches for 200ms AND heading is within 5 degrees, the movement exits.
    float timeout = 0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  float target_heading = calculateTargetHeading(faceX, faceY, backward, x, y);

  move.boomerang(x, y, target_heading, backward,
                 drive_kp, drive_kd,
                 heading_kp, heading_kd,
                 drive_max, heading_max,
                 lead, min_voltage, settle_error,
                 setback, slew, timeout,
                 breaklength);
}

void faceLeadAndBoomerangTo(
    float x, float y, float target_heading,
    bool backward = false,
    float breaklength = 3,
    float lead = 0.6, // Carrot point pullback = lead * distance + setback
    float setback = 5, //Setback is a minimum distance from the target point at which the robot will start to prioritize heading like crazy
    float min_voltage = 0, //For exiting at high speed and motion chaining
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3, //The robot switches to final pose correction when within 3 inches of the target. If it stays within 3 inches for 200ms AND heading is within 5 degrees, the movement exits.
    float timeout = 0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD,
    float face_breakang = 12
) {

  // Compute the initial carrot point (behind target along target heading)
  float dx_target = x - chassis.x;
  float dy_target = y - chassis.y;
  float target_distance = sqrt(dx_target * dx_target + dy_target * dy_target);
  float pullback = lead * target_distance + setback;
  float target_angle_rad = TO_RAD(target_heading);
  float carrot_x = x - sin(target_angle_rad) * pullback;
  float carrot_y = y - cos(target_angle_rad) * pullback;

  // Spot turn to face the initial lead point of the boomerang
  float face_heading = calculateTargetHeading(carrot_x, carrot_y, backward);
  face(face_heading, face_breakang);

  move.boomerang(x, y, target_heading, backward,
                 drive_kp, drive_kd,
                 heading_kp, heading_kd,
                 drive_max, heading_max,
                 lead, min_voltage, settle_error,
                 setback, slew, timeout,
                 breaklength);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//   _____ _   _  _____ ______ _____ _______ _____ ____  _   _   ____   ____   ____  __  __
//  |_   _| \ | |/ ____|  ____|  __ \__   __|_   _/ __ \| \ | | |  _ \ / __ \ / __ \|  \/  |
//    | | |  \| | |    | |__  | |__) | | |    | || |  | |  \| | | |_) | |  | | |  | | \  / |
//    | | | . ` | |    |  __| |  ___/  | |    | || |  | | . ` | |  _ <| |  | | |  | | |\/| |
//   _| |_| |\  | |____| |____| |      | |   _| || |__| | |\  | | |_) | |__| | |__| | |  | |
//  |_____|_| \_|\_____|______|_|      |_|  |_____\____/|_| \_| |____/ \____/ \____/|_|  |_|
///////////////////////////////////////////////////////////////////////////////////////////////////

void _2029BoomerangTo(
    float x, float y, float target_heading,
    bool backward = false,
    float exit_dist = 3,
    float dLead = 15.0,
    float gLead = 0.4,
    float close_ghost_dist = 20.0,
    float min_voltage = 0,
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3,
    float timeout = 0,
    float close_end_dist = 8.0,
    float parallel_dist = 8.0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  move.inceptionBoomerang(x, y, target_heading, backward,
                          drive_kp, drive_kd,
                          heading_kp, heading_kd,
                          drive_max, heading_max,
                          dLead, gLead,
                          min_voltage, settle_error,
                          slew, timeout,
                          close_end_dist, close_ghost_dist,
                          parallel_dist, exit_dist);
}

void _2029BoomerangToFacePoint(
    float x, float y,
    float faceX, float faceY,
    bool backward = false,
    float exit_dist = 3,
    float dLead = 15.0,
    float gLead = 0.4,
    float min_voltage = 0,
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3,
    float timeout = 0,
    float close_end_dist = 8.0,
    float close_ghost_dist = 20.0,
    float parallel_dist = 8.0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  float target_heading = calculateTargetHeading(faceX, faceY, backward, x, y);

  move.inceptionBoomerang(x, y, target_heading, backward,
                          drive_kp, drive_kd,
                          heading_kp, heading_kd,
                          drive_max, heading_max,
                          dLead, gLead,
                          min_voltage, settle_error,
                          slew, timeout,
                          close_end_dist, close_ghost_dist,
                          parallel_dist, exit_dist);
}

void faceLeadAnd2029BoomerangTo(
    float x, float y, float target_heading,
    bool backward = false,
    float exit_dist = 3,
    float dLead = 15.0,
    float gLead = 0.4,
    float close_ghost_dist = 20.0,
    float min_voltage = 0,
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3,
    float timeout = 0,
    float close_end_dist = 8.0,
    float parallel_dist = 8.0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD,
    float face_breakang = 8
) {

  // Compute the initial carrot point (behind target along target heading)
  float target_angle_rad = TO_RAD(target_heading);
  float carrot_x = x - sin(target_angle_rad) * dLead;
  float carrot_y = y - cos(target_angle_rad) * dLead;

  // Spot turn to face the initial lead point of the boomerang
  float face_heading = calculateTargetHeading(carrot_x, carrot_y, backward);
  face(face_heading, face_breakang);

  move.inceptionBoomerang(x, y, target_heading, backward,
                          drive_kp, drive_kd,
                          heading_kp, heading_kd,
                          drive_max, heading_max,
                          dLead, gLead,
                          min_voltage, settle_error,
                          slew, timeout,
                          close_end_dist, close_ghost_dist,
                          parallel_dist, exit_dist);
}

///////////////////////////////////////////////////////////////////
//    _____ _    _ _______      ________   _______ ____
//   / ____| |  | |  __ \ \    / /  ____| |__   __/ __ \
//  | |    | |  | | |__) \ \  / /| |__       | | | |  | |
//  | |    | |  | |  _  / \ \/ / |  __|      | | | |  | |
//  | |____| |__| | | \ \  \  /  | |____     | | | |__| |
//   \_____|\____/|_|  \_\  \/   |______|    |_|  \____/
///////////////////////////////////////////////////////////////////

void curveTo(float x1, float y1, float x2, float y2, float lookDist = 12, float breakLength = 5, TurnType turnType = noturn, float breakang = 8, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykp = YKP, float hkp = HKP) {
  // Compute the true Bezier control point: cp = p1 * 2 - (p0 + p2) / 2
  // where p0 = current position, p1 = pass-through point, p2 = end point
  float cpX = x1 * 2 - (chassis.x + x2) / 2;
  float cpY = y1 * 2 - (chassis.y + y2) / 2;

  // Face the true control point first based on turnType
  if (turnType == spot) {
    facePoint(cpX, cpY, breakang);
  } else if (turnType == leftswing) {
    swingOnLeftToPoint(cpX, cpY, breakang);
  } else if (turnType == rightswing) {
    swingOnRightToPoint(cpX, cpY, breakang);
  }

  move.CurveTo(x1, y1, x2, y2, yspeed, hspeed, ykp, hkp, slew, lookDist, breakLength);
}

////////////////////////////////////////////////////////////////////////////////////
//    _______          ________ ______ _____    _______ _    _ _____  _   _  _____ 
//   / ____\ \        / /  ____|  ____|  __ \  |__   __| |  | |  __ \| \ | |/ ____|
//  | (___  \ \  /\  / /| |__  | |__  | |__) |    | |  | |  | | |__) |  \| | (___  
//   \___ \  \ \/  \/ / |  __| |  __| |  ___/     | |  | |  | |  _  /| . ` |\___ \ 
//   ____) |  \  /\  /  | |____| |____| |         | |  | |__| | | \ \| |\  |____) |
//  |_____/    \/  \/   |______|______|_|         |_|   \____/|_|  \_\_| \_|_____/ 
////////////////////////////////////////////////////////////////////////////////////                                                                  

void sweepOnLeft(float heading, float sweepRatio, float breakang = 8) {
  sweep_ratio = sweepRatio;
  move.swingOnLeft(heading, amaxSpeed, breakang);
  sweep_ratio = 0;
}

void sweepOnRight(float heading, float sweepRatio, float breakang = 8) {
  sweep_ratio = sweepRatio;
  move.swingOnRight(heading, amaxSpeed, breakang);
  sweep_ratio = 0;
}

void sweepOnLeftToPoint(float x, float y, float sweepRatio, float breakang = 8, bool backward = false) {
  sweep_ratio = sweepRatio;
  move.swingOnLefttoPoint(x, y, amaxSpeed, breakang, backward);
  sweep_ratio = 0;
}

void sweepOnRightToPoint(float x, float y, float sweepRatio, float breakang = 8, bool backward = false) {
  sweep_ratio = sweepRatio;
  move.swingOnRighttoPoint(x, y, amaxSpeed, breakang, backward);
  sweep_ratio = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
//   _____        _____ _______ _____          _        _______ _    _ _____  _   _ 
//  |  __ \ /\   |  __ \__   __|_   _|   /\   | |      |__   __| |  | |  __ \| \ | |
//  | |__) /  \  | |__) | | |    | |    /  \  | |         | |  | |  | | |__) |  \| |
//  |  ___/ /\ \ |  _  /  | |    | |   / /\ \ | |         | |  | |  | |  _  /| . ` |
//  | |  / ____ \| | \ \  | |   _| |_ / ____ \| |____     | |  | |__| | | \ \| |\  |
//  |_| /_/    \_\_|  \_\ |_|  |_____/_/    \_\______|    |_|   \____/|_|  \_\_| \_|
/////////////////////////////////////////////////////////////////////////////////////////  
enum partialSide {
  onLeft,
  onRight
};

float partialOutput = 0;
float partialOutputSide = 0;

void partialTurn(partialSide side, float partial, float heading, float breakang = 8, float dir = 0) {
  partialOutput = partial;
  if (side == onLeft) {
    partialOutputSide = 1;
  } else {
    partialOutputSide = 2;
  }
  
  face(heading, breakang, dir);

  partialOutput = 0;
  partialOutputSide = 0;
}

void partialTurnToPoint(partialSide side, float partial, float x, float y, float breakang = 8, bool backward = false, int dir = 0) {
  partialOutput = partial;
  if (side == onLeft) {
    partialOutputSide = 1;
  } else {
    partialOutputSide = 2;
  }

  facePoint(x, y, breakang, backward, dir);

  partialOutput = 0;
  partialOutputSide = 0;
}
                                            


/////////////////////////////////////
//            _____   _____  _____ 
//      /\   |  __ \ / ____|/ ____|
//     /  \  | |__) | |    | (___  
//    / /\ \ |  _  /| |     \___ \ 
//   / ____ \| | \ \| |____ ____) |
//  /_/    \_\_|  \_\\_____|_____/ 
/////////////////////////////////////

void arcTo(float x1, float y1, float x2, float y2, float speed = ymaxSpeed, float breakLength = 5, bool backward = false, float arcCtKP = ARC_CT_KP, float arcHdgKP = ARC_HDG_KP, float alignMargin = 8) {
  move.arc(x1, y1, x2, y2, speed, breakLength, arcCtKP, arcHdgKP, alignMargin, backward, amaxSpeed);
}

/////////////////////////////////////
//   _____  _    _  _____ _    _ 
//  |  __ \| |  | |/ ____| |  | |
//  | |__) | |  | | (___ | |__| |
//  |  ___/| |  | |\___ \|  __  |
//  | |    | |__| |____) | |  | |
//  |_|     \____/|_____/|_|  |_|
/////////////////////////////////////

void push(float volts, float slope, float pitchAmount) {
  while (IMU1.orientation(pitch, degrees) < pitchAmount) {
    move.voldrive(volts,volts);
    volts += slope;
    delay(10);
  }
}



///////////////////////////////////////////////////////
//   _____ _____ _____    __  __  ____  _____   _____
//  |  __ \_   _|  __ \  |  \/  |/ __ \|  __ \ / ____|
//  | |__) || | | |  | | | \  / | |  | | |  | | (___
//  |  ___/ | | | |  | | | |\/| | |  | | |  | |\___ \
//  | |    _| |_| |__| | | |  | | |__| | |__| |____) |
//  |_|   |_____|_____/  |_|  |_|\____/|_____/|_____/
///////////////////////////////////////////////////////

float oldHKP = HKP;
float oldHKD = HKD;

float oldYKP = YKP;
float oldYKI = YKI;
float oldYKD = YKD;

float oldARC_CT_KP = ARC_CT_KP;
float oldARC_HDG_KP = ARC_HDG_KP;

void modifyAngularPID(float newTKP, float newTKD) {
  turnPID.scalePID = false;
  turnPID.setFallbackConstants(newTKP, newTKD);
}

void modifySwingPID(float newSKP, float newSKD) {
  swingPID.scalePID = false;
  swingPID.setFallbackConstants(newSKP, newSKD);
}

void modifyHeadingPID(float newHKP, float newHKD) {
  HKP = newHKP;
  HKD = newHKD;
}

void modifyLateralPID(float newYKP, float newYKD) {
  YKP = newYKP;
  YKD = newYKD;
}

void modifyArcGains(float newCtKP, float newHdgKP) {
  ARC_CT_KP = newCtKP;
  ARC_HDG_KP = newHdgKP;
}

void revertToOriginalPIDs() {
  turnPID.scalePID = true;
  swingPID.scalePID = true;

  HKP = oldHKP;
  HKD = oldHKD;

  YKP = oldYKP;
  YKI = oldYKI;
  YKD = oldYKD;

  ARC_CT_KP = oldARC_CT_KP;
  ARC_HDG_KP = oldARC_HDG_KP;
}

/////////////////////////////////////////////////////////////////////////////
//           _    _ _______ ____  _   _  _____   _    _ ______ _____  ______ 
//      /\  | |  | |__   __/ __ \| \ | |/ ____| | |  | |  ____|  __ \|  ____|
//     /  \ | |  | |  | | | |  | |  \| | (___   | |__| | |__  | |__) | |__   
//    / /\ \| |  | |  | | | |  | | . ` |\___ \  |  __  |  __| |  _  /|  __|  
//   / ____ \ |__| |  | | | |__| | |\  |____) | | |  | | |____| | \ \| |____ 
//  /_/    \_\____/   |_|  \____/|_| \_|_____/  |_|  |_|______|_|  \_\______|
//
/////////////////////////////////////////////////////////////////////////////

//Delayed matchloader drop
bool runDelayedDrop = false;

double dropMatchloaderIn = 1000;
double dropMatchloaderFor = 0;

void delayedDrop(double dropIn, double dropFor = 0) {
  dropMatchloaderIn = dropIn;
  dropMatchloaderFor = dropFor;
  runDelayedDrop = true;
}

bool runTwoDelayedDrops = false;

double dropMatchloaderIn1 = 0;
double dropMatchloaderFor1 = 0;

double dropMatchloaderIn2 = 0;
double dropMatchloaderFor2 = 0;

void twoDelayedDrops(double dropIn1, double dropFor1, double dropIn2, double dropFor2) {
  dropMatchloaderIn1 = dropIn1;
  dropMatchloaderIn2 = dropIn2;

  dropMatchloaderFor1 = dropFor1;
  dropMatchloaderFor2 = dropFor2;
  runTwoDelayedDrops = true;
}

double dropX1 = 0, dropY1 = 0, dropDist1 = 0, dropTime1 = 0;
double dropX2 = 0, dropY2 = 0, dropDist2 = 0, dropTime2 = 0;
bool drop1Active = false, drop2Active = false;

void distanceToPointDrop(double x1, double y1, double dist1, double time1, double x2 = 0, double y2 = 0, double dist2 = 0, double time2 = 0) {
  dropX1 = x1; dropY1 = y1; dropDist1 = dist1; dropTime1 = time1;
  drop1Active = true;
  if (dist2 > 0) {
    dropX2 = x2; dropY2 = y2; dropDist2 = dist2; dropTime2 = time2;
    drop2Active = true;
  }
}

int delayedMatchloaderDropFcn() {
  while (true) {
    if (runDelayedDrop) {
      delay(dropMatchloaderIn);
      matchloaderDown();
      if (dropMatchloaderFor > 0) {
        delay(dropMatchloaderFor);
        matchloaderUp();
        dropMatchloaderFor = 0;
      }
    runDelayedDrop = false; 
    }

    // if (drop1Active) {
    //   double dist = sqrt(pow(chassis.x - dropX1, 2) + pow(chassis.y - dropY1, 2));
    //   if (dist <= dropDist1) {
    //     matchloaderDown();
    //     if (dropTime1 > 0) { delay(dropTime1); matchloaderUp(); }
    //     drop1Active = false;
    //   }
    // }

    // if (drop2Active) {
    //   double dist = sqrt(pow(chassis.x - dropX2, 2) + pow(chassis.y - dropY2, 2));
    //   if (dist <= dropDist2) {
    //     matchloaderDown();
    //     if (dropTime2 > 0) { delay(dropTime2); matchloaderUp(); }
    //     drop2Active = false;
    //   }
    // }

    if (runTwoDelayedDrops) {
      delay(dropMatchloaderIn1);
      matchloaderDown();
      delay(dropMatchloaderFor1);
      matchloaderUp();
      delay(dropMatchloaderIn2);
      matchloaderDown();
      if (dropMatchloaderFor2 != 0) {
        delay(dropMatchloaderFor2);
        matchloaderUp();
      }
      runTwoDelayedDrops = false;
    }

    vex::this_thread::sleep_for(10);
  }
  return(0);
}

//CONSTANTS ////////////////////////////////////////////////////////////////
double longGoalX = 48;
double longGoalY = 30;

double trueLongGoalX = 48;
double trueLongGoalY = 24.240;

double matchloaderX = 47;
double matchloaderY = 60;

double trueMatchloaderY = 53.4;

double quad;

void scoreLong(double quadrant, double time2, double time1 = 270) {
  double flipX = 1;
  double flipY = 1;
  double targH = 180;
  if (quadrant == 1) {
    flipX = 1;
    flipY = 1;
  } else if (quadrant == 2) {
    flipX = -1;
    flipY = 1;
  } else if (quadrant == 3) {
    flipX = -1;
    flipY = -1;
  } else if (quadrant == 4) {
    flipX = 1;
    flipY = -1;
  }

  if (quadrant == 1 || quadrant == 2) {
    targH = 0;
  } else if (quadrant == 3 || quadrant == 4) {
    targH = 180;
  }


  drivePID.minOutput = 8;
  faceLeadAndBoomerangTo(flipX*longGoalX, flipY*trueLongGoalY, targH, true, 8, 0.8, 0);
  ymaxSpeed = 8;
  driveBackward(15, targH, 0);
  delay(time1);
  drivePID.minOutput = 0;
  hoodUp();
  intakeMode = longGoal;
  face(targH, 0);
  delay(time2*0.2);
  drivePID.minOutput = 12.7;
  driveBackward(20, targH, 0);
  delay(time2*0.8);
  ymaxSpeed = 12.7;
  chassis.setPos(flipX*trueLongGoalX, flipY*trueLongGoalY, chassis.h);

  drivePID.minOutput = 0;

}

void HookProcedure() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.seamlessTransitions = true;

  chassis.setPos(0,0,180);

  float originalHeading = chassis.h;
  float originalX = chassis.x;
  float originalY = chassis.y;

  // // modifyAngularPID(_90DegTKP * 1.5, _90DegTKD * 0.2);
  // move.bigTime = 20;
  // face(-120, 20);
  // revertToOriginalPIDs();
  // turnPID.minOutput = 0;
  hookDown();
  driveForward(14, -120);

  // drivePID.minOutput = 8;
  // driveForward(6);
  drivePID.minOutput = 0;
  ymaxSpeed = 7;
  driveBackward(25, -170, 8);
  leftdrive.stop(brake);
  rightdrive.stop(brake);

  // swingPID.minOutput = 12.7;
  // turnPID.minOutput = 10;
  // partialTurn(onRight, 0.1, originalHeading+65, 0);
  // delay(500);
  // turnPID.minOutput = 0;
  // // swingOnRight(originalHeading+77, 5);
  // swingPID.minOutput = 0;
  // revertToOriginalPIDs();
  // face(originalHeading);
}

void southpawHook() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;

  chassis.setPos(0,0,0);

  float originalHeading = chassis.h;

  bangbangDrive(1, 0);
  swingOnLeft(originalHeading+150, 20, false, 2);
  // face(originalHeading+160, 10);
}

bool nextmovement = false;

void setnextmovementtrue(vex::axisType axis, double x, double y, double z) {
  nextmovement = true;
}

void arpit(bool endEarly, int flip) {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;
  move.seamlessTransitions = true;

  midDescoreDown();

  hookDown();

  dropMatchloaderIn = 450;
  dropMatchloaderFor = 0;
  runDelayedDrop = true;

  chassis.setPos(13, -45.82,0);

  if (flip == 1) {
    readWall(1, 1,true);
    readWall(4,4, true);
  }

  if (flip == 3) {
    readWall(3, 3, true);
    readWall(4,4, true);
  }

  moveTo(flip*27, -25, false, 10, 0);

  facePoint(flip*48, -27, 2, true);

  nextmovement = false;
  IMU1.collision(setnextmovementtrue);
  drivePID.minOutput = 8;
  drivePID.maxOutput = 8;
  straightTo(flip*48, -27, true, 0);
  while (not nextmovement) {
    delay(10);
  }
  drivePID.minOutput = 0;
  drivePID.maxOutput = 12.7;

  modifyAngularPID(YKP*10000000, YKD);
  if(flip == -1){
    sweepOnLeft(180, 0.2, 0);
  } else if(flip == 1){
    sweepOnRight(180, 0.2, 0);
  }

  revertToOriginalPIDs();
  delay(500);
  move.voldrive(-8, -8);
  intakeMode = longGoal;
  delay(300);
  face(180,0);
  delay(300);
  move.voldrive(-8,-8);
  delay(700);
  chassis.setPos(flip * trueLongGoalX, trueLongGoalY, chassis.h);

   if(endEarly) {
    return;
   }

   HookProcedure();
   driveBackward(25);
   leftdrive.stop(hold);
   rightdrive.stop(hold);

}

void tenB(float flip, bool exitEarly = false, bool exitAtGoal = false, bool endLower = false) {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;

  intakeMode = intake;
  midDescoreDown();

  hookUp();

  chassis.setPos(flip*13, -45.82,0);

  if (flip == 1) {
    readWall(1, 1, true);
    readWall(4, 4, true);
  } 

  if (flip == -1) {
    readWall(3, 3,true);
    readWall(4,4, true);
  }

  if (flip == 1) {
    twoDelayedDrops(450, 250, 520, 0);
    moveTo(22, -24, false, 12, 0, 60);
  }

  if (flip == -1) {
    twoDelayedDrops(450, 250, 600, 0);
    moveTo(-26, -24, false, 12, 0, 60);
  }

  ymaxSpeed = 7;
  move.turndisablelength = 3;
  if (flip == 1) {
    moveTo(44, -1.5, false, 0);
    delay(900);
  }

  if (flip == -1) {
    moveTo(-42, -6, false, 0);
    delay(800);
  }

  
  move.turndisablelength = 6;

  ymaxSpeed = 12.7;
  face(flip*90);
  
  if (exitAtGoal) {
    return;
  }

  //Back up
  if (endLower) {
    if (flip == 1) {
      _2029BoomerangTo(53, -39, flip*90, true,8, 32, 0, 12);
    }
    if (flip == -1) {
      _2029BoomerangTo(-56, -39, flip*90, true,8, 32, 0, 12);
    }
  }

  if (not endLower) {
    _2029BoomerangTo(flip*56, -32, flip*90, true,8, 32, 0, 12);
  }

  face(180,3);
  delay(50);

  if (flip == 1) {
    readWall(1, 3);
  }

  if (flip == -1) {
    readWall(3, 1);
  }

  if (exitEarly) {
    return;
  }

}

void tenBLeft() {
  tenB(-1);
}

void tenBRight() {
  tenB(1);
}


void sevenRush(double flip, bool exitAtLoader = false) {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;
  move.seamlessTransitions = true;
  bangbangJumpstart = false;

  intakeMode = intake;
  midDescoreDown();
  hookUp();

  chassis.setPos(-13, flip*45.82,0);

  if (flip == 1) {
    readWall(1, 1,true);
    readWall(4,4, true);

  }

  if (flip == -1) {
    readWall(3, 3,true);
    readWall(4,4, true);
  }

  delayedDrop(400, 0);

  moveTo(flip*26, -22, false, 12, 0, 60);

  moveTo(flip*45, -45, true, 12);

  ymaxSpeed = 5;
  straightlineTo(flip*(matchloaderX+1), -matchloaderY, false, 15);
  ymaxSpeed = 12.7;

  // //To matchloader
  // ymaxSpeed = 12.7;
  // modifyAngularPID(_180DegTKP*1.5, _180DegTKD);
  // if (flip == -1) {
  //   partialTurnToPoint(onLeft, 0.15, flip*matchloaderX, -trueMatchloaderY, 5, false);
  //   revertToOriginalPIDs();
  //   // moveTo(flip*(matchloaderX-2), -trueMatchloaderY+5, false, 12);
  // }

  // if (flip == 1) {
  //   partialTurnToPoint(onRight, 0.15, flip*matchloaderX, -trueMatchloaderY, 5, false);
  //   revertToOriginalPIDs();
  //   // moveTo(flip*(matchloaderX-2), -trueMatchloaderY+5, false, 12);
  // }
  // boomerangTo(flip*matchloaderX, -trueMatchloaderY, flip*170, false, 5, 0.3, 0);
  // move.voldrive(0,0);
  // move.voldrive(4,4);

  // if (flip == -1) {
  //   sweepOnLeft(180, 0.2, 0);
  // } 

  // if (flip == 1) {
  //   sweepOnRight(180, 0.2, 0);
  // }
  swingPID.minOutput = 0;

  delay(500);

  ymaxSpeed = 12.7;

  if (flip == 1) {
    quad = 4;
  }

  if (flip == -1) {
    quad = 3;
  }

  if (exitAtLoader) {
    return;
  }
  matchloaderUp();

  scoreLong(quad, 1400);

  hookDown();
  HookProcedure();

  leftdrive.stop(hold);
  rightdrive.stop(hold);
  delay(50000000);

}

void sevenRushRight() {
  sevenRush(1);
}

void sevenRushLeft() {
  sevenRush(-1);
}

void leftSplit(int blocks) {
  if (blocks == 9) {
    tenB(-1, false);
    scoreLong(3, 800, 400);
    intakeMode = antiSpill;
  } else if (blocks == 7) {
    arpit(true, -1);
  }

  chassis.setPos(-48, -28.982, chassis.h);
  straightlineTo(-46, -67, false, 0, 5, 5.5);
  intakeMode = antiSpill;
  delay(300);
  hoodDown();
  delay(550);
  intakeMode = intake;
  delay(650);
  intakeMode = antiSpill;
  delay(100);

  ymaxSpeed = 12.7;
  chassis.setPos(chassis.x, -trueMatchloaderY, chassis.h);
  // sweepOnLeftToPoint(-20, -20, 0.3, 8, true);
  bangbangDrive(-2, 0);
  drivePID.minOutput = 4;
  drop4bar();
  _2029BoomerangTo(-6, -12, -145, true, 12, 30, 1);
  intakeMode = midGoalSlow;
  move.voldrive(-2, -2);
  delay(200);
  drop4bar();
  // hoodUp();
  slowMidSpeed = 50;
  delay(1500);
  ymaxSpeed = 5;
  driveForward(8);
  raise4bar();
  intakeMode = antiSpill;
  driveBackward(8);
  delay(5000000);

  // ymaxSpeed = 6;
  // straightlineTo(-matchloaderX, -matchloaderY, false, 8);
  // ymaxSpeed = 12.7;
  // hoodDown();
  // delay(700);

  // intakeMode = intake;
  // ymaxSpeed = 12.7;

  // bangbangDrive(-1, 0);
  // _2029BoomerangTo(-8, -8, -135, true, 8);
  // drop4bar();
  // intakeMode = antiSpill;
  // move.voldrive(-2,-2);
  // delay(250);
  // slowMidSpeed = 30;
  // intakeMode = midGoalSlow;
  // delay(500);
  // if (sawBadColor) {
  //   intakeMode = spit;
  // }
  // // driveForward(12);
  // // raise4bar();
  // // driveBackward(12);
}

void leftMidOnly(int blocks) {
  sevenRush(-1);

  bangbangDrive(-1.5,0);  
  _2029BoomerangTo(-11, -11, -135, true, 8);
  drop4bar();
  move.voldrive(-2,-2);
  delay(200);
  intakeMode = midGoal;

}

void rightSplit(int blocks) {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;

  tenB(1);
  scoreLong(4, 1000, 350);

  /*if (blocks == 9) {
    // tenB(1);
    // scoreLong(4, 1200);
  } else if (blocks == 7) {
    arpit(true, 1);
  } */

  chassis.setPos(trueLongGoalX, -trueLongGoalY, 180);
  matchloaderDown();
  intakeMode = intake;
  ymaxSpeed = 12.7;
  revertToOriginalPIDs();
  ymaxSpeed = 5;
  straightTo(matchloaderX, -matchloaderY, false, 15);
  hoodDown(); 
  delay(700);
  chassis.setPos(chassis.x, -trueMatchloaderY, chassis.h);

  intakeMode = intake;
  ymaxSpeed = 12.7;


  ymaxSpeed = 12.7;
  bangbangDrive(-2, 0);
  matchloaderUp();
  // facePoint(0,0);
  moveTo(20, -18, true, 8);
  modifyAngularPID(_180DegTKP*1.5, _180DegTKD*0.5);
  facePoint(-1,0, 3);
  revertToOriginalPIDs();
  straightTo(13, -13, false, 5);
  // rightdrive.stop(brake);
  // driveBackward(4);
  // move.voldrive(0,0);
  midDescoreUp();
  lowGoalSpeed = 70;
  intakeMode = lowGoal;
  ymaxSpeed = 3;
  //driveBackward(10, 45, 0);
  delay(1200);
  ymaxSpeed = 12.7;
  modifyLateralPID(YKP*5, 0);
  moveTo(33, -25, true,11);
  hookDown();
  swingOnRight(180, 12);
  driveBackward(6, 180);
  move.voldrive(0,0);
  delay(500000);
}


void SOLOAWP() {
auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;
  move.seamlessTransitions = true;
  antiJam = true;

  hookUp();

  chassis.setPos(0, 0, 270);
  //readWall(1, 4, true);
  readWall(4, 3, true);
  intakeMode = intake;
  modifyLateralPID(YKP, 0);
  driveForward(8, chassis.h, 0);
  delay(500);
  revertToOriginalPIDs();
  //voltagedrive(3, 3);
  drivePID.minOutput = 12.7;
  straightTo(48, -48, true, 0);
  delay(250);
  drivePID.minOutput = 0;
  modifyLateralPID(YKP*0.8, YKD*1.3);
  straightTo(47, -48, true, 3);
  face(180);
  delay(80);
  readWall(1, 3, false);
  revertToOriginalPIDs();
  matchloaderDown();

  straightlineTo(47.6, -67, false, 0, 5, 5.2);
  delay(950);
  
  ymaxSpeed = 9;
  boomerangTo(48, -24, 180, true, 12);
  intakeMode = topStageJamInsurance;
  matchloaderUp();
  move.voldrive(-7, -7);
  intakeMode = longGoal;
  delay(50);
  hoodUp();
  delay(400);
  face(180, 0);
  delay(100);
  move.voldrive(-7, -7);
  delay(600);
  revertToOriginalPIDs();

  ymaxSpeed = 12.7;

  chassis.setPos(48, -28.982, chassis.h);
  intakeMode = intake;
  //swingOnRightToPoint(-20, -24, 5, false);
  move.voldrive(0, 0);
  //sweepOnRight(-70, 0.5);
  face(-50);
  //facePoint(8, -25);
  ymaxSpeed = 12.7;
  hoodDown();
  twoDelayedDrops(150, 350, 650, 0);
  driveForward(7, chassis.h, 5);
  _2029BoomerangTo(-19, -28, -90, false, 15, 32, 0.8);
  revertToOriginalPIDs();
  delay(200);
  intakeMode = topStageJamInsurance;
  modifyAngularPID(_90DegTKP*1.5, _90DegTKD*0.5);
  facePoint(-46, -45, 11);
  modifyLateralPID(0.8*YKP, 1.3*YKD);
  moveTo(-47, -45, false, 5, 20);
  //_2029BoomerangTo(-46, -47, 180, false, 3, 13, 1);
  revertToOriginalPIDs();
  delay(100);
  face(180);
  readWall(3, 1);
  ymaxSpeed = 8;
  moveTo(-48, -28, true, 10, 1, 20, spot);
  intakeMode = longGoal;
  hoodUp();
  move.voldrive(-7, -7);
  delay(100);
  face(180, 0);
  intakeMode = longGoal;
  matchloaderDown();
  delay(50);
  move.voldrive(-7, -7);
  delay(480);
  intakeMode = antiSpill;
  delay(100);
  chassis.setPos(-48, -28.982, chassis.h);
  straightlineTo(-46, -67, false, 0, 5, 5.5);
  delay(600);
  hoodDown();
  delay(250);
  intakeMode = intake;
  delay(700);
  intakeMode = antiSpill;
  delay(200);

  ymaxSpeed = 12.7;
  chassis.setPos(chassis.x, -trueMatchloaderY, chassis.h);
  // sweepOnLeftToPoint(-20, -20, 0.3, 8, true);
  bangbangDrive(-2, 0);
  drivePID.minOutput = 4;
  drop4bar();
  _2029BoomerangTo(-6, -9, -142, true, 12, 30, 1);
  slowMidSpeed = 30;
  intakeMode = midGoalSlow;
  move.voldrive(-2, -2);
  delay(250);
  drop4bar();
  // hoodUp();
  delay(5000000);
}


void SOLO() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeLag = false;


  intakeMode = intake;

  hookUp();

  chassis.setPos(0,0,-90);
  readWall(4, 3, true);
  
  driveForward(8);

  intakeMode = intake;
  straightTo(matchloaderX-4, chassis.y, true, 2);
  matchloaderDown();

  face(180, 3);
  readWall(1, 3);

  straightlineTo(matchloaderX, -matchloaderY, false, 12, 5, 6);
  delay(900);
  matchloaderUp();

  scoreLong(4, 800, 50);

  chassis.setPos(trueLongGoalX, -trueLongGoalY,chassis.h);

  face(-70);
  hoodDown();
  ymaxSpeed = 12.7;
  twoDelayedDrops(200, 300, 500, 0);
  _2029BoomerangTo(-20, -24, -90, false, 15, 30, 1);
  goTo(-45, -40, false, 2);

  face(180, 3);
  delay(100);
  readWall(3, 1);

  facePoint(-48, -28, 5, true);
  scoreLong(3,1000, 50);
  hoodDown();
  
  intakeMode = intake;
  ymaxSpeed = 12.7;
  straightlineTo(-matchloaderX, -matchloaderY, false, 0);
  delay(250);
  ymaxSpeed = 6;
  straightTo(-matchloaderX, -matchloaderY, false, 12);
  hoodDown();
  delay(600);

  intakeMode = intake;
  ymaxSpeed = 10;
  bangbangDrive(-1, 0);
  move.seamlessTransitions = true;

  _2029BoomerangTo(-11, -11, -135, true, 8, 48, 0);
  drop4bar();
  intakeMode = coasting;
  move.voldrive(-4,-4);
  delay(100);
  intakeMode = midGoalSlow;
  delay(1000000);
  // driveForward(12);
  // midDescoreUp();
  // driveBackward(12);


}

void leftCounter() {
  tenB(-1);

  intakeMode = intake;
  ymaxSpeed = 7.5;
  straightlineTo(-matchloaderX, -matchloaderY, false, 8);
  ymaxSpeed = 12.7;
  hoodDown();
  delay(500);

  intakeMode = intake;
  ymaxSpeed = 12.7;

  bangbangDrive(-1, 0);
  _2029BoomerangTo(-11, -11, -135, true, 8);
  drop4bar();
  intakeMode = coasting;
  move.voldrive(-2,-2);
  delay(250);
  intakeMode = midGoalSlow;
  delay(500);
  if (sawBadColor) {
    intakeMode = spit;
  }



}

void twoMidGoals() {
  sevenRush(-1, true);
  ymaxSpeed = 12.7;
  // intakeMode = antiSpill;
  intakeMode = antiSpill;
  bangbangDrive(-1,0);
  _2029BoomerangTo(-8, -4, -135, true, 10);
  intakeMode = antiSpill;
  move.voldrive(-1,-1);
  drop4bar();
  delay(100);
  slowMidSpeed = 50;
  intakeMode = midGoalSlow;
  delay(800);
  intakeMode = antiSpill;
  delay(200);

  matchloaderUp();

  bangbangDrive(2, 0);
  modifyAngularPID(_90DegTKD*2, _90DegTKD*0.5);
  partialTurnToPoint(onLeft, 0.35, -3, -18, 8);
  raise4bar();
  revertToOriginalPIDs();

  drivePID.minOutput = 8;
  straightTo(-3, -18);

  midDescoreUp();
  // swingPID.minOutput = 8;
  swingOnLeft(-45, 0);
  delay(800);
  // partialTurnToPoint(onLeft, 0.2, 0, 0, 15);
  drivePID.minOutput = 12.7;
  midDescoreUp();
  driveForward(20, -45, 0);
  delay(300);
  extendMidGoalHook();
  delay(300);
  drivePID.minOutput = 0;
  ymaxSpeed = 4;
  driveBackward(5,chassis.h, 0);
  lowGoalSpeed = 50;
  intakeMode = lowGoal;
  delay(500000000);
}

void sai_insane_auto(){
  printingToConsole = true;
  tenB(-1, false, true);
  scoreLong(3, 1200);
  intakeMode = antiSpill;
  _2029BoomerangTo(-11, -3, 45, true,
  8, 10, 0);
  // faceLeadAnd2029BoomerangTo(-11, -3, 45
  // brain_screen_print = ("hi my name's jeff")
  // chassis.turn_to_angle(45);
  // console.log("hi my name's jeff how yo doing lucasssssssssssssssssssssss");
}
void partnerCamp() {
  sevenRush(1, true);
  chassis.setPos(chassis.x, -trueMatchloaderY, chassis.h);
  drivePID.minOutput = 0;
  bangbangDrive(-4, 0);
  matchloaderUp();
  ymaxSpeed = 12.7;
  faceLeadAndBoomerangTo(15, -10, -45, false, 8);
  driveForward(20, -45, 0);
  midDescoreUp();
  lowGoalSpeed = 25;
  intakeMode = lowGoal;
  // goTo(13, -13);
  delay(5000000000);

}

void nineBallTwoMidGoals() {
  tenB(-1, true, false, true);
  ymaxSpeed = 5;
  straightlineTo(-matchloaderX, -matchloaderY, false, 0);
  // faceLeadAndBoomerangTo(-matchloaderX, -matchloaderY, 180, false, 12, 0.4, 0);
  delay(800);
  ymaxSpeed = 12.7;
  // sweepOnLeftToPoint(-8, -2, 0.3, 20, true);
  intakeMode = antiSpill;
  _2029BoomerangTo(-8, -2, 45, true, 12);
  move.voldrive(-2,-2);
  drop4bar();
  delay(200);
  slowMidSpeed = 50;
  intakeMode = midGoalSlow;
  delay(700);
  intakeMode = antiSpill;

  matchloaderUp();

  bangbangDrive(2, 0);
  modifyAngularPID(_90DegTKD*2, _90DegTKD*0.5);
  partialTurnToPoint(onLeft, 0.35, -4, -18, 8);
  raise4bar();
  revertToOriginalPIDs();

  drivePID.minOutput = 8;
  straightTo(-4, -18);

  midDescoreUp();
  swingPID.minOutput = 8;
  swingOnLeft(-45, 0);
  delay(800);
  // partialTurnToPoint(onLeft, 0.2, 0, 0, 15);
  drivePID.minOutput = 12.7;
  midDescoreUp();
  driveForward(20, -45, 0);
  delay(300);
  extendMidGoalHook();
  delay(300);
  move.voldrive(-3, -3);
  lowGoalSpeed = 30;
  intakeMode = lowGoal;

  delay(500000000);
}

void tenBTwoMidGoals() {
  tenB(-1, false, true);
  ymaxSpeed = 11;
  intakeMode = antiSpill;
  _2029BoomerangTo(-11, -3, 45, true, 8, 10, 0);
  // faceLeadAnd2029BoomerangTo(-11, -3, 45, true, 8, 10, 0);
  ymaxSpeed = 12.7;
  drop4bar();
  move.voldrive(-2,-2);
  delay(200);

  slowMidSpeed = 40;
  intakeMode = midGoalSlow;
  delay(1200);
  intakeMode = antiSpill;

  matchloaderUp();

  bangbangDrive(2, 0);
  modifyAngularPID(_90DegTKD*2, _90DegTKD*0.5);
  partialTurnToPoint(onLeft, 0.35, -8.5, -18, 8);
  raise4bar();
  revertToOriginalPIDs();

  drivePID.minOutput = 8;
  straightTo(-8.5, -18);

  midDescoreUp();
  // swingPID.minOutput = 8;
  swingOnLeft(-45, 0);
  delay(800);
  // partialTurnToPoint(onLeft, 0.2, 0, 0, 15);
  drivePID.minOutput = 12.7;
  midDescoreUp();
  driveForward(20, -45, 0);
  delay(300);
  extendMidGoalHook();
  delay(300);
  move.voldrive(-2, -2);
  lowGoalSpeed = 20;
  intakeMode = lowGoal;

  delay(500000000);
}

void nineBall() {
  tenB(1, true, false, true);

  ymaxSpeed = 5;
  straightlineTo(matchloaderX, -matchloaderY, false, 18);
  ymaxSpeed = 12.7;
  hoodDown();
  delay(700);

  scoreLong(4, 2000);
  matchloaderUp();

  // HookProcedure();

  driveForward(8);
  hoodDown();
  drivePID.minOutput = 12.7;
  driveBackward(12, chassis.h, 0);
  delay(10000000000);



}

void leftSevenCounter() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;
  int flip = -1;

  intakeMode = intake;
  midDescoreDown();

  hookUp();

  chassis.setPos(13, -45.82,0);

  readWall(3, 3, true);
  readWall(4, 4, true);
  

  /*delayedDrop(400, 0);

  moveTo(22, -24, false, 12, 60);
  matchloaderUp();

  delayedDrop(550, 0);

  ymaxSpeed = 7;
  moveTo(42, -6, false, 0);
  delay(1000);

  ymaxSpeed = 12.7;
  face(-90);
  delay(100);
  drive_min_output = 12;
  moveTo(flip*24, -27, true, 12, 30);
  moveTo(flip*47, -38, true, 6, 30);
  */
  
  tenB(-1);
  drive_min_output = 0;
  //face(180);
  readWall(3, 1);
  straightlineTo(flip*47.5, -67, false, 0, 5, 6.5);
  delay(700);
  readWall(3, 1);
  delay(900);
  intakeMode = antiSpill;
  ymaxSpeed = 12.7;
  sweepOnLeftToPoint(flip*20, -20, 0.4, 10, true);
  _2029BoomerangTo(-6.5, -5, 120, true, 15, 30, 1);
  move.voldrive(-2, -2);
  drop4bar();
  intakeMode = midGoal;
  delay(600);
  matchloaderUp();
  
  intakeMode = antiSpill;
  drive_min_output = 12;
  driveForward(10, chassis.h, 5);
  moveTo(flip*42, -3, true, 12);
  face(30);
  raise4bar();
  hookDown();
  delay(100);
  sweepOnLeft(0, 0.8, 0);
  straightTo(flip*38, -36, true, 12);
  _2029BoomerangTo(flip*49, -45, 50, true, 5, 8, 1);
  face(180);
  readWall(3, 1);
  _2029BoomerangTo(flip*48, -28, 180, true, 7, 14, 1);
  move.voldrive(-7, -7);
  hoodUp();
  intakeMode = longGoal;
}

void rightSevenCounter() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;

  intakeMode = intake;
  midDescoreDown();

  hookUp();

  chassis.setPos(13, -45.82,0);

    readWall(1, 1, true);
    readWall(4, 4, true);

  /*delayedDrop(400, 0);

  moveTo(flip*22, -24, false, 12, 60);
  matchloaderUp();

  delayedDrop(550, 0);

  ymaxSpeed = 7;
  moveTo(flip*42, -6, false, 0);
  delay(1000);

  ymaxSpeed = 12.7;
  face(flip*90);
  delay(100);
  drive_min_output = 12;
  moveTo(flip*24, -27, true, 12, 30);
  moveTo(flip*47, -38, true, 6, 30);*/

  tenB(1);
  drive_min_output = 0;
  face(180);
  readWall(1, 3);
  straightlineTo(48, -67, false, 0, 5, 6.5);
  delay(600);
  readWall(1, 3);
  delay(600);
  intakeMode = antiSpill;
  ymaxSpeed = 12.7;
  driveBackward(10, chassis.h, 5);
  matchloaderUp();
  facePoint(6, 7);
  _2029BoomerangTo(6, 7, -60, false, 15, 30, 1);
  move.voldrive(2, 2);
  drop4bar();
  intakeMode = lowGoal;
  delay(600);
  matchloaderUp();
  
  intakeMode = antiSpill;
  drive_min_output = 12;
  driveForward(10, chassis.h, 5);
  moveTo(42, -3, true, 12);
  delay(1000000000);
  face(160);
  raise4bar();
  hookDown();
  delay(100);
  sweepOnLeft(0, 0.8, 0);
  straightTo(38, -36, false, 12);
  _2029BoomerangTo(49, -45, 50,false, 5, 8, 1);
  face(180);
  readWall(1, 3);
  _2029BoomerangTo(48, -28, 180, true, 7, 14, 1);
  move.voldrive(-7, -7);
  hoodUp();
  intakeMode = longGoal;
}


void leftSevenSplit(){
  //arpit(true);


}

//////////////////////////////////////////////
//    _____ _  _______ _      _       _____ 
//   / ____| |/ /_   _| |    | |     / ____|
//  | (___ | ' /  | | | |    | |    | (___  
//   \___ \|  <   | | | |    | |     \___ \ 
//   ____) | . \ _| |_| |____| |____ ____) |
//  |_____/|_|\_\_____|______|______|_____/ 
//////////////////////////////////////////////

void frontClearProcedure() {
  intakeMode = intake;
  liftOdom();
  move.voldrive(0,0);
  midDescoreDown();
  delay(500);

  midDescoreUp();
  delay(150);
  midDescoreDown();
  
  delay(600);
  midDescoreUp();

  move.voldrive(5,5);
  delay(500);
  move.voldrive(0,0);
  midDescoreDown();
  delay(200);

  move.voldrive(-1.5, -1.5);
  delay(1000);
  midDescoreUp();
  delay(200);
  midDescoreDown();
  delay(200);

  move.voldrive(9, 9);
  delay(800);

  chassis.setPos(0, 0, 180);
  delay(20);

///////////////////////////////////////////////////////////
  move.voldrive(-12.7, -12.7);
  delay(600);

  while(true) {
    if (IMU1.orientation(pitch, degrees) > 2) {
      move.voldrive(-12.7, -12.7);
      delay(10);
    } else {
      intakeMode = antiSpill;
      move.voldrive(-12.7, -12.7);
      delay(40);
      break;
    }
  }

  move.voldrive(0,0);
  delay(60);

  leftdrive.stop(brake);
  rightdrive.stop(brake);
  dropOdom();

  midDescoreDown();

  delay(200);

}

double flipX;
double flipY;
double targH;
void intoLoader(double quadrant, double time = 1700) {
  if (quadrant == 1) {
    flipX = 1;
    flipY = 1;
  } else if (quadrant == 2) {
    flipX = -1;
    flipY = 1;
  } else if (quadrant == 3) {
    flipX = -1;
    flipY = -1;
  } else if (quadrant == 4) {
    flipX = 1;
    flipY = -1;
  }

  if (quadrant == 1 || quadrant == 2) {
    targH = 0;
  } else if (quadrant == 3 || quadrant == 4) {
    targH = 180;
  }

  facePoint(matchloaderX*flipX, (trueMatchloaderY)*flipY, 8, false);

  matchloaderDown();
  boomerangTo(matchloaderX*flipX, (trueMatchloaderY-4)*flipY, targH, false, 8, 0.8, 0);
  ymaxSpeed = 4;
  driveForward(100000, targH, 0);
  ymaxSpeed = 12.7;
  delay(time);

  // if (quadrant == 1) {
  //   face(0);
  //   readWall(2,2);
  //   readWall(1, 1);
  // }

}





double flip;
void loaderToOtherSide(bool left, bool right) {
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  drivePID.minOutput = 0;
  if (left) {
    flip = 1;
  }

  if (right) {
    flip = -1;
  }

  move.seamlessTransitions = true;

  bangbangDrive(-0.25,0);

  if (left) {
    moveTo(-62, -22, false, 20, 2, 359);
    boomerangTo(-60, 27, 0, false, 3, 0.6, 0);
  }

  if (right) {
    moveTo(62, 22, false, 20, 2, 359);
    boomerangTo(60, -27, 180, false, 3, 0.6, 0);
  }

  move.seamlessTransitions = false;

  if (left) {
    face(0);
    readWall(2, 2);
    readWall(3,3, true);
  }

  if (right) {
    face(180);
    readWall(4, 2);
    readWall(1, 3);
  }

  facePoint(-longGoalX*flip, (trueLongGoalY)*flip, 8, true);
  ymaxSpeed = 9;
  moveTo(-longGoalX*flip, (trueLongGoalY)*flip, true, 8);
  // move.voldrive(-3, -12);
  swingPID.minOutput = 12.7;
  if (left) {
    sweepOnLeft(0, 0.2, 0);
  }
  if (right) {
    sweepOnLeft(180, 0.2, 0);
  }
  delay(500);
  // revertToOriginalPIDs();

  move.voldrive(-12, -12);
  hoodUp();
  intakeMode = longGoal;
  delay(2000);

  chassis.setPos(-trueLongGoalX*flip, trueLongGoalY*flip, chassis.h);

  matchloaderDown();
  intakeMode = intake;

  ymaxSpeed = 5;
  straightTo(-matchloaderX*flip, matchloaderY*flip, false, 0);
  delay(1000);
  hoodDown();
  delay(2000);
  ymaxSpeed = 12.7;

  if (left) {
    scoreLong(2, 2000);
  }

  if (right) {
    scoreLong(4, 2000);
  }


  driveForward(7);
  hoodDown();
  driveBackward(7, chassis.h, 0);
  delay(500);
  
  matchloaderUp();

}

void sevenMidProcedure() {
  ymaxSpeed = 2;
  hmaxSpeed = 12.7;
  drivePID.minOutput = 0;
  move.seamlessTransitions = false;
  move.ptpCosineScaling = true;
  dropOdom();

  hoodUp();
  intakeMode = midGoalSlow;
  midGoalSpeed = 40;
  ymaxSpeed = 1;
  drivePID.maxOutput = 1;
  double count = 0;
  while (count<7) {
    drivePID.maxOutput = 1;
    driveForward(2,chassis.h, 0);
    delay(500);
    driveBackward(2,chassis.h, 0);
    delay(500);
    count+=1;
  }

  ymaxSpeed = 3;
  drivePID.maxOutput = 3;
  driveForward(3,chassis.h, 0);
  delay(500);
  intakeMode = coasting;
  driveBackward(5,chassis.h, 0);
  delay(500);

  ymaxSpeed = 12.7;
  drivePID.maxOutput = 12.7;
}

void sideClear() {
  drivePID.minOutput = 12.7;
  drivePID.maxOutput = 12.7;
  move.seamlessTransitions = true;

  chassis.setPos(-trueLongGoalX, trueLongGoalY, 0);

  moveTo(-31, 54, false, 11, 0, 20);
  drivePID.minOutput = 0;
  // intakeMode = intake;
  face(90, 3);

  driveForward(5);
  liftOdom();
  push(3, 0.1, 7.5);
  delayedDrop(0, 1000);
  move.voldrive(7,7);
  delay(500);
  move.voldrive(4,4);
  delay(1000);
  move.voldrive(7,7);
  delay(800);

  move.seamlessTransitions = false;

  dropOdom();
  driveBackward(4, 90, 3);
  face(90);
  readWall(1, 2);
  readWall(2, 3);

  ymaxSpeed = 8;
  faceLeadAndBoomerangTo(30, 30, -135);
  move.voldrive(0,0);

}

void testSideClear() {
  auto_settings();

  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  drivePID.minOutput = 12.7;
  move.seamlessTransitions = true;
  move.ptpCosineScaling = true;
  sensor4Offset = 23.2;
  dropOdom();
  chassis.setPos(-trueLongGoalX, trueLongGoalY, 0);
  intakeMode = intake;
  hookUp();
  midDescoreDown();

  drivePID.minOutput = 12.7;
  moveTo(-31, 54, false, 8, 0, 20);
  drivePID.minOutput = 0;
  face(90, 3);

  driveForward(5);
  liftOdom();
  push(3, 0.1, 7.5);
  delayedDrop(1000, 500); 
  move.voldrive(6,6);
  delay(500);
  move.voldrive(4,4);
  delay(500);
  move.voldrive(7,7);
  delay(1700);
  dropOdom();
  move.voldrive(0,0);
  delay(200);
  move.seamlessTransitions = false;

  driveBackward(4, 90, 3);
  face(180,0);
  delay(700);
  readWall(1, 3);
  readWall(2, 4);

  ymaxSpeed = 12.7;
  goTo(12, 36);
  ymaxSpeed = 5;
  intakeMode = antiSpill;
  goTo(24, 22);
  matchloaderDown();
  drop4bar();

  midDescoreUp();
  ymaxSpeed = 5;
  straightlineTo(10,6, true, 8);
  driveBackward(8, 45);
  move.voldrive(0,0);
  delay(500);

  sevenMidProcedure();

  ymaxSpeed = 12.7;

  straightTo(40,30);
  raise4bar();
  hoodDown();

  face(0,3);
  matchloaderDown();
  readWall(1, 1);

  ymaxSpeed = 5;
  straightTo(matchloaderX, matchloaderY, false, 0);
  hoodDown();
  delay(1700);
  chassis.setPos(chassis.x, trueMatchloaderY, chassis.h);
  ymaxSpeed = 12.7;

  loaderToOtherSide(false, true);

  move.voldrive(0,0);
  delay(10000);


}

void smallSideClear() {
  liftOdom();
  delayedDrop(1000, 500); 
  move.voldrive(8,8);
  delay(1000);
  move.voldrive(4,4);
  delay(500);
  move.voldrive(7,7);
  delay(1000);
  dropOdom();
  move.voldrive(0,0);
  delay(200);
  move.seamlessTransitions = false;

  driveBackward(4, 90, 3);
  swingOnRight(180,0);
  delay(700);
  readWall(3, 1);
  readWall(4, 2);
}


void SKILLS() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;
  sensor4Offset = 23.2;
  move.seamlessTransitions = true;
  midDescoreDown();
  hookUp();

  chassis.setPos(0, 0, 180);
  frontClearProcedure();

  readWall(4, 2);
  face(90, 8);
  driveForward(12, 90);
  face(90, 3);
  readWall(1, 2, true);

  goTo(20, -24);
  facePoint(10, -10);
  ymaxSpeed = 6;
  straightTo(10, -10, false, 12);
  move.voldrive(0,0);
  delay(100);
  leftdrive.stop(brake);
  rightdrive.stop(brake);
  midDescoreUp();
  lowGoalSpeed = 20;
  intakeMode = lowGoal;
  // move.voldrive(2,2);
  delay(3000);

  modifyAngularPID(YKP, 0);
  move.bigTime = 10;
  sweepOnLeft(0,0.2);
  move.bigTime = 300;
  revertToOriginalPIDs();
  face(-90, 3);
  readWall(4, 3);
  midDescoreDown();

  ymaxSpeed = 12.7;
  moveTo(-38, -48);
  // faceLeadAnd2029BoomerangTo(-40, -48, -145, false, 8, 24, 1, 12);
  matchloaderDown();
  face(180, 3);
  readWall(3, 1);

  ymaxSpeed = 5;
  straightlineTo(-matchloaderX, -75, false, 0);
  delay(2000);
  ymaxSpeed = 12.7;
  chassis.setPos(chassis.x, -trueMatchloaderY, chassis.h);

  // /////////////////////////////////////////////////////
  // intoLoader(3);
  loaderToOtherSide(true, false);
  // /////////////////////////////////////////////////////

  // /////////////////////////////////////////////////////
  // sideClear();
  // delay(100);
  // ////////////////////////////////////////////////////
  
  matchloaderUp();
  testSideClear();

  move.seamlessTransitions = true;
  moveTo(31, -54, false, 11, 0, 20);
  intakeMode = spit;
  face(-90, 3);
  move.voldrive(8,8);
  delay(1000);

}


void SmallTest(){
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;
  move.seamlessTransitions = true;

  move.voldrive(-5, -5);


  delay(50000000);

}

void SmallSkills() {  
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;
  move.seamlessTransitions = true;
  sensor4Offset = 23.2;
  chassis.setPos(0,0,-90);

  intakeMode = intake;
  readWall(4, 3);

  //Side clear
  smallSideClear();
  ymaxSpeed = 8;
  straightlineTo(-47, -48, true);
  face(180, 3);
  readWall(3, 1);
  straightlineTo(-longGoalX, -longGoalY, true, 15);
  move.voldrive(-7, -7);
  delay(300);
  face(180, 0);
  delay(100);
  move.voldrive(-10, -10);
  delay(100);
  matchloaderDown();
  hoodUp();
  intakeMode = longGoal;
  delay(1000);

  //first loader
  chassis.setPos(-trueLongGoalX, -trueLongGoalY, chassis.h);
  // ymaxSpeed = 8;
  // straightlineTo(-43, chassis.y);
  // matchloaderDown();
  // face(180, 3);
  // readWall(3, 1);
  ymaxSpeed = 5;
  straightlineTo(-matchloaderX, -matchloaderY, false, 15);
  ymaxSpeed = 8;
  delay(1000);
  hoodDown();
  delay(2000);

  //Other side first time
  loaderToOtherSide(true, false);

  //Park clear
  dropOdom();
  chassis.setPos(-trueLongGoalX, trueLongGoalY, 0);
  intakeMode = intake;
  hookUp();
  midDescoreDown();

  drivePID.minOutput = 12.7;
  moveTo(-31, 54, false, 8, 0, 20);
  drivePID.minOutput = 0;
  face(90, 3);

  driveForward(5);
  liftOdom();
  hoodDown();
  move.voldrive(8,8);
  delay(1000);
  move.voldrive(4,4);
  delay(500);
  move.voldrive(7,7);
  delay(1700);
  dropOdom();
  move.voldrive(0,0);
  delay(200);
  move.seamlessTransitions = false;

  swingOnRight(0, 3);
  readWall(1, 1);
  readWall(2, 2);

  ymaxSpeed = 8;
  straightlineTo(48, 40, true);

  face(0, 3);
  readWall(1, 1);

  straightlineTo(longGoalX, longGoalY, true, 15);
  move.voldrive(-7, -7);
  delay(300);
  hoodUp();
  intakeMode = longGoal;
  delay(200);
  face(0, 0);
  delay(200);
  move.voldrive(-10,-10);
  delay(500);
  chassis.setPos(longGoalX, trueLongGoalY, chassis.h);

  matchloaderDown();
  ymaxSpeed = 5;
  straightlineTo(matchloaderX, matchloaderY, false, 15);
  ymaxSpeed = 8;
  delay(1000);
  hoodDown();
  delay(2000);

  loaderToOtherSide(false, true);

  drivePID.maxOutput = 12.7;
  ymaxSpeed = 12.7;

  drivePID.minOutput = 12.7;
  moveTo(31, -54, false, 8, 0, 20);
  drivePID.minOutput = 0;
  face(-90, 3);

  move.voldrive(6,8);
  delay(800);















}