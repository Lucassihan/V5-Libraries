#include "vex.h"
#include "threads.h"
#include "movement.h"
#include "driver.h"
#include "auton.h"
#include <cstdio>
#include <ios>
using namespace vex;
#include <cmath>
#include "math.h"
#include "driveControllers.h"
#include "logos.h"

brain Brain;
controller Controller = controller(primary);
std::string currentAuton;

// Rollers
motor intakeRollers = motor(PORT12, ratio6_1, true);
motor upperRollers = motor(PORT11, ratio6_1, false);

// Chassis
motor fl = motor(PORT13, ratio6_1, true);
motor ml = motor(PORT14, ratio6_1, true);
motor bl = motor(PORT15, ratio6_1, true);

motor fr = motor(PORT18, ratio6_1, false);
motor mr = motor(PORT19, ratio6_1, false);
motor br = motor(PORT20, ratio6_1, false);

motor_group leftdrive = motor_group(fl, ml, bl);
motor_group rightdrive = motor_group(fr, mr, br);

// Odometry and Heading
inertial IMU1 = inertial(PORT7);
inertial IMU2 = inertial(PORT4);

rotation TX = rotation(PORT6, false);
rotation TY = rotation(PORT21, true);

//Tracker wheel effective diameter
double txD = 2;
double tyD = 2*1.01265822785;

//Tracker wheel offsets
double txOffset =  -2.222126924; //-1.684495946.    //2.222126924
double tyOffset = -1.050972841; //-7/8  //1.050972841

//IMU Multiplier
double imu_multiplier = 1.00676;

//Wall reading distance sensors
distance DistSensor1 = distance(PORT8);
distance DistSensor2 = distance(PORT3);
distance DistSensor3 = distance(PORT16);
distance DistSensor4 = distance(PORT9);

//DRIVE CHOICE
bool curvatureDrive = false;
bool arcadeDrive = true;

//Mag distance sensors
distance OtwDistSensor = distance(PORT8);
distance ChamberDistSensor = distance(PORT15);

//Color sensor
optical OtwColorSensor = optical(PORT1);

//Pistons
digital_out hood = digital_out(Brain.ThreeWirePort.G);
digital_out hook = digital_out(Brain.ThreeWirePort.A); //Brain.ThreeWirePort.A
digital_out matchloader = digital_out(Brain.ThreeWirePort.D);
digital_out midDescore = digital_out(Brain.ThreeWirePort.H);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.B);
digital_out odomLift = digital_out(Brain.ThreeWirePort.F);
digital_out midGoalHook = digital_out(Brain.ThreeWirePort.E);

//Timers
timer R2Time;

bool l1Pressed_first = false;
bool l1Pressed_second = false;
bool l1released = false;

bool l2Pressed_first = false;
bool l2Pressed_second = false;
bool l2released = false;

bool r1Pressed_first = false;
bool r1Pressed_second = false;
bool r1released = false;

bool r2Pressed_first = false;
bool r2Pressed_second = false;
bool r2released = false;

bool noOtherButtonPressed = true;

bool l1NewPress = false;
bool l2NewPress = false;
bool r1NewPress = false;
bool r2NewPress = false;
bool upNewPress = false;
bool downNewPress = false;
bool leftNewPress = false;
bool rightNewPress = false;
bool aNewPress = false;
bool bNewPress = false;
bool xNewPress = false;
bool yNewPress = false;

bool l1JustReleased = false;
bool l2JustReleased = false;
bool r1JustReleased = false;
bool r2JustReleased = false;
bool upJustReleased = false;
bool downJustReleased = false;
bool leftJustReleased = false;
bool rightJustReleased = false;
bool aJustReleased = false;
bool bJustReleased = false;
bool xJustReleased = false;
bool yJustReleased = false;

// How long each button has been continuously held (ms), 0 when not pressed
uint32_t l1HeldFor = 0, l2HeldFor = 0, r1HeldFor = 0, r2HeldFor = 0;
uint32_t upHeldFor = 0, downHeldFor = 0, leftHeldFor = 0, rightHeldFor = 0;
uint32_t aHeldFor = 0, bHeldFor = 0, xHeldFor = 0, yHeldFor = 0;

// Snapshot of how long the button was held at the moment it was released (ms).
// Valid for one cycle alongside xxxJustReleased, then resets to 0.
uint32_t l1PressTime = 0, l2PressTime = 0, r1PressTime = 0, r2PressTime = 0;
uint32_t upPressTime = 0, downPressTime = 0, leftPressTime = 0, rightPressTime = 0;
uint32_t aPressTime = 0, bPressTime = 0, xPressTime = 0, yPressTime = 0;

bool l1Exhausted = false, l2Exhausted = false, r1Exhausted = false, r2Exhausted = false;

void controllerButtonMonitoringFcn() {
  static bool prevL1 = false, prevL2 = false, prevR1 = false, prevR2 = false;
  static bool prevUp = false, prevDown = false, prevLeft = false, prevRight = false;
  static bool prevA = false, prevB = false, prevX = false, prevY = false;
  static uint32_t lastTickTime = vex::timer::system();

  bool curL1 = Controller.ButtonL1.pressing();
  bool curL2 = Controller.ButtonL2.pressing();
  bool curR1 = Controller.ButtonR1.pressing();
  bool curR2 = Controller.ButtonR2.pressing();
  bool curUp = Controller.ButtonUp.pressing();
  bool curDown = Controller.ButtonDown.pressing();
  bool curLeft = Controller.ButtonLeft.pressing();
  bool curRight = Controller.ButtonRight.pressing();
  bool curA = Controller.ButtonA.pressing();
  bool curB = Controller.ButtonB.pressing();
  bool curX = Controller.ButtonX.pressing();
  bool curY = Controller.ButtonY.pressing();
  uint32_t now = vex::timer::system();

  // New press detection (true for one cycle only)
  l1NewPress = (curL1 && !prevL1);
  l2NewPress = (curL2 && !prevL2);
  r1NewPress = (curR1 && !prevR1);
  r2NewPress = (curR2 && !prevR2);
  upNewPress = (curUp && !prevUp);
  downNewPress = (curDown && !prevDown);
  leftNewPress = (curLeft && !prevLeft);
  rightNewPress = (curRight && !prevRight);
  aNewPress = (curA && !prevA);
  bNewPress = (curB && !prevB);
  xNewPress = (curX && !prevX);
  yNewPress = (curY && !prevY);

  // Just released detection (true for one cycle only)
  l1JustReleased = (!curL1 && prevL1);
  l2JustReleased = (!curL2 && prevL2);
  r1JustReleased = (!curR1 && prevR1);
  r2JustReleased = (!curR2 && prevR2);
  upJustReleased = (!curUp && prevUp);
  downJustReleased = (!curDown && prevDown);
  leftJustReleased = (!curLeft && prevLeft);
  rightJustReleased = (!curRight && prevRight);
  aJustReleased = (!curA && prevA);
  bJustReleased = (!curB && prevB);
  xJustReleased = (!curX && prevX);
  yJustReleased = (!curY && prevY);

  // Snapshot hold duration on release (valid for one cycle alongside xxxJustReleased)
  l1PressTime = l1JustReleased ? l1HeldFor : 0;
  l2PressTime = l2JustReleased ? l2HeldFor : 0;
  r1PressTime = r1JustReleased ? r1HeldFor : 0;
  r2PressTime = r2JustReleased ? r2HeldFor : 0;
  upPressTime = upJustReleased ? upHeldFor : 0;
  downPressTime = downJustReleased ? downHeldFor : 0;
  leftPressTime = leftJustReleased ? leftHeldFor : 0;
  rightPressTime = rightJustReleased ? rightHeldFor : 0;
  aPressTime = aJustReleased ? aHeldFor : 0;
  bPressTime = bJustReleased ? bHeldFor : 0;
  xPressTime = xJustReleased ? xHeldFor : 0;
  yPressTime = yJustReleased ? yHeldFor : 0;

  // Reset exhausted flags on release
  if (l1JustReleased) l1Exhausted = false;
  if (l2JustReleased) l2Exhausted = false;
  if (r1JustReleased) r1Exhausted = false;
  if (r2JustReleased) r2Exhausted = false;

  // Held-for tracking — accumulates while pressed, resets to 0 on release
  l1HeldFor = curL1 ? (l1HeldFor + (now - lastTickTime)) : 0;
  l2HeldFor = curL2 ? (l2HeldFor + (now - lastTickTime)) : 0;
  r1HeldFor = curR1 ? (r1HeldFor + (now - lastTickTime)) : 0;
  r2HeldFor = curR2 ? (r2HeldFor + (now - lastTickTime)) : 0;
  upHeldFor = curUp ? (upHeldFor + (now - lastTickTime)) : 0;
  downHeldFor = curDown ? (downHeldFor + (now - lastTickTime)) : 0;
  leftHeldFor = curLeft ? (leftHeldFor + (now - lastTickTime)) : 0;
  rightHeldFor = curRight ? (rightHeldFor + (now - lastTickTime)) : 0;
  aHeldFor = curA ? (aHeldFor + (now - lastTickTime)) : 0;
  bHeldFor = curB ? (bHeldFor + (now - lastTickTime)) : 0;
  xHeldFor = curX ? (xHeldFor + (now - lastTickTime)) : 0;
  yHeldFor = curY ? (yHeldFor + (now - lastTickTime)) : 0;

  // Classify each button on its rising edge — stays stable until released
  if (curL1 && !prevL1) {
    bool combo = l2Pressed_first || r1Pressed_first || r2Pressed_first;
    l1Pressed_first = !combo; l1Pressed_second = combo;
  }
  if (!curL1) { l1Pressed_first = false; l1Pressed_second = false; }

  if (curL2 && !prevL2) {
    bool combo = l1Pressed_first || r1Pressed_first || r2Pressed_first;
    l2Pressed_first = !combo; l2Pressed_second = combo;
  }
  if (!curL2) { l2Pressed_first = false; l2Pressed_second = false; }

  if (curR1 && !prevR1) {
    bool combo = l1Pressed_first || l2Pressed_first || r2Pressed_first;
    r1Pressed_first = !combo; r1Pressed_second = combo;
  }
  if (!curR1) { r1Pressed_first = false; r1Pressed_second = false; }

  if (curR2 && !prevR2) {
    bool combo = l1Pressed_first || l2Pressed_first || r1Pressed_first;
    r2Pressed_first = !combo; r2Pressed_second = combo;
  }
  if (!curR2) { r2Pressed_first = false; r2Pressed_second = false; }

  noOtherButtonPressed = !(curL1 || curL2 || curR1 || curR2);

  prevL1 = curL1; prevL2 = curL2; prevR1 = curR1; prevR2 = curR2;
  prevUp = curUp; prevDown = curDown; prevLeft = curLeft; prevRight = curRight;
  prevA = curA; prevB = curB; prevX = curX; prevY = curY;
  lastTickTime = now;
}

bool midDescoreIsUp;

void midDescoreUp() {
  midDescore.set(1);
  midDescoreIsUp = true;
}

void midDescoreDown() {
  midDescore.set(0);
  midDescoreIsUp = false;
}

void midDescoreToggle() {
  if (midDescoreIsUp) {
    midDescoreDown();
  } else {
    midDescoreUp();
  }
}

void hookUp() {
  hook.set(1);
}

void hookDown() {
  hook.set(0);
}

void hoodUp() {
  hood.set(1);
}

void hoodDown() {
  hood.set(0);
}

bool matchloaderIsDown = false;
void matchloaderUp() {
  matchloader.set(0);
  matchloaderIsDown = false;
}

void matchloaderDown() {
  matchloader.set(1);
  matchloaderIsDown = true;
}

void liftOdom() {
  odomLift.set(0);
}

void dropOdom() {
  odomLift.set(1);
}

bool fourBarUp = false;

void raise4bar() {
  intakeLift.set(0);
  fourBarUp = true;
}

void drop4bar() {
  intakeLift.set(1);
  fourBarUp = false;
}

bool midGoalHookIsOut = false;

void extendMidGoalHook() {
  midGoalHook.set(1);
  midGoalHookIsOut = true;
}

void retractMidGoalHook() {
  midGoalHook.set(0);
  midGoalHookIsOut = false;
}







void coastAllRollers() {
  intakeRollers.stop(coast);
  upperRollers.stop(coast);
}

void intakeFcn() {
  intakeRollers.spin(forward, 100, percent);
  upperRollers.spin(forward, 100, percent);
}

void scoreFcn() {
  intakeRollers.spin(forward, 100, percent);
  upperRollers.spin(forward, 100, percent);
}

void spitFcn() {
  intakeRollers.spin(reverse, 100, percent);
  upperRollers.spin(reverse, 100, percent);
}

double lowGoalSpeed = 20;
void lowGoalFcn(bool suspendUpper = false) {
    intakeRollers.spin(reverse, lowGoalSpeed, percent);
    if (suspendUpper) {
      upperRollers.stop(coast);
    } else {
      upperRollers.spin(reverse, 100, percent);
    }
}

//Intake monitor variables
bool blockOtw = false;

bool readingRed = false;
bool readingBlue = false;

bool blockOtwIsRed = false;
bool blockOtwIsBlue = false;

//Color sensor threshold constants
double redThresholdUpper = 355;
double redThresholdLower = 330;
double blueThresholdUpper = 260;
double blueThresholdLower = 200;

//Distance sensor threshold
double magDistanceThreshold = 2;
double chamberDistanceThreshold = 1.5;

//Reverse amount for color sort
double reverseTimeLongSort = 300;
double reverseTimeMidSort = 500;

//Chamber purge time
double chamberPurgeTime = 200;

//Purge oppo colors during store
bool purgingDuringStore = true;

//Intake control
IntakeMode intakeMode = coasting;

//Color sort toggles
ColorSortMode colorSortMode = none;

//Mid goal speed
int midGoalSpeed = 100;

//Low goal delay
double lowGoalIn = 0;

//Anti jam
bool intakeAntiJamEnabled = true;

//Delayed goal prime
double startPrimingAfter = 0;
bool startPrimingLowGoal = false;
double spitIn = 0;

bool suspendUpperRollers = false;
bool slowLongGoal = false;

bool wasJustIntaking = false;

double score3Time = 500;

bool firstLongGoal = false;

bool intakeLag = false;

bool autonSort = false;

bool intakeSort = false;

bool sawBadColor = false;

double slowMidSpeed = 35;

bool antiJam = true;

bool justDropped4Bar = false;

bool suspendLower = false;


// void antiJam(bool upper, bool intake) {
//   delay(100);
//   if (upper) {
//     if (upperRollers.velocity(pct) < 10) {
//       intakeRollers.spin(forward, 100, percent);

//       upperRollers.spin(reverse, 100, percent);
//       delay(200);
//       upperRollers.spin(forward, 100, percent);
//     }
//   }

//   if (intake) {
//     if (intakeRollers.velocity(pct) < 10) {
//       upperRollers.spin(forward, 100, percent);

//       intakeRollers.spin(reverse, 100, percent);
//       delay(200);
//       intakeRollers.spin(forward, 100, percent);
//     }
//   }
// }

// Intake Control Task
int magControlFcn() {
  while (true) {
    switch (intakeMode) {
      case (intake):
          intakeFcn();
          wasJustIntaking = true;
          
          if (intakeSort) {
            if (colorSortMode == teamRedColorSort && readingBlue && OtwColorSensor.isNearObject()) {
              hoodUp();
            } else if (colorSortMode == teamBlueColorSort && readingRed && OtwColorSensor.isNearObject()) {
              hoodUp();
            } else {
              hoodDown();
            }
          }
          if (antiJam) {
            if (upperRollers.velocity(pct) < 5) {
              intakeRollers.spin(forward, 100, percent);

              upperRollers.spin(reverse, 100, percent);
              delay(100);
              upperRollers.spin(forward, 100, percent);
            }

            upperRollers.spin(forward, 100, percent);
            intakeRollers.spin(forward, 100, percent);
            delay(300);

            if (intakeRollers.velocity(pct) < 3) {
              upperRollers.spin(forward, 100, percent);

              intakeRollers.spin(reverse, 100, percent);
              delay(100);
              intakeRollers.spin(forward, 100, percent);
            }
          }

          // upperRollers.spin(forward, 100, percent);
          // intakeRollers.spin(forward, 100, percent);
          // delay(300);

          // antiJam(true, true);

          sawBadColor = false;

        break;

      case (spit):
        spitFcn();

        break;

      case (lowGoal):
        if (suspendUpperRollers) {
          lowGoalFcn(true);
        } else {
          lowGoalFcn(false);
        }

        upperRollers.spin(reverse, 100, percent);
        intakeRollers.spin(reverse, lowGoalSpeed, percent);

        break;

      case (coasting):
        coastAllRollers();
        if (wasJustIntaking && intakeLag) {
          upperRollers.spin(forward, 100, percent); //make sure the blocks are pressing against the hood, makes scoring more consistent
          delay(250);
        }
        wasJustIntaking = false;

        break;

      case (longGoal):
        if (not suspendLower) {
          scoreFcn();
        } else if (suspendLower) {
          upperRollers.spin(forward, 100, percent);
          intakeRollers.stop(brake);
        }
        
        if (autonSort) {
          if (colorSortMode == teamRedColorSort && readingBlue) {
            scoreFcn();

            intakeRollers.spin(reverse,100,percent);
            upperRollers.spin(reverse,100,percent);
            delay(500);

            sawBadColor = true;
          }

          if (colorSortMode == teamBlueColorSort && readingRed) {
            scoreFcn();

            intakeRollers.spin(reverse,100,percent);
            upperRollers.spin(reverse,100,percent);
            delay(500);

            sawBadColor = true;
          }
        }

        if (antiJam && not suspendLower) {
          if (upperRollers.velocity(pct) < 3) {
            upperRollers.spin(forward, 100, percent);

            upperRollers.spin(reverse, 100, percent);
            delay(100);
            upperRollers.spin(forward, 100, percent);
          }

        upperRollers.spin(forward, 100, percent);
        intakeRollers.spin(forward, 100, percent);
        delay(200);

        }



        break;

      case (midGoal):
        scoreFcn();

        if (autonSort) {
          if (colorSortMode == teamRedColorSort && readingBlue) {
            scoreFcn();

            intakeRollers.spin(reverse,100,percent);
            upperRollers.spin(reverse,100,percent);
            delay(500);
            sawBadColor = true;
          }

          if (colorSortMode == teamBlueColorSort && blockOtwIsRed) {
            scoreFcn();

            intakeRollers.spin(reverse,100,percent);
            upperRollers.spin(reverse,100,percent);
            delay(500);
            sawBadColor = true;
          }
        }
        
        break;

      case (midGoalSlow):
        upperRollers.spin(forward, slowMidSpeed, percent);
        intakeRollers.spin(forward, 100, percent);

        break;

      case (truncatedIntake):
        intakeRollers.spin(forward, 100, percent);
        upperRollers.stop(brake);

        break;

      case (topStageJamInsurance):
        intakeRollers.spin(forward, 100, percent);

        upperRollers.spin(reverse, 100, percent);
        delay(200);
        upperRollers.spin(forward, 100, percent);
        delay(200);

        break;

      case (antiSpill):
        intakeRollers.spin(forward, 100, percent);
        upperRollers.spin(reverse, 100, percent);

        break;

      case (onlyTopStage):
        intakeRollers.stop(coast);
        upperRollers.spin(forward,50,percent);

        break;

      case (onlyBottomStage):
        intakeRollers.spin(forward, 100, percent);
        upperRollers.stop(brake);

        break;
    }

    if (justDropped4Bar) {
      upperRollers.spin(reverse, 100, percent);
      delay(200);
      justDropped4Bar = false;
    }

    vex::this_thread::sleep_for(10);
    
  }
  return 0;
}

// Mag monitoring task
int magMonitorFcn() {
  while (true) {
    //Is there a block otw?
    if (OtwDistSensor.objectDistance(inches) < magDistanceThreshold) {blockOtw = true;} else {blockOtw = false;}

    //What color am I reading?
    if (OtwColorSensor.hue() < redThresholdUpper && OtwColorSensor.hue() > redThresholdLower) {
      readingRed = true;
      readingBlue = false;
    } else if (OtwColorSensor.hue() < blueThresholdUpper && OtwColorSensor.hue() > blueThresholdLower) {
      readingBlue = true;
      readingRed = false;
    } else {
      readingRed = false;
      readingBlue = false;
    }

    //If im reading X color, and I know a block is on a way, then the block on the way must be X colored.
    if (blockOtw && readingRed) {
      blockOtwIsRed = true;
      blockOtwIsBlue = false;
    } else if (blockOtw && readingBlue) {
      blockOtwIsBlue = true;
      blockOtwIsRed = false;
    } else {
      blockOtwIsRed = false;
      blockOtwIsBlue = false;
    }

    vex::this_thread::sleep_for(10);
  }
  return 0;
}


double raiseIntakeIn = 0;

int delayedIntakeLiftFcn() {
  while (true) {
    if (raiseIntakeIn != 0) {
      delay(raiseIntakeIn);
      midDescoreUp();
      raiseIntakeIn = 0;
    }
  vex::this_thread::sleep_for(10);
  }
}













int trackDisplayTask() {
    while (true) {

        // ---------- Wheel & Gear Constants ----------
        const double wheelDiameter = 3.25;                         // inches
        const double wheelCircumference = wheelDiameter * M_PI;    // π * d

        // Gear ratio: motor → wheel
        // 36T driving 48T = 36/48 = 0.75  (wheel turns 0.75 for every motor turn)
        const double gearRatio = 36.0 / 48.0;

        // ---------- Read encoder ----------
        double motorDegrees = mr.position(vex::deg); // motor deg

        // Convert motor deg → wheel deg
        double wheelDegrees = motorDegrees * gearRatio;

        // Convert wheel deg → inches
        double inchesTracked = (wheelDegrees / 360.0) * wheelCircumference;

        // ---------- Display ----------
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(2, 10);
        Controller.Screen.print("MR Inches: %.3f", inchesTracked);

        vex::this_thread::sleep_for(50);
    }
    return 0;
}

int controllerScreenTask(void) {
  double TX_initial = TX.position(rotationUnits::deg); // store starting position

  while (true) {
    float leftMotorTempAvg = (
      fl.temperature(celsius) +
      ml.temperature(celsius) +
      bl.temperature(celsius)
    ) / 3.0;

    float rightMotorTempAvg = (
      fr.temperature(celsius) +
      mr.temperature(celsius) +
      br.temperature(celsius)
    ) / 3.0;

    float chassisTempAvg = (leftMotorTempAvg + rightMotorTempAvg) / 2.0;

    Controller.Screen.clearScreen();
    
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("X:%.3f", chassis.x);
    
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Y:%.3f", chassis.y);
    
    Controller.Screen.setCursor(3, 1); 
    // Controller.Screen.print("H: %.3f", chassis.h);
    Controller.Screen.print("Upper: %.3f", upperRollers.temperature(celsius));

    Controller.Screen.setCursor(1, 10);
    Controller.Screen.print("Chassis:%2.f", chassisTempAvg);

    Controller.Screen.setCursor(2, 10);
    Controller.Screen.print("Lower: %.3f", intakeRollers.temperature(celsius));
    // Controller.Screen.print("Sort: %s", colorSortMode == teamRedColorSort ? "Red" : colorSortMode == teamBlueColorSort ? "Blue" : "None");

    Controller.Screen.setCursor(3, 13);
    Controller.Screen.print(currentAuton.c_str());

    vex::this_thread::sleep_for(100);
  }
  return 0;
}


//BRAIN SCREEN STUFF
bool printingDistances = true;
int brainScreenTaskFcn() {
  Brain.Screen.setFont(mono20); // Or monoXL

  while (true) {
    // HUD overlay (distance sensors)
    if (printingDistances) {
    Brain.Screen.printAt(5,   20, "D1: %.5f   ", DistSensor1.objectDistance(inches));
    Brain.Screen.printAt(350, 20, "D2: %.5f   ", DistSensor2.objectDistance(inches));
    Brain.Screen.printAt(5,  235, "D3: %.5f   ", DistSensor3.objectDistance(inches));
    Brain.Screen.printAt(350,235, "D4: %.5f   ", DistSensor4.objectDistance(inches));
    }

    // // Tracking wheel inches for offset tuning
    // double txInches = TX.position(deg) * txD * M_PI / 360.0;
    // double tyInches = TY.position(deg) * tyD * M_PI / 360.0;
    // Brain.Screen.printAt(5, 120, "TX: %.5f in   ", txInches);
    // Brain.Screen.printAt(5, 145, "TY: %.5f in   ", tyInches);

    vex::this_thread::sleep_for(50);
  }
  return 0;
}

void vexcodeInit(void) {

  task controllerScreen(controllerScreenTask);

  task magControlTask(magControlFcn);
  // task magMonitorTask(magMonitorFcn);
  task delayedMatchloaderDropTask(delayedMatchloaderDropFcn);
  task brainScreenTask(brainScreenTaskFcn);
  // task delayedIntakeLiftTask(delayedIntakeLiftFcn);

  // task trackDisplayTaskCall(trackDisplayTask);

  OtwColorSensor.setLightPower(100);
  OtwColorSensor.setLight(ledState::on);
  OtwColorSensor.integrationTime(5);

  dropOdom();
  matchloaderUp();
  hookDown();
  retractMidGoalHook();
  // midDescoreUp();

  colorSortMode = none;
  intakeMode = coasting;
  fourBarUp = true;

}

competition Competition;

PID turnPID;
PID swingPID;
PID drivePID;
PID cataPID;
PID arcPID;
movement move;
robot chassis;

void pre_auton(void) {
  vexcodeInit();
  task motors(autonMotors);
  task data(collectData);
  task output(dataSend);

  move.autondriveenabled = false;
}

bool didAuton = false;
int autonIndex = 3;

void autonomous(void) {
  move.autondriveenabled = true;

  didAuton = true;

  // rightSplit(9);
  // nineBall();
  // sevenRushRight();
  // SOLOAWP();
  // leftSplit(9);
  // SKILLS();
  // SmallSkills();

  // partnerCamp();
  twoMidGoals();
  // tenBTwoMidGoals();
  // nineBallTwoMidGoals();

  // SmallTest();
  // leftSplit(9);
  // frontClearProcedure();
  // nineBall();
  // leftEarlySplit();
  // leftMidOnly();
  // tenBLeft();
  // tenBRight();
  // tenBLeft();
  //  rightSplit(7);
  // leftSplit(9);
  // sevenRushLeft();
  // tenB();
  // sevenRushRight();
  // hkMovement();
  // sixRushSplit();
  // arpitWithInsurance();
  //rightSevenCounter();
  // testSideClear();
  // sideClear();
  // SKILLS();
  // twoMidGoals();
  // sixRush();
  //arpit(true, 1);
  // tenBRight();
  // rightSplit(9);
  // SOLO();
  // delay(5000000);
  // FastRightSide();

  // if (autonIndex == 0) {
  //   SOLO();

  // } else if (autonIndex == 1) {
  //   //arpit();

  // } else if (autonIndex == 2) {

  // } else if (autonIndex == 3) {

  // } else if (autonIndex == 4) {

  // } else if (autonIndex == 5) {

  // } else if (autonIndex == 6) {

  // } else if (autonIndex == 7) {

  // } else if (autonIndex == 8) {

  // } else if (autonIndex == 9) {

  // }
    
  // delay(9999999999999); Used to be delay but I think was causing issues

  while (Competition.isAutonomous()) { //This instead to keep the thread running during auton
    wait(20, msec);
  }
}

void usercontrol(void) {

  autonSort = true;
  
  bool hookUnlocked;

  double trueH;

  if (didAuton) {
    hookUnlocked = true;
    matchloaderUp();
    liftOdom();
    intakeSort = true;
    antiJam = false;
  } else {
    hookUnlocked = false;
    antiJam = false;
  }

  liftOdom();

  intakeLag = false;

  retractMidGoalHook();

  liftOdom();
  move.autondriveenabled = false;

  while (true) {
    trueH = chassis.h; 

    //BUTTON MONITOR////////////////////////////////////
    controllerButtonMonitoringFcn();
    ////////////////////////////////////////////////////

    //DRIVE CHOICE//////////////////////////////////////
    if (arcadeDrive) {
      arcadeDriveFcn();
    }

    if (curvatureDrive) {
      curvatureDriveFcn();
    }
    
    ////////////////////////////////////////////////////

    //INTAKE////////////////////////////////////////////
    if (l2Pressed_first) {
      midDescoreDown();
      hoodDown();
      raise4bar();
      intakeMode = intake;
      // if (not fourBarUp) {
      //   intakeMode = antiSpill;
      // } else {
      //   intakeMode = intake;
      // }
    }

    /////////////////////////////////////////////////////

    //SPIT///////////////////////////////////////////////
    if (l1Pressed_first) {
      midDescoreDown();
      intakeMode = spit;
    }

    if (l1Pressed_first && l2Pressed_second) {
      midDescoreUp();
      lowGoalSpeed = 50; //was 70
      intakeMode = lowGoal;
    }

    if (not l1Pressed_first) {
      midDescoreDown();
    }

    /////////////////////////////////////////////////////

    //MID GOAL//////////////////////////////////////////
    if (r1Pressed_first) {
      if (fourBarUp) {
        drop4bar();
        r1Exhausted = true;
        justDropped4Bar = true;
      } else if (not r1Exhausted) {
        intakeMode = midGoal;
        justDropped4Bar = false;
      }
    }

    if (r1JustReleased) {
      r1Exhausted = false;
    }

    /////////////////////////////////////////////////////

    //LONG GOAL//////////////////////////////////////////
    if (r2Pressed_first) {
      if (not fourBarUp) {
        raise4bar(); // just raise, don't score
        r2Exhausted = true;
      } else if (not r2Exhausted) {
        hoodUp();
        matchloaderUp();
        intakeMode = longGoal;
      }
    }

    if (r2JustReleased) {
      r2Exhausted = false;
    }

    if (aNewPress) {
      hoodDown();
    }

    /////////////////////////////////////////////////////

    //COASTING///////////////////////////////////////////
    if (not l2Pressed_first && not r2Pressed_first && not r1Pressed_first && not l1Pressed_first) {
      intakeMode = coasting;
      firstLongGoal = false;
    }

    /////////////////////////////////////////////////////

    //HOOK///////////////////////////////////////////////
    if (Controller.ButtonDown.pressing()) {
      hookDown();
      hookUnlocked = true; 
    } else {
      if (hookUnlocked) {
        hookUp();
      }
    }

    /////////////////////////////////////////////////////

    //HOOK MACRO!////////////////////////////////////////
    // if (Controller.Axis2.position(percent) > 90) {
    //   move.autondriveenabled = true;
    //   hookDown();
    //   HookProcedure();
    //   move.autondriveenabled = false;
    // }

    /////////////////////////////////////////////////////

    //MID GOAL HOOK /////////////////////////////////////
    if (rightNewPress || yNewPress) {
      if (midGoalHookIsOut) {
        retractMidGoalHook();
      } else {
        extendMidGoalHook();
      }
    }

    //OTHER HOOK////////////////////////////////////////
    // if (Controller.Axis2.position(percent) < -90) {
    //   move.autondriveenabled = true;
    //   hookDown();
    //   southpawHook();
    //   move.autondriveenabled = false;
    // }

    /////////////////////////////////////////////////////

    //COLOR SORT CONTROL//////////////////////////////////
    if (leftNewPress) {
      if (colorSortMode == none) {
        if (not didAuton) {
          drawRedLogo();
        }
        colorSortMode = teamRedColorSort;
      } else if (colorSortMode == teamRedColorSort) {
        if (not didAuton) {
          drawBlueLogo();
        }
        colorSortMode = teamBlueColorSort;
      } else if (colorSortMode == teamBlueColorSort) {
        if (not didAuton) {
          drawNeutralLogo();
        }
        colorSortMode = none;
      }

      if (didAuton) {
        colorSortMode = none;
      }

      Controller.rumble(".");
    }

    /////////////////////////////////////////////////////

    //MATCHLOADER CONTROL////////////////////////////////
    // if (yNewPress) {
    //   if (matchloaderIsDown) {
    //     matchloaderUp();
    //   } else {
    //     matchloaderDown();
    //   }
    // }

    if (bNewPress) {
      if (matchloaderIsDown) {
        matchloaderUp();
      } else {
        matchloaderDown();
      }
    }

    /////////////////////////////////////////////////////

    // if (rightNewPress) {
    //   autonIndex += 1;
    //   Controller.rumble(".");
    // }

    // if (autonIndex == 0) {
    //   currentAuton = "SOLO";

    // } else if (autonIndex == 1) {
    //   currentAuton = "L6RUSH";

    // } else if (autonIndex == 2) {
    //   currentAuton = "R6RUSH";

    // } else if (autonIndex == 3) {
    //   currentAuton = "L6SPLIT";

    // } else if (autonIndex == 4) {
    //   currentAuton = "R6SPLIT";

    // } else if (autonIndex == 5) {
    //   currentAuton = "L9SPLIT";

    // } else if (autonIndex == 6) {
    //   currentAuton = "R9SPLIT";

    // } else if (autonIndex == 7) {
    //   currentAuton = "L7CNTR";

    // } else if (autonIndex == 8) {
    //   currentAuton = "R7CNTR";

    // } else if (autonIndex == 9) {
    //   currentAuton = "SKILLS";
    // }

    // // bounds clamp
    // if (autonIndex > 4) {
    //   autonIndex = 0;
    // }


    // if (xNewPress) {
    //   move.autondriveenabled = true;
    //   SKILLS();
    //   delay(10000000);
    //   move.autondriveenabled = false;
    // }


  wait(10, msec);
  }
}

// void usercontrol(void) {

//   autonSort = true;
  
//   bool hookUnlocked;

//   double trueH;

//   if (didAuton) {
//     hookUnlocked = true;
//     matchloaderUp();
//     liftOdom();
//     intakeSort = true;
//     antiJam = false;
//   } else {
//     hookUnlocked = false;
//     antiJam = false;
//   }

//   intakeLag = false;

//   retractMidGoalHook();

//   liftOdom();
//   move.autondriveenabled = false;

//   while (true) {
//     move.autondriveenabled = true;
//     SmallSkills();
//     delay(5000000000);
//     move.autondriveenabled = false;

//     //BUTTON MONITOR////////////////////////////////////
//     controllerButtonMonitoringFcn();
//     ////////////////////////////////////////////////////

//     //DRIVE CHOICE//////////////////////////////////////
//     if (arcadeDrive) {
//       arcadeDriveFcn();
//     }

//     if (curvatureDrive) {
//       curvatureDriveFcn();
//     }
    
//     ////////////////////////////////////////////////////

//     //INTAKE////////////////////////////////////////////
//     if (l2Pressed_first) {
//       midDescoreDown();
//       hoodDown();
//       raise4bar();
//       intakeMode = intake;
//       // if (not fourBarUp) {
//       //   intakeMode = antiSpill;
//       // } else {
//       //   intakeMode = intake;
//       // }
//     }

//     /////////////////////////////////////////////////////

//     //SPIT///////////////////////////////////////////////
//     if (l1Pressed_first) {
//       midDescoreUp();
//       lowGoalSpeed = 25;
//       intakeMode = lowGoal;
//     }

//     // if (l1Pressed_first && l2Pressed_second) {
//     //   midDescoreUp();
//     //   lowGoalSpeed = 50; //was 70
//     //   intakeMode = lowGoal;
//     // }

//     // if (not l1Pressed_first) {
//     //   midDescoreDown();
//     // }

//     if (l1Pressed_first && l2Pressed_second) {
//       midDescoreDown();
//       lowGoalSpeed = 60; //was 70
//       intakeMode = lowGoal;
//     }

//     //ACTIVATE THIS AGAIN
//     // if (not l1Pressed_first) {
//     //   midDescoreDown();
//     // }


//     /////////////////////////////////////////////////////

//     //MID GOAL//////////////////////////////////////////
//     if (r1Pressed_first) {
//       if (fourBarUp) {
//         drop4bar();
//         r1Exhausted = true;
//         justDropped4Bar = true;
//         intakeMode = antiSpill;
//       } else if (not r1Exhausted) {
//         slowMidSpeed = 30;
//         intakeMode = midGoalSlow;
//         justDropped4Bar = false;
//       }
//     }

//     if (r1JustReleased) {
//       r1Exhausted = false;
//     }

//     /////////////////////////////////////////////////////

//     //LONG GOAL//////////////////////////////////////////
//     if (r2Pressed_first) {
//       if (not fourBarUp) {
//         raise4bar(); // just raise, don't score
//         r2Exhausted = true;
//       } else if (not r2Exhausted) {
//         hoodUp();
//         matchloaderUp();
//         intakeMode = longGoal;
//       }
//     }

//     if (r2JustReleased) {
//       r2Exhausted = false;
//     }

//     if (aNewPress) {
//       hoodDown();
//     }

//     /////////////////////////////////////////////////////

//     //COASTING///////////////////////////////////////////
//     if (not l2Pressed_first && not r2Pressed_first && not r1Pressed_first && not l1Pressed_first) {
//       intakeMode = coasting;
//       firstLongGoal = false;
//     }

//     /////////////////////////////////////////////////////

//     //HOOK///////////////////////////////////////////////
//     if (Controller.ButtonDown.pressing()) {
//       hookDown();
//       hookUnlocked = true; 
//     } else {
//       if (hookUnlocked) {
//         hookUp();
//       }
//     }

//     /////////////////////////////////////////////////////

//     //HOOK MACRO!////////////////////////////////////////
//     // if (Controller.Axis2.position(percent) > 90) {
//     //   move.autondriveenabled = true;
//     //   hookDown();
//     //   HookProcedure();
//     //   move.autondriveenabled = false;
//     // }

//     /////////////////////////////////////////////////////

//     //MID GOAL HOOK /////////////////////////////////////
//     if (rightNewPress || yNewPress) {
//       if (midGoalHookIsOut) {
//         retractMidGoalHook();
//       } else {
//         extendMidGoalHook();
//       }
//     }

//     //OTHER HOOK////////////////////////////////////////
//     // if (Controller.Axis2.position(percent) < -90) {
//     //   move.autondriveenabled = true;
//     //   hookDown();
//     //   southpawHook();
//     //   move.autondriveenabled = false;
//     // }

//     /////////////////////////////////////////////////////

//     //COLOR SORT CONTROL//////////////////////////////////
//     if (leftNewPress) {
//       if (colorSortMode == none) {
//         if (not didAuton) {
//           drawRedLogo();
//         }
//         colorSortMode = teamRedColorSort;
//       } else if (colorSortMode == teamRedColorSort) {
//         if (not didAuton) {
//           drawBlueLogo();
//         }
//         colorSortMode = teamBlueColorSort;
//       } else if (colorSortMode == teamBlueColorSort) {
//         if (not didAuton) {
//           drawNeutralLogo();
//         }
//         colorSortMode = none;
//       }

//       if (didAuton) {
//         colorSortMode = none;
//       }

//       Controller.rumble(".");
//     }

//     /////////////////////////////////////////////////////

//     //MATCHLOADER CONTROL////////////////////////////////
//     // if (yNewPress) {
//     //   if (matchloaderIsDown) {
//     //     matchloaderUp();
//     //   } else {
//     //     matchloaderDown();
//     //   }
//     // }

//     if (bNewPress) {
//       if (matchloaderIsDown) {
//         matchloaderUp();
//       } else {
//         matchloaderDown();
//       }
//     }

//     /////////////////////////////////////////////////////

//     // if (rightNewPress) {
//     //   autonIndex += 1;
//     //   Controller.rumble(".");
//     // }

//     // if (autonIndex == 0) {
//     //   currentAuton = "SOLO";

//     // } else if (autonIndex == 1) {
//     //   currentAuton = "L6RUSH";

//     // } else if (autonIndex == 2) {
//     //   currentAuton = "R6RUSH";

//     // } else if (autonIndex == 3) {
//     //   currentAuton = "L6SPLIT";

//     // } else if (autonIndex == 4) {
//     //   currentAuton = "R6SPLIT";

//     // } else if (autonIndex == 5) {
//     //   currentAuton = "L9SPLIT";

//     // } else if (autonIndex == 6) {
//     //   currentAuton = "R9SPLIT";

//     // } else if (autonIndex == 7) {
//     //   currentAuton = "L7CNTR";

//     // } else if (autonIndex == 8) {
//     //   currentAuton = "R7CNTR";

//     // } else if (autonIndex == 9) {
//     //   currentAuton = "SKILLS";
//     // }

//     // // bounds clamp
//     // if (autonIndex > 4) {
//     //   autonIndex = 0;
//     // }


//     // if (xNewPress) {
//     //   move.autondriveenabled = true;
//     //   SKILLS();
//     //   delay(10000000);
//     //   move.autondriveenabled = false;
//     // }


//   wait(10, msec);
//   }
// }

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}