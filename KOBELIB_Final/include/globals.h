
#ifndef GLOBALS_H
#define GLOBALS_H

#include "vex.h"
using namespace vex;

// -------------------
// Motor Declarations
// -------------------
extern motor fl;
extern motor ml;
extern motor bl;

extern motor fr;
extern motor mr;
extern motor br;

extern motor intakeRollers;
extern motor secondRollers;
extern motor scoreRoller;

extern motor_group leftdrive;
extern motor_group rightdrive;

// -------------------
// Sensors
// -------------------
extern inertial IMU1;

extern rotation TX;
extern rotation TY;

extern double txD;
extern double tyD;

extern double tyOffset;
extern double txOffset;

extern double imu_multiplier;

// Wall distance sensors
extern distance DistSensor1;
extern distance DistSensor2;
extern distance DistSensor3;
extern distance DistSensor4;

// Intake distance sensors
extern distance OtwDistSensor;
extern distance ChamberDistSensor;

// Optical color sensor
extern optical OtwColorSensor;

// -------------------
// Pneumatics
// -------------------
extern digital_out hook;
extern digital_out hood;
extern digital_out matchloader;

// -------------------
// Intake + Color Sort Enums
// -------------------
enum IntakeMode {
  intake,
  spit,
  lowGoal,
  coasting,
  longGoal,
  midGoal,
  midGoalSlow,
  truncatedIntake,
  primeLowGoal,
  purgeOne,
  topStageJamInsurance,
  antiSpill,
  onlyTopStage,
  onlyBottomStage
};
extern IntakeMode intakeMode;

enum ColorSortMode {
  teamRedColorSort,
  teamBlueColorSort,
  none
};
extern ColorSortMode colorSortMode;

// -------------------
// Intake Monitoring Variables
// -------------------
extern bool blockOtw;

extern bool readingRed;
extern bool readingBlue;

extern bool blockOtwIsRed;
extern bool blockOtwIsBlue;

extern bool blockInChamber;

// Thresholds
extern double redThresholdUpper;
extern double redThresholdLower;
extern double blueThresholdUpper;
extern double blueThresholdLower;

extern double magDistanceThreshold;
extern double chamberDistanceThreshold;

// Sorting reverse timing
extern double reverseTimeLongSort;
extern double reverseTimeMidSort;

// Chamber purge time
extern double chamberPurgeTime;

// Store + purge logic
extern bool purgingDuringStore;

// Auton prime scoring
extern bool primeScoring;
extern double acceptableUnloadDist;

// Hook logic
extern bool raiseHook;

// Matchloader auton drop logic
extern bool runDelayedDrop;
extern double dropMatchloaderIn;
extern double raiseMatchloaderIn;

//Purge one ball
extern bool purgeOneBall;

//Intake prime delay
extern double startPrimingAfter;
extern bool startPrimingLowGoal;

extern double offsetPrint;


//Functions
void matchloaderUp();
void matchloaderDown();

void hookUp();
void hookDown();

void midDescoreUp();
void midDescoreDown();

void hoodDown();
void hoodUp();

void raise4bar();
void drop4bar();

// -------------------
// Brain Screen Globals
// -------------------
extern int lastColorSortState;
int getColorSortState();
void drawBackground(int state);

// -------------------
// Task Function Declarations
// -------------------
int magControlFcn();
int magMonitorFcn();
int scorePrimeFcn();
int hookFcn();
int delayedMatchloaderDropFcn();
int controllerScreenTask(void);
int brainScreenTaskFcn();
int delayedMatchloaderDropFcn();
extern int midGoalSpeed;

extern bool runDelayedDrop;
extern double dropMatchloaderIn;
extern double dropMatchloaderFor;
extern double lowGoalIn;

extern double lowGoalSpeed;
extern bool intakeAntiJamEnabled;

extern bool suspendScoreRoller;

extern double raiseIntakeIn;

extern bool slowLongGoal;

extern void delayedDrop();
extern void distanceToPointDrop();
extern double spitIn;

extern bool intakeLag;

extern bool suspendUpperRollers;

extern bool autonSort;

extern bool sawBadColor;

extern void dropOdom();
extern void liftOdom();

extern double trueH;

extern void extendMidGoalHook();
extern void retractMidGoalHook();

extern double slowMidSpeed;

extern bool antiJam;

extern bool suspendLower;



//DATA PRINTING
extern bool printingToConsole;

#endif
