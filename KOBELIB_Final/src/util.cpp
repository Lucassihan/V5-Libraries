#include "vex.h"

double TO_RAD(double n) { // converts the input to radians
  return n * 0.01745329252;
}

double TO_DEG(double n) { // converts the input to degrees
  return n * 57.29577951;
}

int signum(float input) { // returns the sign of an input
  if (input > 0.0) return 1;
  if (input < 0.0) return -1;
  return 0;
}

float min(float first, float second) {
  if (first < second) { 
    return first; 
      } else {
    return second;
  }
}

float max(float first, float second) {
  if (first > second) { 
    return first; 
      } else {
    return second;
  }
}

void delay(int ms) { // blocking operation
  wait(ms, msec);
}

void velocitydrive(float left, float right) {
  leftdrive.spin(fwd, left, pct);
  rightdrive.spin(fwd, right, pct);
}

void voltagedrive(float left, float right) {
  leftdrive.spin(fwd, left, volt);
  rightdrive.spin(fwd, right, volt);
}

void stopdrive(int stopping) {
  if (stopping == 0) {
    leftdrive.stop(coast);
    rightdrive.stop(coast);
  } else if (stopping == 1) {
    leftdrive.stop(brake);
    rightdrive.stop(brake);
  } else if (stopping == 2) {
    leftdrive.stop(hold);
    rightdrive.stop(hold);
  } 
}

void drivestopping(int stopping) {
  if (stopping == 0) {
    leftdrive.setStopping(coast);
    rightdrive.setStopping(coast);
  } else if (stopping == 1) {
    leftdrive.setStopping(brake);
    rightdrive.setStopping(brake);
  } else if (stopping == 2) {
    leftdrive.setStopping(hold);
    rightdrive.setStopping(hold);
  } 
}