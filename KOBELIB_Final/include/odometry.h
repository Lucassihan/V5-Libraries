#include "vex.h"
#include "globals.h"


class robot 
{

  private:
  double hx, hy;
  double thx, thy;

  public:
  double x, y, h, xtarg, ytarg, xtarg2, ytarg2, htarg, paralleltracker, perpindiculartracker;
  double lasth, hradians, lasthradians, lastparalleltracker, lastperpindiculartracker;
 // double SS = 7.715656571, SR = 0.0335657775;
   //double SS = 0.543498 * 1, SR = 0.01661577 * 1;
   //double SS = 4.4098215406, SR = -1.8013953114495;
   //double SS = 6.439649681528662, SR = -1.129299363057325;
   // ss sideways (x), sr forwards (y)
  //  double SS = 0.5217099034;
  //  double SR = -0.3693986229;

  // TRACKER WHEEL CONFIGURATION HERE
   double SS = txOffset;
   double SR = tyOffset;

  double PARALLEL_DIV = 360.0 / (M_PI * tyD);

  double PERP_DIV = 360.0 / (M_PI * txD);

  double R = (paralleltracker - lastparalleltracker) / PARALLEL_DIV;
  double S = (perpindiculartracker - lastperpindiculartracker) / PERP_DIV;

  double rawH = 0;
  double lastRawH = 0;


   //464.59 



  void collectdata() {
    if (IMU1.isCalibrating()/* or IMU2.isCalibrating()*/){
      h = 0, x = 0, y = 0;
    } else {
      double imuNow = IMU1.rotation();
      double delta = imuNow - lastRawH;
      lastRawH = imuNow;

      h += delta * imu_multiplier;


      // h = (IMU1.rotation()* imu_multiplier)
    // omly using 2 IMU
    }
  hradians = TO_RAD(h);
  perpindiculartracker = TX.position(deg) * 1;
  paralleltracker = TY.position(deg) * -1; // Used to use motor encoder, used to be *-1
  // paralleltracker = TY.position(deg) * -1; //USING VERTICAL ODOM
  }

  void resetData() {
    IMU1.resetRotation();
    lastRawH = IMU1.rotation();

    h = 0;  
    TX.setPosition(0, deg);
    TY.setPosition(0, deg); //USING MOTOR ENCODER
    // TY.setPosition(0, deg); //USING VERTICAL ODOM
    perpindiculartracker = 0, paralleltracker = 0; // tracking wheel inputs
    lasthradians = 0, hradians = 0; // previous robot targets
    x = 0, y = 0; // global coordinates
  }

  void setPos(float newx, float newy, float newh) {
    resetData();
    IMU1.setRotation(newh, deg);

    lastRawH = newh;
    h = newh;
    hradians = TO_RAD(newh);
    lasthradians = hradians;

    TX.setPosition(0, deg);
    TY.setPosition(0, deg); //USING MOTOR ENCODER
    // TY.setPosition(0, deg); //USING VERTICAL ODOM
    perpindiculartracker = 0, paralleltracker = 0; // tracking wheel inputs
    xtarg = x, ytarg = y, htarg = h; // robot targets
    lastperpindiculartracker = 0, lastparalleltracker = 0;
    x = newx, y = newy; // global coordinates
  }

// Odom algorithm
void updateOdom() {
  double A = hradians - lasthradians; // change in heading since last cycle
  lasthradians = hradians;

  //TRACKER WHEEL CONFIGURATION HERE
  //Y tracking wheel divide by 41.86737401 (2.75 inch) for inches (57.2957795131 for 2 inch). 
  //THIS IS DERIVED BY (360/(PI*WHEEL_DIAMETER)). IT IS A FORUMLA FOR ENCODER TICKS PER INCH

  //3.25, 48 - 36
  double R = (paralleltracker - lastparalleltracker) / PARALLEL_DIV;
  lastparalleltracker = paralleltracker;

  double S = (perpindiculartracker - lastperpindiculartracker) / PERP_DIV; // change in local X in inches since last cycle
  lastperpindiculartracker = perpindiculartracker;

  // hy; // The hypotenuse of the triangle formed by the middle of the robot on
  // the starting position and ending position and the middle of the circle it
  // travels around
  double i; // Half on the angle that I've traveled
  // hx; // The same as h but using the back instead of the side wheels

  if (A < -0.000001 or A > 0.000001) {
    double ry = R / A; // The radius of the circle the robot travel's around with the right side of the robot
    double rx = S / A; // The radius of the circle the robot travel's around with the back of the robot
    i = A / 2.0;
    double sinI = sin(i);

    hy = ((ry + SR) * sinI) * 2.0;
    hx = ((rx + SS) * sinI) * 2.0;
    thx += hx;
    thy += hy;
  } else {
    hy = R;
    i = 0;
    hx = S;
  }

  double p = i + hradians; // The global ending angle of the robot
  double cosP = cos(p);
  double sinP = sin(p);
  x += hy * sinP; // update using Y tracking wheel
  y += hy * cosP;

  x += hx * cosP; // update using the X tracking wheel
  y += hx * -sinP;
}

double pointangle() { // gets the absolute angle to the target
double ang = atan2(xtarg - x, ytarg - y);
return TO_DEG(ang);
}

double pointangle(float xtarget, float ytarget) { // gets the absolute angle 
double ang = atan2(xtarget - x, ytarget - y);
return TO_DEG(ang);
}

double wrapangle(double ang) { // bounds the input to be within 180 of 0
  while (ang < -180) {
    ang += 360;
  }
  while (ang > 180) {
    ang -= 360;
  }
  return ang;
}

float pointdist() { // calculates the distance from the robots position to the
float dist = sqrt(pow(xtarg - x, 2) + pow(ytarg - y, 2));
return dist;
}

float relativeangle() { // calculates the relative angle from the robots current
float ang = pointangle() - h;
return wrapangle(ang);
}

float length(float x1, float y1, float x2, float y2) { // finds the length of a line drawn between two points
float X = x1 - x2;
float Y = y1 - y2;
float dist = sqrt(X * X + Y * Y);
return dist;
}


float cartesianX() {
float ang2 = (((IMU1.heading()/* + IMU2.heading())/2*/ + 90)));
if (ang2 > 360) {
ang2 = ang2 - 360;
}
double a = TO_RAD(ang2);
float newx = -1 * (pointdist() * (cos(a)));
return newx;
}

float cartesianY() {
float ang2 = (((IMU1.heading()/* + IMU2.heading())/2*/ + 90)));
if (ang2 > 360) {
ang2 = ang2 - 360;
}
double a = TO_RAD(ang2);
float newy = pointdist() * (sin(a));
return newy;
}

float bcartesianX() {
float ang2 = (((IMU1.heading()/* + IMU2.heading())/2*/ - 90)));
if (ang2 > 360) {
ang2 = ang2 - 360;
}
double a = TO_RAD(ang2);
float newx = -1 * (pointdist() * (cos(a)));
return newx;
}

float bcartesianY() {
float ang2 = (((IMU1.heading()/* + IMU2.heading())/2*/ - 90)));
if (ang2 > 360) {
ang2 = ang2 - 360;
}
double a = TO_RAD(ang2);
float newy = pointdist() * (sin(a));
return newy;
}

float polarToCartesianX(float input) {
float ang2 = (((IMU1.heading()/* + IMU2.heading())/2*/ - 90)));
if (ang2 > 360) {
ang2 = ang2 - 360;
}
double a = TO_RAD(ang2);
float newx = -1 * (input * (cos(a)));
return newx;
}

float polarToCartesianY(float input) {
float ang2 = (((IMU1.heading()/* + IMU2.heading())/2*/ - 90)));
if (ang2 > 360) {
ang2 = ang2 - 360;
}
double a = TO_RAD(ang2);
float newx = -1 * (input * (sin(a)));
return newx;
}



};

extern robot chassis;