#include <math.h>

double TO_RAD(double n); // converts the input to radians

double TO_DEG(double n); // converts the input to degrees

int signum(float input); // returns the sign of an input

float max(float first, float second); // returns the bigger of 2 inputs

float min(float first, float second); // returns the smaller of 2 inputs

void delay(int ms); // delays the code from executing

void velocitydrive(float left, float right);

void voltagedrive(float left, float right);

void stopdrive(int stopping);

void drivestopping(int stopping);
