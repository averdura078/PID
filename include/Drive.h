#include "PID.h"

// NEEDS MAJOR TUNING
double dP = 0.5;
double dI = 0.0;
double dD = 0.01;

// these are for drive PID
double targetDistance = 0;  // initialize target distance to 0
double currentDistance = 0; // initialize current distance to 0