#include "PID.h"

// NEEDS MAJOR TUNING
double tP = 0.4;
double tI = 0.1;
double tD = 0.1;

// these are for turn PID
double targetRotation = 0;  // initialize target rotation to 0
double currentRotation = 0; // initialize current rotation to 0