//PID CONSTANTS AND PID FUNCTIONS ARE DECLARED HERE

#include "RobotConfig.h"

// drive constants
extern double dP; // proportional constant
extern double dI; // integral constant
extern double dD; // derivative constant

// turn constants
extern double tP; // proportional constant
extern double tI; // integral constant
extern double tD; // derivative constant

extern int loopDelay; // delay between control loop iterations (in milliseconds)

// PID functions
void turnWithPID(double target);
void driveWithPID(double target);