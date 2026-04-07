//PID CONSTANTS AND PID FUNCTIONS ARE DECLARED HERE

#include "RobotConfig.h"

// drive constants
extern double dP;
extern double dI;
extern double dD;

// turn constants
extern double tP;
extern double tI;
extern double tD;

extern int loopDelay; // delay between control loop iterations (in milliseconds)

void turnWithPID(double target);
void driveWithPID(double target);