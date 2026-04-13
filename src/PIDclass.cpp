#include "PIDclass.h"

// PID class constructor
PID::PID(PIDType type, double p, double i, double d) : type(type), p(p), i(i), d(d), integral(0), previousError(0), loopDelay(10) 
// integral and previousError set to 0 so controller starts clean
// loopDelay set to 10 ms (delay between each PID loop)
{
}

void PID::moveWithPID(double target)
{
    // method body goes here
}

void PID::turnWithPID(double target)
{
    // method body goes here
}

// "Adding PID class declaration in PID.h"