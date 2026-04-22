/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       avery                                                     */
/*    Created:      3/3/2026, 1:47:00 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "RobotConfig.h"
#include "PID.h"
#include "PIDclass.h"

using namespace vex;

PID movePID(MOVE, 3, 0.1, 0.01);
PID turnPID(TURN, 0.4, 0.1, 0.001);

int main()
{
    Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

    if (!killSwitchActivated) // if kill switch not activated, test PID functions
    {
        // using PID class (PIDclass.h and PIDclass.cpp)
        // movePID.moveWithPID(12); // drive forward 12 inches
        // turnPID.turnWithPID(90);  // turn to 90 degrees

        // using PID functions (PID.h and PID.cpp)
        // driveWithPID(12); // drive forward 12 inches
        // turnWithPID(90); // turn to 90 degrees
    }

    while (1)
    {
    }
}