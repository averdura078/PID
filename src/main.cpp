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
PID turnPID(TURN, 0.4, 0.05, 0.001);

int main()
{
    Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

    while (1)
    {
        // using PID class (PIDclass.h and PIDclass.cpp)
        // movePID.moveWithPID(10); // drive forward 10 inches
        // turnPID.turnWithPID(90);  // turn to 90 degrees

        // using PID functions (PID.h and PID.cpp)
        // driveWithPID(10); // drive forward 10 inches
        // turnWithPID(90);  // turn to 90 degrees

        break; // use break if you want the robot to stop after reaching the target distance. Remove break if you want the robot to keep trying to reach the target distance (in case it gets pushed off course or something).
    }
}
