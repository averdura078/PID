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

using namespace vex;

int main()
{
    Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

    while (1)
    {
        // turnWithPID(90);  // turn 90 degrees
        driveWithPID(10); // drive forward 10 inches
        break;            // use break if you want the robot to stop after reaching the target distance. Remove break if you want the robot to keep trying to reach the target distance (in case it gets pushed off course or something).
    }
}
