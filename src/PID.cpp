// PID CONSTANTS AND PID FUNCTIONS ARE DEFINED HERE

#include "PID.h"

using namespace vex;

// drive constants (tune them here)
double dP = 0.5;  // proportional constant
double dI = 0.01; // integral constant
double dD = 0.01; // derivative constant

// turn constants (tune them here)
double tP = 0.4; // proportional constant
double tI = 0.1; // integral constant
double tD = 0.1; // derivative constant

// driveWithPID constants (CHANGE THESE FOR YOUR ROBOT)
double gearRatio = 2.0; // gear ratio of your drivetrain (used in driveWithPID function)
// if you have a complex compound drivetrain, you may choose to calculate this experimentally through trial and error:
// code your robot to drive a distance (ex. 12 inches) using driveWithPID, manually measure how far it goes, and adjust this ratio until it is accurate.
double wheelDiameter = 3.25; // diameter of your wheels in inches (used in driveWithPID function)

int loopDelay = 10; // delay between PID loop iterations (in milliseconds)

void driveWithPID(double target)
{
    double targetDistance = target; // set the target distance to the target (the number you pass it when using this function)
    double previousError = 0;       // this is used to save the error from the previous loop iteration so we can calculate the derivative (the change in error) in the current loop iteration.
    double integral = 0;            // initialize integral (sum of past errors over time) to 0
    while (true)
    {        
        // calculate the average rotation of the left and right motors (in degrees)
        double leftDegrees = leftTop.position(rotationUnits::deg);
        double rightDegrees = rightTop.position(rotationUnits::deg);
        double currentMotorRotation = (leftDegrees + rightDegrees) / 2.0;

        // convert motor degrees to wheel degrees
        double wheelRotation = currentMotorRotation * gearRatio;

        // convert wheel degrees to distance traveled by the robot
        double currentDistance = (M_PI * wheelDiameter) * wheelRotation / 360.0; // wheelDiameter is the diameter of the wheel, so this formula calculates the distance traveled by the robot based on how much the wheel has rotated.
        // The formula is circumference * (π * diameter) * (rotation in degrees / 360), which gives us the distance traveled in inches.

        // P
        double error = targetDistance - currentDistance; // calculate error as the difference between target distance and current distance

        // D
        double derivative = (error - previousError) / (loopDelay / 1000.0); // compute derivative as the change in error over time since the last loop iteration.

        // I
        integral += error * (loopDelay / 1000.0); // compute integral as the accumulation of error as time passes (in seconds)

        // clamp integral term to prevent integral windup (when the integral term accumulates too much error and causes the robot to overshoot the target)
        if (integral > 200)
            integral = 200;
        if (integral < -200)
            integral = -200;

        // calculate motor power using PID control formula
        double motorPower = dP * error + dI * integral + dD * derivative;

        // clamp to prevent motor burnout (don't let motors spin at more than 90% power)
        if (motorPower > 90)
            motorPower = 90;
        if (motorPower < -90)
            motorPower = -90;

        // make the robot drive
        LeftDrive.spin(forward, motorPower, percentUnits::pct);
        RightDrive.spin(forward, motorPower, percentUnits::pct);

        // stop if the error is less than or equal to 0.1 inches (you may want to adjust this threshold)
        if (fabs(error) <= 0.1)
        { // stop condition
            LeftDrive.stop();
            RightDrive.stop();
            break;
        }

        previousError = error; // save the current error for the next loop iteration

        // Use this for troubleshooting. May use brain screen instead and check other values too.
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Current Distance: %f", currentDistance);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Error: %f", error);

        // allow other tasks to run
        this_thread::sleep_for(loopDelay);
    }
}

void turnWithPID(double target)
{
    double targetRotation = target; // set the target distance to the target (the number you pass it when using this function)
    double previousError = 0;       // this is used to save the error from the previous loop iteration so we can calculate the derivative (the change in error) in the current loop iteration.
    double integral = 0;            // initialize integral (sum of past errors over time) to 0
    while (true)
    {
        double currentRotation = Drivetrain.rotation(vex::rotationUnits::deg); // measure current drivetrain rotation (uses inertial sensor)

        // P
        double error = targetRotation - currentRotation; // calculate error as the difference between target rotation and current rotation

        // D
        double derivative = (error - previousError) / (loopDelay / 1000.0); // compute derivative as the change in error over the change in timesince the last loop iteration.

        // I
        integral += error * (loopDelay / 1000.0); // compute integral as the accumulation of error as time passes (in seconds)

        // clamp integral term to prevent integral windup (when the integral term accumulates too much error and causes the robot to overshoot the target)
        if (integral > 200)
            integral = 200;
        if (integral < -200)
            integral = -200;

        // calculate motor power using PID control formula
        double motorPower = tP * error + tI * integral + tD * derivative;

        // clamp to prevent motor burnout (don't let motors spin at more than 90% power)
        if (motorPower > 90)
            motorPower = 90;
        if (motorPower < -90)
            motorPower = -90;

        // make the robot spin
        LeftDrive.spin(forward, motorPower, percentUnits::pct);
        RightDrive.spin(forward, -motorPower, percentUnits::pct);

        // prevent oscillation around the target
        if (fabs(error) <= 0.3) // stop if the error is less than or equal to 0.3 degrees (you may want to adjust this threshold)
        {
            LeftDrive.stop();
            RightDrive.stop();
            break; // break out of the while loop if close enough
        }

        previousError = error; // save the current error for the next loop iteration

        // Use this for troubleshooting. May use brain screen instead and check other values too.
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Current Rotation: %f", currentRotation);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Error: %f", error);

        // allow other tasks to run
        this_thread::sleep_for(loopDelay);
    }
}