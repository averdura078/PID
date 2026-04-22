Read this document to learn the basics of PID control, what is going on in this project, how to implement it into your own project, and how to tune your PID.



***Basics of PID Control***:
A PID is used to accurately get your robot to a target distance or angle. There are 3 components of a PID: Proportional, Integral, and Derivative.
At the most basic level, the amount of power your motors receive depends on 3 things: how far you are from the target (P), how fast you’re approaching it (D), and how far off you’ve been over time (I).

**Simple Math**: 
motorPower = P + I + D
- Proportional control
    - Responsible for getting the robot to the target (P is the main component of the PID)
    - Based on size of error
- Integral control
    - Responsible for eliminating leftover error over time
    - Based on the sum of past errors
- Derivative control
    - Responsible for "braking" (preventing overshoot of a target)
    - Based on how quickly the error is changing

**Actual Math** (you will see this code logic / general structure in this code):
- We define kP, kI, and kD as the proportional, integral, and derivative constants we will adjust in tuning (k is to indicate constant (unchanging) value)
    - If using PID.h and PID.cpp, our turn constants are tP, tI, and tD, and our drive constants are dP, dI, and dD.
    - If using PIDclass.h and PIDclass.cpp, turn and drive constants are p, i, and d.
- The proportional, integral, and derivative terms of motorPower are obtained by multiplying the thing each term depends on by the respective constant
- We add these 3 terms to get motorPower: 
motorPower = proportional term + integral term + derivative term 
motorPower = kP * error + kI * integral + kD * derivative;

**How each component is calculated**:
P
- proportional = error 
- proportional term = kP * error (this means the product of the proportional constant and the error).
- the proportional term reacts to the error: bigger error = more motor power

Example of proportional function
If the robot is far from the target, it moves fast. As it approaches, it begins going slower. The farther it is, the more power it gets.

I
- integral += error * ∆t (this means integral equals current value of integral plus the product of current error and time passed IN SECONDS).
- integral term = kI * integral
- the integral term reacts to the robot being slightly behind a target for a period of time, and as it accumulates, more power is given to eliminate the error.

Example of integral function
If you program the robot to spin to 90 degrees but it stops at 89.5, the integral “remembers” that the robot has been behind the target for a while, gradually adding more power until the error disappears.

D
- derivative = d(error)/dt (this means the derivative of the error over the derivative of the time, or the change in error over the change in time). Simplified: derivative = (error - previousError) / ∆t (Note: Change in time IN SECONDS).
- derivative term = kD * derivative
- the derivative term reacts to the robot going too fast as it approaches the target and slows it down, preventing it from going too far.

Example of derivative function
If you program the robot to spin to 90 degrees, and it is moving too quickly as it approaches, the derivative “brakes” to prevent overshooting.



***HOW TO IMPLEMENT***
2 ways:
- using PID.h and PID.cpp
    - easiest
    - less flexible (cannot use PID with anything besides your drivetrain)
- using PIDclass.h and PIDclass.cpp
    - slightly harder
    - more flexible (may create multiple instances of the PID to use with your drivetrain, intake, arm, etc.)

**How to Implement (using PID.h and and PID.cpp)**
1. Create the files you are missing
    - You are definitely missing PID.h and PID.cpp. Create a file called PID.h under the "include" folder and create a file called PID.cpp under the "src" folder.
    - You might be missing RobotConfig.h and RobotConfig.cpp. Create a file called RobotConfig.h under the "include" file and create a file called RobotConfig.cpp under the "src" folder.
    - You do not need to add any files with extension .md. These files are for humans only and do not impact the code. You may choose to add README.md and its contents to your project purely for convenience as you figure out your PID system.

2. Copy and paste the content from this project's files into the respective files you just created. Then edit for your robot.
    - You need to follow the demonstrated structure in RobotConfig.h and RobotConfig.cpp but edit it for your specific motors and ports. Note: if you change the names of or delete motors, ensure consistency throughout the project. You may need to edit driveWithPID() to reset and read from 2 motors on either side of your drivetrain that you defined. 
    - You need to change gearRatio and wheelDiameter in PID.cpp to match your robot.

3. IMPORTANT: include the header files vex.h, RobotConfig.h, and PID.h at the top of main.cpp and any other files you wish to access the PID system in, and ensure you are using the vex namespace. (copy and paste the following code to your main.cpp)

#include "vex.h"
#include "RobotConfig.h"
#include "PID.h"

using namespace vex;

4. Ensure you are calibrating the inertial sensor at the beginning of your main loop (copy and paste the following code to the top of your main loop)

Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

5. Use the turn PID in main.cpp like this: turnWithPID(90); (Units in degrees).
   Use the drivePID in main.cpp like this: driveWithPID(12); (Units in inches).
   ONLY TEST AND TUNE ONE AT ONCE BY UNCOMMENTING ONE THESE LINES AT A TIME IN MAIN.CPP.

6. Tune your constants on PID.cpp following the instructions below ("Step-By-Step Tuning Checklist" and "Tips for Troubleshooting / Perfecting Constants)

Alternatively, if you are starting from scratch (you don't have your own project already):
1. Ignore all 6 steps above
2. Make a copy of this project
3. Edit RobotConfig.h (using instructions on that page), RobotConfig.cpp (using instructions on that page) for your robot
4. Edit PID.cpp (using instructions on that page)
5. For how to tune: reference instructions below ("Step-By-Step Tuning Checklist" and "Tips for Troubleshooting / Perfecting Constants")



**How to Implement (using PIDclass.h and and PIDclass.cpp)**
1. Create the files you are missing
    - You are definitely missing PIDclass.h and PIDclass.cpp. Create a file called PIDclass.h under the "include" folder and create a file called PIDclass.cpp under the "src" folder.
    - You might be missing RobotConfig.h and RobotConfig.cpp. Create a file called RobotConfig.h under the "include" file and create a file called RobotConfig.cpp under the "src" folder.
    - You do not need to add any files with extension .md. These files are for humans only and do not impact the code. You may choose to add README.md and its contents to your project purely for convenience as you figure out your PID system.

2. Copy and paste the content from this project's files into the respective files you just created. Then edit for your robot.
    - You need to follow the demonstrated structure in RobotConfig.h and RobotConfig.cpp but edit it for your specific motors and ports. Note: if you change the names of or delete motors, ensure consistency throughout the project. You may need to edit driveWithPID() to reset and read from 2 motors on either side of your drivetrain that you defined. 
    - You need to change gearRatio and wheelDiameter in PID.cpp to match your robot.

3. IMPORTANT: include the header files vex.h, RobotConfig.h, and PIDclass.h at the top of main.cpp and any other files you wish to access the PID system in, and ensure you are using the vex namespace. (copy and paste the following code to your main.cpp)

#include "vex.h"
#include "RobotConfig.h"
#include "PIDclass.h"

using namespace vex;

4. Ensure you are calibrating the inertial sensor at the beginning of your main loop (copy and paste the following code to the top of your main loop)

Inertial.calibrate(); // calibrate the inertial sensor
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
    }

5. Define your PID objects like this (you will need 2 for a drivetrain, a move object (for linear motion) and a turn object (for turning)): 
    PID movePID(MOVE, 3, 0.1, 0.01); <-- create a PID object called movePID. MOVE tells it it's type. 
                                            The numbers are the p, i and d constants.
    PID turnPID(TURN, 0.4, 0.1, 0.001); <-- create a PID object called turnPID. TURN tells it it's type. 
                                            The numbers are the p, i and d constants.

    Now you have a complete PID for your drivetrain.

5. Use the turnPID you just created in main.cpp like this: turnPID.turnWithPID(90); (Units in degrees).
   Use the movePID you just created in main.cpp like this: movePID.moveWithPID(12); (Units in inches).
   ONLY TEST AND TUNE ONE AT ONCE BY UNCOMMENTING ONE THESE LINES AT A TIME IN MAIN.CPP.

6. Tune your constants in the line where you define your PID object in main.cpp following the instructions below ("Step-By-Step Tuning Checklist" and "Tips for Troubleshooting / Perfecting Constants)

7. When we use a PID class, we can define multiple PID objects or "instances" of the class. We are not limited to a single drivetrain PID anymore. If you want to use this PID class with other parts of your robot, for example an intake, arm, etc., do it! Name them something different and repeat steps 5-6 as many times as you want. IMPORTANT: Depending on what you are using it for and how different it is from the classic drivetrain, you may need to edit the class and contents of its functions directly. 

Alternatively, if you are starting from scratch (you don't have your own project already):
1. I recommend beginning with using functions rather than classes, as it is a gentler learning curve.
2. Go to this set of instructions above: "How to Implement (using PID.h and and PID.cpp)"



***How to Tune***
**Step-By-Step Tuning Checklist**
1. Set kI and kD to 0.
2. Increase kP until the robot slightly oscillates around the target and / or goes slightly too far.
3. Increase kD to dampen the oscillation / overshoot.
4. Add a tiny amount of kI to fix steady-state error (if it stops just short of the target and stays there).

**Tips for Troubleshooting / Perfecting Constants**
p
Bigger p constant = faster gain, Smaller p constant = slower gain. 
If the robot is oscillating around the target distance, try reducing this value. If the robot is not reaching the target distance, try increasing this value.

d
Bigger d constant = stronger braking effect. Smaller d constant = weaker braking effect.
If the robot overshoots the target, try increasing this value. If it stops too early, try decreasing this value.

i
Bigger i constant = stronger correction for leftover error. Smaller i constant = weaker correction.
If the robot consistently stops short of the target, try increasing this value. If it overshoots or oscillates, try decreasing this value.

*NOTE: any generative AI including GitHub Copilot which is built into Visual Studio Code can be helpful for tuning your PID and troubleshooting any other errors you run into! GOOD LUCK AND HAPPY CODING!*


