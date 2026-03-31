double motorPower = 0;    // initialize motor power to 0
double error = 0;         // initialize error to 0
double previousError = 0; // initialize previous error to 0
double derivative = 0;    // initialize derivative to 0
double integral = 0;      // initialize integral to 0

//drive
// NEEDS MAJOR TUNING
double dP = 0.5;
double dI = 0.0;
double dD = 0.01;

double targetDistance = 0;  // initialize target distance to 0
double currentDistance = 0; // initialize current distance to 0



//turn
// NEEDS MAJOR TUNING
double tP = 0.4;
double tI = 0.1;
double tD = 0.1;

double targetRotation = 0;  // initialize target rotation to 0
double currentRotation = 0; // initialize current rotation to 0


