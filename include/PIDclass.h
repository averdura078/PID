#include "RobotConfig.h"

enum PIDType
{
    MOVE,
    TURN
};

class PID
{
    public:
    //PID object constructor. You can create multiple PID objects with different constants if you want (ex. one for driving straight and one for turning).
    PID(PIDType type, double p, double i, double d);

    // PID functions you can call
    void moveWithPID(double target);
    void turnWithPID(double target);

    private:
    // the type of PID controller (MOVE for going straight, TURN for turning)
    PIDType type;

    double p, i, d; // PID constants

    // these values persist across PID loop iterations, so store them in the class
    double integral;
    double previousError;

    //delay between PID loop iterations (in milliseconds)
    int loopDelay;
};