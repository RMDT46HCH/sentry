#include "motor_task.h"
#include "dji_motor.h"
#include "servo_motor.h"

void MotorControlTask()
{
    DJIMotorControl();
    ServeoMotorControl();
}
