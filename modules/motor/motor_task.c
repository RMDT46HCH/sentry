#include "motor_task.h"
#include "dji_motor.h"
#include "servo_motor.h"
#include "bsp_dwt.h"
static uint8_t cnt = 0; //设定不同电机的任务频率

void MotorControlTask()
{
    cnt=DWT_GetTimeline_s();
    if(cnt%10==0) //100hz
    DJIMotorControl();
    if(cnt%50==0) //20hz
    ServeoMotorControl();
}
