#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#include "referee_protocol.h"
/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
//#define CHASSIS_BOARD //底盘板
#define GIMBAL_BOARD  //云台板

 #define VISION_USE_UART // 使用串口发送视觉数据

 #define ODOM_USE_UART // 使用串口发送巡航数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2900  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 1 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 1340.0f      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE  20          // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -10           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 30    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 12            // 拨盘一圈的装载量
#define TOTAL_BULLET 750             //总载弹量
#define COOLING_VAL 80
#define HEAT_LIMIT 380
#define RECOVER_HEAT_LIMIT 200
#define CHANGE_LIMIT 300

// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 390              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 390             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 76.25f         // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换


// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转

    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制（哨兵的是100w，感觉可以不用）
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;
/* ----------------用于记录时间或标志位的结构体---------------- */
typedef struct
{
    float t_shoot;
    float t_pitch;
    float t_cmd_error;
    uint8_t vision_flag;
    uint8_t shoot_flag;
    uint8_t cmd_error_flag;
    uint8_t fire_flag;
}DataLebel_t;
/* ----------------用于计算热量用到的的结构体---------------- */

typedef struct
{
    float last_loader_total_angle;//用于储存上一次计算时角度
    float last_loader_total_heat_angle;//用于储存上一次计算热量时用到的角度
    float loader_total_angle;//当前角度
    uint8_t first_flag;//用于读取初始化电机角度
    float rest_bullet;
    uint16_t shoot_heat_l;//枪口当前热量(计算值)
    uint16_t shoot_heat_r;//枪口当前热量(计算值)

    float heat_k;//控制热量系数
    //读取裁判系统时间间隔
    float last_time;
    float last_bullet_speed;
    uint8_t shoot_l;
    uint8_t shoot_r;
}cal_bullet_t;
/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;    
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    float fric_rate;
    float bullet_real_speed;

} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
    //发给巡航的数据
     attitude_t chassis_imu_data;
     float real_vx;
     float real_vy;
     float real_wz;
     uint16_t remain_HP;
     //发给视觉的数据
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red
    Vision_Work_Mode_e vision_work_mode;
    //发给云台的数据
    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    uint8_t cmd_error_flag;
} Chassis_Upload_Data_s;


typedef struct
{
    //发给视觉的数据
    attitude_t gimbal_imu_data;
    uint16_t   yaw_motor_single_round_angle;
    uint8_t cmd_error_flag;
} Gimbal_Upload_Data_s;

typedef struct
{
    uint8_t over_heat_flag;
    float loader_speed_aps;
    uint8_t cmd_error_flag;

} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H