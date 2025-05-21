//app
#include "chassis.h"
#include "robot_def.h"

//module
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "ins_task.h"
#include "general_def.h"
#include "buzzer.h"
#include "rm_referee.h"
#include "referee_task.h"
#include "Board2Board.h"

//bsp
#include "bsp_dwt.h"
#include "arm_math.h"
/************************************** CommUsed **************************************/
static BoardCommInstance *chassis_can_comm;               // 双板通信CAN comm
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;             // 底盘接收到的控制命令（发布中心发给底盘的）
static Chassis_Upload_Data_s chassis_feedback_data;     // 底盘回传的反馈数据
static Referee_Interactive_info_t ui_data;              // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static referee_info_t* referee_data;                    // 用于获取裁判系统的数据

/*********************************** CalculateSpeed ***********************************/
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // 四个轮子的实例
static Cal_Chassis_Info_t chassis_info;                             // 底盘速度计算信息

void ChassisInit()
{
/***************************MOTOR_INIT******************************/
    // 底盘电机的初始化，包括什么通信、什么id、pid及电机安装是正装还是反装（相当于给最终输出值添负号）及型号
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = 
            {
                .Kp = 6, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = 
            {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },

        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //电机id号一定一定得一一对应
    chassis_motor_config.can_init_config.tx_id = 0x201;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);
    
    chassis_motor_config.can_init_config.tx_id = 0x202;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 0x204;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 0x203;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

/************************************** RefereeCommInit **************************************/
    referee_data   = UITaskInit(&huart6,&ui_data);

/************************************** ChassisCommInit **************************************/
    //双板通信
    BoardComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            //云台的tx是底盘的rx，别搞错了！！！
            .tx_id = 0x209,
            .rx_id = 0x200,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chassis_can_comm = BoardCommInit(&comm_conf); // can comm初始化
}

/*****************************************MoveChassis********************************************/
static void ChassisStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
}

/**
 * @brief  
 */
static void ChassisRotateSet()
{
    chassis_info.cnt = (float32_t)DWT_GetTimeline_s();//用于变速小陀螺
    switch (chassis_cmd_recv.chassis_mode)
    {
        case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动
            chassis_cmd_recv.wz = 0;
        break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // (底盘跟随云台)（系数待调）
            chassis_cmd_recv.wz = -0.05*chassis_cmd_recv.offset_angle*abs(chassis_cmd_recv.offset_angle);
        break;
        case CHASSIS_ROTATE: // 变速小陀螺
            chassis_cmd_recv.wz = (1200+100*(float32_t)sin(chassis_info.cnt))*4.75;
        break;
        case CHASSIS_NAV:
            chassis_cmd_recv.wz = (1200+100*(float32_t)sin(chassis_info.cnt))*4.75*chassis_cmd_recv.w/100;
        break;

        default:
        break;
    }
}

/**
 * @brief 计算每个底盘电机的输出,底盘正运动学解算
 *                                
 */
static void MecanumCalculate()
{   
    chassis_info.cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_info.sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_info.chassis_vx = chassis_cmd_recv.vx * chassis_info.cos_theta - chassis_cmd_recv.vy * chassis_info.sin_theta; 
    chassis_info.chassis_vy = chassis_cmd_recv.vx * chassis_info.sin_theta + chassis_cmd_recv.vy * chassis_info.cos_theta;

    chassis_info.vt_lf = chassis_info.chassis_vx - chassis_info.chassis_vy - chassis_cmd_recv.wz ;
    chassis_info.vt_lb = chassis_info.chassis_vx + chassis_info.chassis_vy - chassis_cmd_recv.wz ;
    chassis_info.vt_rb = chassis_info.chassis_vx - chassis_info.chassis_vy + chassis_cmd_recv.wz ;
    chassis_info.vt_rf = chassis_info.chassis_vx + chassis_info.chassis_vy + chassis_cmd_recv.wz ;
}

/**
 * @brief
 *
 */
static void ChassisOutput()
{
    DJIMotorSetRef(motor_lf, chassis_info.vt_lf);
    DJIMotorSetRef(motor_rf, chassis_info.vt_rf);
    DJIMotorSetRef(motor_lb, chassis_info.vt_lb);
    DJIMotorSetRef(motor_rb, chassis_info.vt_rb);
}

/*****************************************SendData********************************************/
/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算，并发给巡航底盘实时数据             
 */
static void SendChassisData()
{
    //to 巡航
    chassis_info.vx = (motor_lf->measure.speed_aps +motor_lb->measure.speed_aps - motor_rb->measure.speed_aps - motor_rf->measure.speed_aps) / 4.0f / REDUCTION_RATIO_WHEEL / 360.0f * PERIMETER_WHEEL/1000 ;
    chassis_info.vy = (-motor_lf->measure.speed_aps +motor_lb->measure.speed_aps + motor_rb->measure.speed_aps - motor_rf->measure.speed_aps) / 4.0f / REDUCTION_RATIO_WHEEL / 360.0f * PERIMETER_WHEEL/1000  ;
    chassis_feedback_data.real_vx = chassis_info.vx * chassis_info.cos_theta + chassis_info.vy * chassis_info.sin_theta;
    chassis_feedback_data.real_vy = -chassis_info.vx * chassis_info.sin_theta + chassis_info.vy * chassis_info.cos_theta;
}

/**
 * @brief  将裁判系统的信息发给巡航、视觉让其进行决策。
 */
static void SendJudgeData()
{
    chassis_feedback_data.Occupation=(referee_data->EventData.event_type >> 21) & 0x03;
    chassis_feedback_data.remain_time=referee_data->GameState.stage_remain_time;
    chassis_feedback_data.game_progress=referee_data->GameState.game_progress;
    
    if(referee_data->GameRobotState.robot_id>7)
    {
        chassis_feedback_data.enemy_color=COLOR_RED;

        chassis_feedback_data.remain_HP=referee_data->GameRobotHP.blue_7_robot_HP;
        chassis_feedback_data.self_hero_HP=referee_data->GameRobotHP.blue_1_robot_HP;
        chassis_feedback_data.self_infantry_HP=referee_data->GameRobotHP.blue_3_robot_HP;

        chassis_feedback_data.enemy_hero_HP=referee_data->GameRobotHP.red_1_robot_HP;
        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.red_3_robot_HP;
        chassis_feedback_data.enemy_sentry_HP=referee_data->GameRobotHP.red_7_robot_HP;
    }
    else
    {
        chassis_feedback_data.enemy_color=COLOR_BLUE;
        chassis_feedback_data.remain_HP=referee_data->GameRobotHP.red_7_robot_HP;
        chassis_feedback_data.self_hero_HP=referee_data->GameRobotHP.red_1_robot_HP;
        chassis_feedback_data.self_infantry_HP=referee_data->GameRobotHP.red_3_robot_HP;

        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.blue_1_robot_HP;
        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.blue_3_robot_HP;
        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.blue_7_robot_HP;
    }   
    chassis_feedback_data.left_bullet_heat= referee_data->PowerHeatData.shooter_17mm_2_barrel_heat;
    chassis_feedback_data.right_bullet_heat= referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    chassis_feedback_data.bullet_num=referee_data->ProjectileAllowance.projectile_allowance_17mm;
    chassis_feedback_data.bullet_speed=referee_data->ShootData.bullet_speed;
}

/*********************************************************************************************************
 *********************************************      TASK     *********************************************
**********************************************************************************************************/
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
/********************************************   GetRecvData  *********************************************/ 
    // 获取新的控制信息
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)BoardCommGet(chassis_can_comm);
/****************************************     ControlChassis     *****************************************/
    //底盘动与不动
    ChassisStateSet();
    //旋转模式及速度设定
    ChassisRotateSet();
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系
    // 根据控制模式进行正运动学解算,计算底盘各个电机的速度
    MecanumCalculate();
    // 根据裁判系统的反馈数据设定闭环参考值
    ChassisOutput();
 /*******************************************     SendData     ********************************************/
    //将裁判系统的信息发给巡航，让其进行决策。
    SendJudgeData();
    // 根据电机的反馈速度计算真实速度发给巡航
    SendChassisData(); 
    BoardCommSend(chassis_can_comm, (void *)&chassis_feedback_data);
}