#include "shoot.h"
#include "robot_def.h"
#include "servo_motor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "rm_referee.h"

/************************************* ShootUsed *************************************/
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
static ServoInstance *change;
/************************************** CommUsed **************************************/
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

/************************************** HeatUsed **************************************/
static cal_bullet_t cal_bullet;

void ShootInit()
{
/************************************** MotorInit **************************************/
    // 摩擦轮电机的初始化，设置can通信如句柄、id，设置pid，及电机安装是正转还是反转（相当于给最终输出值添负号）及电机型号
    Motor_Init_Config_s friction_config = {
        .can_init_config = 
        {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 
    };
    loader_config.can_init_config.tx_id=3;
    loader = DJIMotorInit(&loader_config);
/************************************** ServoInit **************************************/
    //舵机初始化，用于切换枪管
    Servo_Init_Config_s config= {
        .htim=&htim1,
        .Channel=TIM_CHANNEL_1,
        //舵机的初始化模式和类型
        .Servo_Angle_Type=Free_Angle_mode,
        .Servo_type=Servo270,
    };
    // 设置好参数后进行初始化并保留返回的指针
    change = ServoInit(&config);

/************************************** ShootCommInit **************************************/
    //创建发布、订阅者
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
/**************************************    HeatInit    *************************************/
    memset(&cal_bullet,0,sizeof(cal_bullet));
    cal_bullet.shoot_l=1;
    cal_bullet.shoot_r=0;
}


/*********************************************************************************************
***************************************      Function      ***********************************
**********************************************************************************************/

/**************************************   CalculateHeat   *************************************/
static void CalHeat()
{
    //切换到右枪管
    if(cal_bullet.shoot_l == 0&&cal_bullet.shoot_r == 1)
    {
        Servo_Motor_FreeAngle_Set(change,135);
        //先等1s，等切换枪管完再发射
        if(DWT_GetTimeline_s()>1)
        shoot_feedback_data.over_heat_flag=0;
    }
    //切换到左枪管
    else if(cal_bullet.shoot_l == 1&&cal_bullet.shoot_r == 0)
    {
        Servo_Motor_FreeAngle_Set(change,32);
        if(DWT_GetTimeline_s()>1)
        shoot_feedback_data.over_heat_flag=0;
    }
    //超热量时换枪管
    if(shoot_cmd_recv.left_bullet_heat>HEAT_LIMIT &&cal_bullet.shoot_l==1)
    {
        shoot_feedback_data.over_heat_flag=1;
        if(DWT_GetTimeline_s()>1)
        {
            // 切换到右枪管
            cal_bullet.shoot_l = 0;
            cal_bullet.shoot_r = 1;
        }
        //两个枪管热量均满，超热量
        if(cal_bullet.shoot_heat_r > CHANGE_LIMIT)
        {
            shoot_feedback_data.over_heat_flag=1;
        }
    }
        
    else if (shoot_cmd_recv.right_bullet_heat > HEAT_LIMIT && cal_bullet.shoot_r == 1)
    {
        shoot_feedback_data.over_heat_flag=1;
        if(DWT_GetTimeline_s()>1)
        {
            // 切换到左枪管
            cal_bullet.shoot_l = 1;
            cal_bullet.shoot_r = 0;
        }

        if(cal_bullet.shoot_heat_l > CHANGE_LIMIT)
        {
            shoot_feedback_data.over_heat_flag=1;
        }
    }
    //让枪管恢复正常
    else if(cal_bullet.shoot_heat_l<RECOVER_HEAT_LIMIT||cal_bullet.shoot_heat_r < RECOVER_HEAT_LIMIT)
    {
        shoot_feedback_data.over_heat_flag=0;
    }
}
/***************************************** MoveShoot ********************************************/
/**
 * @brief 整个发射机构状态设置
 */
static void ShootStateSet()
{
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }
}

/**
 * @brief 拨盘状态设置及转速设定
 */

static void ShootLoaderSet()
{
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 4);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度
        break;
    // 拨盘反转,对速度闭环
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, -4000);
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
}

/**
 * @brief   摩擦轮状态设置及射速设定
 */
static void ShootSpeedSet()
{
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        DJIMotorSetRef(friction_l, 32000);
        DJIMotorSetRef(friction_r, 32000);
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
}

/**************************************    SendData    *************************************/
static void SendShootData()
{
    shoot_feedback_data.speed=loader->measure.speed_aps;
}

/************************************************************************************************
***************************************        TASK        **************************************
*************************************************************************************************/
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
/**********************************     GetRecvData     ***********************************/
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
/**********************************     ControlShoot     ***********************************/
    //电机是否上电
    ShootStateSet();
    //发射模式设定
    ShootLoaderSet();
    //射速设定
    ShootSpeedSet();
/**********************************     CalculateHeat     ***********************************/
    //计算热量并进行枪管切换
    CalHeat();
/**********************************        SendData        ***********************************/
    //给发布中心电机实际情况，从而调节拨盘电机的模式
    SendShootData();
    // 反馈数据,用于卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}