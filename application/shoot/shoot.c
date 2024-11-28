#include "shoot.h"
#include "robot_def.h"
#include "servo_motor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "rm_referee.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static cal_bullet_t cal_bullet;
static ServoInstance *change;

void ShootInit()
{
    // 左摩擦轮
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
    friction_config.can_init_config.tx_id = 2,
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 1; // 右摩擦轮
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 10
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

    //初始化参数
    Servo_Init_Config_s config= {
        .htim=&htim1,
        .Channel=TIM_CHANNEL_1,
        //舵机的初始化模式和类型
        .Servo_Angle_Type=Free_Angle_mode,
        .Servo_type=Servo270,
    };
    // 设置好参数后进行初始化并保留返回的指针
    change = ServoInit(&config);


    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    memset(&cal_bullet,0,sizeof(cal_bullet));
}


static void CalHeat()
{
    // 记录初始情况下的电机角度和时间
    if (cal_bullet.first_flag == 0)
    {
        cal_bullet.last_time = DWT_GetTimeline_ms();
        cal_bullet.last_loader_total_heat_angle = loader->measure.init_total_angle;
        cal_bullet.shoot_heat_l = 0;
        cal_bullet.shoot_heat_r = 0;
        cal_bullet.first_flag = 1;
        cal_bullet.shoot_l = 1;
        cal_bullet.shoot_r = 0;
        cal_bullet.rest_bullet = TOTAL_BULLET;
        Servo_Motor_FreeAngle_Set(change,40);
    }

    // 计算实际发弹数以及剩余弹量
    if ((loader->measure.total_angle - cal_bullet.last_loader_total_angle) >= ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER && cal_bullet.rest_bullet > 0)
    {
        cal_bullet.rest_bullet -= (loader->measure.total_angle - cal_bullet.last_loader_total_angle) / (ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
        cal_bullet.last_loader_total_angle = loader->measure.total_angle;
    }

    // 以10Hz每秒减少、增加热量
    if (DWT_GetTimeline_ms() - cal_bullet.last_time >= 100)
    {
            if (cal_bullet.shoot_heat_l >= COOLING_VAL / 10)
            {
                cal_bullet.shoot_heat_l -= COOLING_VAL / 10;
            }
            else
            {
                cal_bullet.shoot_heat_l = 0;
            }
            if (cal_bullet.shoot_heat_r >= COOLING_VAL / 10)
            {
                cal_bullet.shoot_heat_r -= COOLING_VAL / 10;
            }
            else
            {
                cal_bullet.shoot_heat_r = 0;
            }
        cal_bullet.last_time = DWT_GetTimeline_ms();

        // 每发射1发小弹丸加10点热量
        if ((loader->measure.total_angle - cal_bullet.last_loader_total_heat_angle) >= 1080)
        {
            if (cal_bullet.shoot_l == 1)
            {
                cal_bullet.shoot_heat_l += ((loader->measure.total_angle - cal_bullet.last_loader_total_heat_angle) / 1080.0) * 10;
            }
            else if (cal_bullet.shoot_r == 1)
            {
                cal_bullet.shoot_heat_r += ((loader->measure.total_angle - cal_bullet.last_loader_total_heat_angle) / 1080.0) * 10;
            }
            cal_bullet.last_loader_total_heat_angle = loader->measure.total_angle;
        }
    }

    // 限幅 + 冷却范围整定
    if (cal_bullet.shoot_heat_l > 2 * HEAT_LIMIT)
    {
        cal_bullet.shoot_heat_l = 2 * HEAT_LIMIT;
    }
    else if (cal_bullet.shoot_heat_l <= 0)
    {
        cal_bullet.shoot_heat_l = 0;
    }

    if (cal_bullet.shoot_heat_r > 2 * HEAT_LIMIT)
    {
        cal_bullet.shoot_heat_r = 2 * HEAT_LIMIT;
    }
    else if (cal_bullet.shoot_heat_r <= 0)
    {
        cal_bullet.shoot_heat_r = 0;
    }

    // 切换枪管
    if (cal_bullet.shoot_heat_l > HEAT_LIMIT && cal_bullet.shoot_l == 1)
    {
        // 切换到右枪管
        Servo_Motor_FreeAngle_Set(change,130);
        cal_bullet.shoot_l = 0;
        cal_bullet.shoot_r = 1;
        if(cal_bullet.shoot_heat_r > CHANGE_LIMIT)
        {
            shoot_feedback_data.over_heat_flag=1;
        }
    }
    else if (cal_bullet.shoot_heat_r > HEAT_LIMIT && cal_bullet.shoot_r == 1)
    {
        // 切换到左枪管
        Servo_Motor_FreeAngle_Set(change,40);
        cal_bullet.shoot_l = 1;
        cal_bullet.shoot_r = 0;
        if(cal_bullet.shoot_heat_l > CHANGE_LIMIT)
        {
            shoot_feedback_data.over_heat_flag=1;
        }
    }
    else if(cal_bullet.shoot_heat_l<RECOVER_HEAT_LIMIT||cal_bullet.shoot_heat_r < RECOVER_HEAT_LIMIT)
    {
        shoot_feedback_data.over_heat_flag=0;
    }
}
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
    // 拨盘反转,对速度闭环（待测试）
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, -1000);
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
}


/*

/**
 * @brief  已实测，发射速度快了就让他慢下来
 */
static void ShootSpeedSet()
{
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        DJIMotorSetRef(friction_l, 26000);
        DJIMotorSetRef(friction_r, 26000);
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
}

static void SendShootData()
{
    shoot_feedback_data.loader_speed_aps=loader->measure.speed_aps;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    //电机是否上电
    ShootStateSet();
    //发射模式设定
    ShootLoaderSet();
    //射速设定
    ShootSpeedSet();
    //计算热量并限制
    CalHeat();
    //给发布中心电机实际情况，从而调节拨盘电机的模式
    SendShootData();
    // 反馈数据,用于卡弹反馈（后续再加个模块离线）
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}