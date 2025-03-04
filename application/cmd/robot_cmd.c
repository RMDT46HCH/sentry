// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
#include "rm_referee.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信

static  BuzzzerInstance *cmd_error_buzzer;
static  BuzzzerInstance *aim_success_buzzer;

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回

static Minipc_Recv_s *minipc_recv_data; //小电脑接收数据指针,初始化时返回
static Minipc_Send_s  minipc_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static DataLebel_t DataLebel;
static float cnt1,cnt2;
#include "can_comm.h"

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 遥控器通信串口初始化
    minipc_recv_data=minipcInit(&huart1); // 视觉通信串口初始化
    //创建一个用于发布给云台控制话题的发布者
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    //创建一个用于订阅云台反馈话题的订阅者
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    //创建一个用于发布给发射机构控制话题的订阅者
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    //创建一个用于发布给发射机构反馈话题的订阅者
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    // 双板CAN通信初始化  
    CANComm_Init_Config_s comm_conf = {
        .can_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 0x200,
            .rx_id =  0x209,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    // 蜂鸣器初始化  
    Buzzer_config_s cmd_buzzer_config={
        .alarm_level=ALARM_LEVEL_HIGH,
        .octave=OCTAVE_1,
    };

    Buzzer_config_s aim_success_buzzer_config= {
        .alarm_level=ALARM_LEVEL_ABOVE_MEDIUM,
        .octave=OCTAVE_2,
    };

    cmd_error_buzzer = BuzzerRegister(&cmd_buzzer_config);
    aim_success_buzzer= BuzzerRegister(&aim_success_buzzer_config);

    shoot_fetch_data.over_heat_flag=0;
    //让云台水平
    gimbal_cmd_send.pitch = 0;
    //将标志位全都置0
    memset(&DataLebel,0,sizeof(DataLebel));
    shoot_cmd_send.friction_mode = FRICTION_OFF;
}
/**
 * @brief 当前云台电机角度计算和零位的误差，计算出云台与底盘的偏角，用于底盘解算
 *        
 *
 */
static void CalcOffsetAngle()
{
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief     云台限位
 */
static void GimbalLocationLimit()
{
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;
}
/**
 * @brief     卡弹检查
 *
 */
static void ShootCheck()
{
    if(shoot_cmd_send.load_mode==LOAD_BURSTFIRE)
    {
        //拨盘速度小于100，且超过1s，认为卡弹，进行回退
        if(abs(shoot_fetch_data.speed)<100&&DWT_GetTimeline_s()>1)
        {
            shoot_fetch_data.loader_error_flag++;
        }
        if(shoot_fetch_data.loader_error_flag==1)
        {
            shoot_cmd_send.load_mode=LOAD_REVERSE;
            shoot_fetch_data.loader_error_flag=0;
        }
    }
    //回退1s，继续射
    else if(shoot_cmd_send.load_mode==LOAD_REVERSE&&DWT_GetTimeline_s()>1)
    {
        shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
    }
}

/**
 * @brief 开启机器人时的共性设置
 *
 */
static void BasicFunctionSet()
{
    //云台基本模式设定
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    GimbalLocationLimit();

    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.shoot_rate=8;
}
/**
 * @brief 判断视觉有没有发信息
 */
static void VisionJudge()
{
    cnt1=sin(DWT_GetTimeline_s());
    if(cnt1>-0.1&&cnt1<1&&DataLebel.cmd_error_flag==0)
    {
        gimbal_cmd_send.last_deep= minipc_recv_data->Vision.deep;
    }

    if(minipc_recv_data->Vision.deep!=0&&DataLebel.cmd_error_flag==0)//代表收到信息
    {
        DataLebel.vision_flag=1;
        //检测到装甲板，开启蜂鸣器
        AlarmSetStatus(aim_success_buzzer, ALARM_ON);
        //与装甲板中心的距离越近，蜂鸣器越响
        if(abs(minipc_recv_data->Vision.yaw)>1&&aim_success_buzzer->loudness<0.5)
        {
            aim_success_buzzer->loudness=0.5*(1/abs(minipc_recv_data->Vision.yaw));
        }
        else if(abs(minipc_recv_data->Vision.yaw)<1 && abs(minipc_recv_data->Vision.pitch)<1)
        {
            //离装甲板距离较近时，开火
            aim_success_buzzer->loudness=0.5;
            DataLebel.fire_flag=1;
        }
        if(minipc_recv_data->Vision.deep-gimbal_cmd_send.last_deep==0&&cnt1<-0.2)
        {
            DataLebel.cmd_error_flag=1;
            DataLebel.fire_flag=0;
            DataLebel.vision_flag=0;
            gimbal_cmd_send.last_deep= minipc_recv_data->Vision.deep;
            AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
        }
    }
     //检测不到装甲板，关蜂鸣器，关火
    else if(minipc_recv_data->Vision.deep==0 && DataLebel.vision_flag==1)       
    {
        DataLebel.fire_flag=0;
        DataLebel.vision_flag=0;
        AlarmSetStatus(aim_success_buzzer, ALARM_OFF);    
    }

}
/********************************RemoteControl****************************/

/**
 * @brief 云台控制，发送yaw和pitch角度
 *
 */
static void GimbalRC()
{
    gimbal_cmd_send.yaw -= 0.0005f * (float)rc_data[TEMP].rc.rocker_right_x;//0
    gimbal_cmd_send.pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_right_y;
    DataLebel.cmd_error_flag=0;
}
/**
 * @brief 底盘控制，发送x和y速度
 *
 */
static void ChassisRC()
{
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy =-10.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_NO_FOLLOW;
    }
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
    }
}
/**
 * @brief 拨盘控制，发送拨盘模式
 *
 */
static void ShootRC()
{
    if(rc_data->rc.dial>200)
    {
        if(shoot_fetch_data.over_heat_flag==1)
        shoot_cmd_send.load_mode=LOAD_STOP;
        else
        {
            //连射
            shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
            ShootCheck();
        }
    }
    else if (rc_data->rc.dial<-200)
    {
        //拨盘反转
        shoot_cmd_send.load_mode=LOAD_REVERSE;
    }
    else if (rc_data->rc.dial<-200)
    {
        shoot_cmd_send.load_mode=LOAD_REVERSE;
    }
    else
    {
        shoot_cmd_send.load_mode=LOAD_STOP;
    }
}
/********************************AutoControl****************************/

/**
 * @brief 云台控制，视觉发的yaw时距离敌人装甲板中心的偏差值
 *
 */
static void GimbalAC()
{
    VisionJudge();
    //没发信息时巡逻
    if(DataLebel.vision_flag==0)
    {
        gimbal_cmd_send.yaw-=0.4;
        DataLebel.t_pitch = (float32_t)DWT_GetTimeline_s();
        //上方不扫，减少扫描范围
        gimbal_cmd_send.pitch =20*abs(sin(2.5*DataLebel.t_pitch));
    }
    else
    {    
        if(abs(minipc_recv_data->Vision.pitch)<400&&abs(minipc_recv_data->Vision.yaw)<400)
        {
            gimbal_cmd_send.yaw-=0.0073f*minipc_recv_data->Vision.yaw;   //往右获得的yaw是减
            gimbal_cmd_send.pitch -= 0.009f*minipc_recv_data->Vision.pitch;
        }
    }
}

/**
 * @brief 底盘控制，巡航的vx，vy的单位是m/s，且是轮速，要将轮子速度转化为电机速度
 *
 */
static void ChassisAC()
{
    chassis_cmd_send.vx=minipc_recv_data->Nav.vx*4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL*1000;
    chassis_cmd_send.vy=minipc_recv_data->Nav.vy*4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL*1000;
    chassis_cmd_send.chassis_mode=CHASSIS_NO_FOLLOW;
}

static void ShootAC()
{
    //检测到装甲板且与其距离较近
    if(DataLebel.fire_flag==1)
    {
        //超热量不开火
        if(shoot_fetch_data.over_heat_flag==1)
        shoot_cmd_send.load_mode=LOAD_STOP;
        else
        {
            shoot_cmd_send.load_mode=LOAD_STOP;
            ShootCheck();
        }
    }
    else
    {
        shoot_cmd_send.load_mode=LOAD_STOP;
    }
}
/**
 * @brief 停止
 *
 */
static void AnythingStop()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
}

/**
 * @brief 
 *
 */
static void RemoteDataDeal()
{
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        BasicFunctionSet();
        GimbalRC();
        ChassisRC();
        ShootRC();
        AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
    }

    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        BasicFunctionSet();
        GimbalAC();
        // //等巡航搞完改为ChassisAC();
         ChassisAC();
         ShootAC();
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) 
    {
        AnythingStop();
    }
}

/**
 * @brief  用于发送来自底盘的裁判信息给发射机构,用于枪管切换
 */
static void SendJudgeShootData()
{
    shoot_cmd_send.bullet_real_speed=chassis_fetch_data.bullet_speed;
    shoot_cmd_send.bullet_num=chassis_fetch_data.bullet_num;
    shoot_cmd_send.left_bullet_heat=chassis_fetch_data.left_bullet_heat;
    shoot_cmd_send.right_bullet_heat=chassis_fetch_data.right_bullet_heat;

}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);

    //获取订阅者信息
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);


    // 根据gimbal的反馈值计算云台和底盘正方向的夹角
    CalcOffsetAngle();

    //在不同挡位下设置对应的不同模式
    RemoteDataDeal();

    // 设置视觉需要用到的数据
    VisionSetFlag();
    VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.Yaw,gimbal_fetch_data.gimbal_imu_data.Pitch,
                        gimbal_fetch_data.gimbal_imu_data.Roll);

    // 设置巡航需要用到的数据       
    NavSetMessage(chassis_fetch_data.real_vx,chassis_fetch_data.real_vy,gimbal_fetch_data.gimbal_imu_data.Yaw,chassis_fetch_data.Occupation
                    ,chassis_fetch_data.remain_HP,chassis_fetch_data.self_infantry_HP,chassis_fetch_data.self_hero_HP
                    ,chassis_fetch_data.enemy_color,chassis_fetch_data.enemy_infantry_HP,chassis_fetch_data.enemy_hero_HP
                    ,chassis_fetch_data.remain_time,shoot_cmd_send.bullet_num,chassis_fetch_data.game_progress
                    );

    //发送给小电脑数据
    SendMinipcData();

    //发送信息给发射机构
    SendJudgeShootData();
    
    //推送发布者信息
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    
    // 双板通信
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
}
