# gimbal
## 声明
- `GimbalUsed`
`static attitude_t *gimbal_IMU_data;` 云台IMU数据
`static DJIMotorInstance *yaw_motor, *pitch_motor;`云台yaw、pitch电机数据

-  `CommUsed` 用于和`cmd`进行数据交互
`static Publisher_t *gimbal_pub;`                   // 云台应用消息发布者(云台将信息反馈给cmd)
`static Subscriber_t *gimbal_sub; `                  // cmd控制消息订阅者
`static Gimbal_Upload_Data_s gimbal_feedback_data;` // 回传给cmd的云台状态信息
`static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;`         // 来自cmd的控制信息

## 初始化 `void GimbalInit();`

- `gimbal_IMU_data = INS_Init(); `IMU先初始化,获取姿态数据指针
-    ` Motor_Init_Config_s yaw_config = {};Motor_Init_Config_s pitch_config = {};`设置电机初始化结构体(包括设置can通信如句柄、id，设置pid，及电机安装是正转还是反转（相当于给最终输出值添负号）、电机型号并将前面获取到姿态数据指针的赋给yaw、pitch电机的其他数据来源)
-   ` yaw_motor = DJIMotorInit(&yaw_config);`将初始化结构体传给yaw电机实例，并初始化yaw电机实例
-   ` pitch_motor = DJIMotorInit(&pitch_config);`将初始化结构体传给pitch电机实例，并初始化pitch电机实例
- 设置发布者用于发布云台反馈出来的信息
    `gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));`
- 设置订阅者，订阅云台的控制信息
    `gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));   ` 


## 功能
- `void GimbalStateSet()`云台状态设定，无力模式让电机停止，陀螺仪模式通过cmd给的控制信息控制云台
- `void SendGimbalData()`发送反馈信息给终端
`gimbal_imu_data`用于设定一个初始化的yaw轴，在云台半周巡逻模式下云台围绕一个单圈角度值进行巡逻
`yaw_motor_single_round_angle`单圈角度，用于麦轮解算时差角计算
`total_round`用于保证在全周巡逻模式下旋转多圈后，云台不重新转回多圈，就能围绕一个单圈角度值进行巡逻
## 云台任务
- `void GimbalTask()`发布云台反馈信息，订阅云台控制信息并控制云台。
将这个任务放在 `RobotTask()`中,并在实时操作系统中做`RobotTask()`。
## TIPS
没有连接的电机就直接注释掉！！！不然会卡在硬件中断！！！
