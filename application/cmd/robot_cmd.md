# gimbal
## 声明
- `HuartUsed`
`static RC_ctrl_t *rc_data;  `                    遥控器数据,初始化时返回
`static Minipc_Recv_s *minipc_recv_data;  `       接收到的小电脑数据,用于底盘到达一个目标点、用于云台到达能够击打到装甲板的位置。初始化时返回
`static Minipc_Send_s  minipc_send_data;`         发送给小电脑的数据，用于视觉判断找哪种颜色的装甲板、导航对赛场情况做出决策

- `CANUsed`
`static BoardCommInstance *cmd_can_comm;`           // 双板通信 主控板实例

- `ChassisUsed`
`static Chassis_Ctrl_Cmd_s chassis_cmd_send;`      // 发送给底盘应用的信息,包括控制信息和UI绘制相关的信息
`static Chassis_Upload_Data_s chassis_fetch_data;` // 从底盘应用接收的反馈信息信息,裁判系统数据（如血量、底盘功率等）

- `GimbalUsed`
`static Publisher_t *gimbal_cmd_pub;`              // 云台控制消息发布者
`static Subscriber_t *gimbal_feed_sub;`            // 云台反馈信息订阅者
`static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;`        // 传递给云台的控制信息
`static Gimbal_Upload_Data_s gimbal_fetch_data;`   // 从云台获取的反馈信息

- `ShootUsed` 
`static Publisher_t *shoot_cmd_pub;`                 // 发射控制消息发布者
`static Subscriber_t *shoot_feed_sub;`               // 发射反馈信息订阅者
`static Shoot_Ctrl_Cmd_s shoot_cmd_send; `           // 传递给发射的控制信息
`static Shoot_Upload_Data_s shoot_fetch_data; `      // 从发射获取的反馈信息

-`Other`  
`static DataLebel_t DataLebel;`                        // 标志位结构体   
`static BuzzzerInstance *aim_success_buzzer;      `    // 识别目标蜂鸣器实例
`static cal_round_patrol_t round_patrol;         `     // 巡逻全周结构体
`static cal_mid_round_patrol_t mid_round_patrol;`      // 巡逻半周结构体
`static cal_temporary_round_patrol_t tem_round_patrol;`// 丢失目标临时巡逻结构体

## 初始化 `void RobotCMDInit();`
### 串口初始化 `HuartInit`
```c
    rc_data = RemoteControlInit(&huart3);       // 遥控器通信串口初始化
    minipc_recv_data=minipcInit(&huart1);       // 视觉通信串口初始化
```
### 云台 发布-订阅初始化 `GimbalCommInit`
```c
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
```
#### 云台pitch初始化
`gimbal_cmd_send.pitch = 0;`

### 发射机构 发布-订阅初始化 `ShootCommInit`
```c
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
```
### 底盘 双板通信初始化 `ChassisCommInit`
```c
    BoardComm_Init_Config_s comm_conf = {
        .can_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 0x200,
            .rx_id =  0x209,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = BoardCommInit(&comm_conf);
```
### 蜂鸣器初始化 `BufferInit`
```c
    Buzzer_config_s aim_success_buzzer_config= {
        .alarm_level=ALARM_LEVEL_ABOVE_MEDIUM,
        .octave=OCTAVE_2,
    };

    aim_success_buzzer= BuzzerRegister(&aim_success_buzzer_config);
```
### 标志位初始化 `OtherInit`
```c
    shoot_fetch_data.over_heat_flag=0;
    //让云台水平
    mid_round_patrol.direction=1;
    tem_round_patrol.direction=1;
    //将标志位全都置0
    memset(&DataLebel,0,sizeof(DataLebel));
```

## 功能
- `void  CalcOffsetAngle()`计算云台与底盘夹角
- `void GimbalLocationLimit()`云台软件限位，即云台达到机械限位的地方，控制值 就不再继续加/减角度了。
- `void ShootCheck()` 见dwt.md
- `void VisionJudge()` 收到视觉信息后，执行控制云台，到达能打的位置后，蜂鸣器响，执行射击。
`DataLebel.flag=1`时，证明进过视觉控制，收到过视觉信息。转回没收到视觉信息即检测不到装甲板，先让`DataLebel.flag=2`，停留一段时间，真的没收到视觉信息，再进入巡逻模式
失去目标后记录当时的yaw，围绕当时的yaw轴进行临时的小巡逻，巡逻一定次数还是没找到目标再进入全圈巡逻或半圈巡逻
```c
        if(tem_round_patrol.flag==0)
        {
            tem_round_patrol.yaw_init=gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            tem_round_patrol.yaw=tem_round_patrol.yaw_init;
            tem_round_patrol.flag=1;
        }
        if(tem_round_patrol.num >12)
        {
            DataLebel.vision_flag=0;
            DataLebel.flag=0; 
            tem_round_patrol.num=0;
            tem_round_patrol.flag=0;
        }
```
- `void BasicFunctionSet()`启动模式下都要做的事情，如判断有无装甲板并执行相应的控制、云台pitch限位、差角计算、开摩擦轮、调射频
- ` GimbalRC()`云台纯遥控器控制
- `ChassisRC() `底盘纯遥控器控制，并用左开关来决定底盘是旋转、不旋转（跟随底盘/不跟随）
**tip：跟随的参数调了，又删了**
- `ShootRC()`拨盘纯遥控器控制
- `GetGimbalInitImu()`记录一开始的初始化yaw角，让他围绕这个yaw进行巡逻。
- `MidRoundPatrol()`半圈巡逻，让他围绕这个yaw进行巡逻。同时要考虑全周巡逻模式下旋转多圈后，云台不重新转回多圈，就能围绕一个单圈角度值进行巡逻。
- `RoundPatrol()`全周巡逻，要考虑到初始化的圈数，再考虑转多圈后的圈数（**不能直接拿编码器的圈数哦，因为云台在旋转的同时，底盘也在旋转，所以直接拿编码器的圈数，要用yaw总角度除360去获取yaw真正转动的圈数**），用这个圈数减去初始化圈数，就是全周巡逻转动的圈数了。
- `TemporaryPatrol()`临时巡逻，临时在丢失角度的那时候巡逻就行。
- `FoundEnermy()`找到目标后，缺多少角度补多少角度
- `GimbalAC()` 获取一开始的yaw，以后他在巡逻时，都会围绕这个yaw巡逻。从头到尾，没收到视觉信息就全周或者半周巡逻。收到视觉信息后丢失数据，就先小巡逻，再进入全周或者半周巡逻。
- `ChassisAC()`,vx、vy就是由真实速度转换成转子速度。就是逆解转子速度转为真实速度。（详看chassis.md）,底盘旋转速度也交给小电脑，他是一个百分比。同时将底盘模式改为小电脑操控的模式（CHASSIS_NAV）
- `ShootAC()` 因为卡弹，所以趁他还没有到达位置，只是识别的时候先退弹，等到位置再射击
- `AnythingStop()`停止 
- @Todo ：按到这个挡位时，让他可以再重新设定想要围绕巡逻的yaw轴。
- `SendJudgeShootData()` 发送从底盘的裁判系统获取的数据给发射机构，让其能换枪管。

## 终端任务`void RobotCMDTask()`
- 订阅云台、底盘、发射机构的反馈信息。
- 在不同挡位下设置对应的不同模式
- 将要用到的数据赋值给要发送给小电脑的数据。
- 发送给小电脑数据
- 发送信息给发射机构
- 推送发布者信息
- 双板通信发送信息
将这个任务放在 `RobotTask()`中,并在实时操作系统中做`RobotTask()`。
## TIPS
如果没有连接板子就直接注释掉双板通信！！！不然会卡在硬件中断！！！
