# shoot
## 声明
- ` ShootUsed`
`static DJIMotorInstance *friction_l, *friction_r, *loader; `// 摩擦轮电机、拨盘电机
`static ServoInstance *change;`                              // 换枪管舵机

- `CommUsed` 
`static Publisher_t *shoot_pub;`                  //发布发射反馈信息的发布者
`static Shoot_Ctrl_Cmd_s shoot_cmd_recv;`         // cmd的发射控制信息
`static Subscriber_t *shoot_sub;`                 //订阅发射控制信息的订阅者
`static Shoot_Upload_Data_s shoot_feedback_data;` // 发射机构的反馈信息

- `HeatUsed`
`static cal_heat_t cal_heat;`                      // 热量计算结构体
## 初始化
`void ShootInit()`
`    Motor_Init_Config_s friction_config = {};`设置摩擦轮电机初始化结构体(包括设置can通信如句柄、pid，及电机安装是正转还是反转（相当于给最终输出值添负号）)将初始化结构体传给摩擦轮电机实例，并初始化摩擦轮电机实例
单独设置id与电机正反装
```c
    friction_config.can_init_config.tx_id = 1,
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
    friction_l = DJIMotorInit(&friction_config);
    friction_config.can_init_config.tx_id = 2; // 右摩擦轮
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);
```
`    Motor_Init_Config_s loader_config = {}`设置拨盘电机初始化结构体(包括设置can通信如句柄、pid，及电机安装是正转还是反转（相当于给最终输出值添负号）)
单独设置id与电机正反装
```c
    loader_config.can_init_config.tx_id=3;
    loader = DJIMotorInit(&loader_config);
```
设置舵机初始化结构体(包括设置定时器句柄、通道，舵机类型)
```c
    Servo_Init_Config_s config= {
        .htim=&htim1,
        .Channel=TIM_CHANNEL_1,
        //舵机的初始化模式和类型
        .Servo_Angle_Type=Free_Angle_mode,
        .Servo_type=Servo270,
    };
    // 设置好参数后进行初始化并保留返回的指针
    change = ServoInit(&config);
```
- 设置发布者用于发布发射机构反馈出来的信息    `shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));`
- 设置订阅者，订阅发射机构的控制信息 `shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));`
- 初始化使能左枪管
```c
    memset(&cal_heat,0,sizeof(cal_heat));
    cal_heat.shoot_l=1;
    cal_heat.shoot_r=0;
```
## 功能
- `static void CalHeat()` 计算热量。`shoot_l`/`shoot_r`分别代表左枪管和右枪管状态，哪个等于1，就让舵机去哪边，一个枪管热量满了就切换枪管到另一边，两个都满了就发超热量标志位。等到恢复好了再发射。
- `ShootStateSet()` 发射机构状态设置,用于开关发射机构
- ` ShootLoaderSet()` 速度环控制拨盘。后续想要打能量机关，再写一个拨盘角度环控制。
示例如下：
```c
static float hibernate_time = 0, dead_time = 0;
static float loader_set_angle = 0;

DJIMotorOuterLoop(loader, ANGLE_LOOP);// 切换到角度环
loader_set_angle = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE*REDUCTION_RATIO_LOADER; // 控制量增加一发弹丸的角度
DJIMotorSetRef(loader, loader_set_angle); 
hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
dead_time = shoot_cmd_recv.dead_time;
if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    return;                                                                          // 触发一次就直接返回 
```

- `ShootSpeedSet()`速度环控制摩擦轮，**因为射速与场地温度有关，所以要根据场地调整摩擦轮速度**
- `SendShootData()` ,发送拨盘速度来判断是否卡弹。

## 任务 `void ShootTask()`
- 从cmd获取控制数据
- 电机是否上电
- 发射模式设定
- 射频设定
- 射速设定
- 计算热量并进行枪管切换
- 给发布中心电机实际情况，从而调节拨盘电机的模式
- 反馈数据,用于卡弹反馈
