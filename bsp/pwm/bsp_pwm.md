# pwm
- 每定义一个`pwm_instance`，就代表一个pwm的**实例**（对象）。

## .c中的函数：

### 对HAL（tim.h）的回调函数的重定义(你可能发现以下函数并不在bsp_pwm.h里。其实是对HAL（tim.h）的回调函数的重定义)
- 接收：
`void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)`
调用模块层里设置的回调函数
### 外部接口
- 注册
`PWMInstance *PWMRegister(PWM_Init_Config_s *config);`
将你设置的参数结构体传到实例的结构体、启动PWM、设置占空比、设置周期
- 使用实例
```c
static PWMInstance *buzzer;
PWM_Init_Config_s buzzer_config = {
    .htim = &htim4,
    .channel = TIM_CHANNEL_3,
    .dutyratio = 0,
    .period = 0.001,
};
buzzer = PWMRegister(&buzzer_config);
```
- 应用
`void PWMStart(PWMInstance *pwm);`启动PWM
`void PWMStop(PWMInstance *pwm);`停止PWM
`void PWMSetPeriod(PWMInstance *pwm, float period);`设置周期（音调）
`void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);`设置占空比 0-1

- 使用实例
```c
static PWMInstance *buzzer;
PWMSetDutyRatio(buzzer, 0.2);
PWMSetPeriod(buzzer, (float)1 / DoFreq);
```

