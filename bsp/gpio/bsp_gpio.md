# gpio
- 每定义一个`gpio_instance`，就代表一个gpio的**实例**（对象）。
## .c中的函数：
`GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config);`应用层设置这三个exti_mode、GPIO_Pin、GPIOx，模块层设置回调函数

`GPIOSet((GPIOInstance *_instance);`设置为高电平
`GPIOToggel((GPIOInstance *_instance);`电平翻转
`GPIOReset((GPIOInstance *_instance);`设置为低电平
## 使用示例
```c
//在app层只需要设置前三个,callback由module自动设置

GPIO_Init_Config_s gpio_init = {
    .exti_mode = GPIO_EXTI_MODE_FALLING, // 注意和CUBEMX的配置一致
    .GPIO_Pin = GPIO_PIN_6, // GPIO引脚
    .GPIOx = GPIOG, // GPIO外设
},

GPIOInstance* test_example = GPIORegister(&gpio_init);
GPIOSet(test_example);
GPIOToggel(test_example);
GPIOReset(test_example);
```

```c
config->acc_int_config.gpio_model_callback = BMI088AccINTCallback;
config->gyro_int_config.gpio_model_callback = BMI088GyroINTCallback;
bmi088_instance->acc_int = GPIORegister(&config->acc_int_config); // 只有在非阻塞模式下才需要注册中断
bmi088_instance->gyro_int = GPIORegister(&config->gyro_int_config);
```