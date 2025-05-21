# Board2Board  
## c文件里的函数
`static void SuperCapRxCallback(CANInstance *_instance);`在bsp_can调用的回调函数

`SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)` 注册用来与功率控制板通信的开发板实例
`void SuperCapSend(SuperCapInstance *instance, uint8_t *data)`   发送数据
`SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)`   读取数据

## 示例代码
```c
static uint16_t power_data;
static SuperCapInstance *cap;                                       // 超级电容
static SuperCap_Msg_s cap_msg;
SuperCap_Init_Config_s cap_conf = {
    .can_config = {
        .can_handle = &hcan2,
        .tx_id = 0x302, // 超级电容默认接收id
        .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    }
};
cap = SuperCapInit(&cap_conf); // 超级电容初始化
SuperCapSend(cap, (uint8_t *)&power_data);
cap_msg = SuperCapGet(cap);
```