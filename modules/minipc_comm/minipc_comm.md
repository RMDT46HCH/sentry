# 小电脑通信
## 小电脑通信的数据协议

static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;

/* 检验数据帧头* ///接收协议被调用
static uint8_t protocol_heade_Check(uint8_t *rx_buf)
{
    if (rx_buf[0] == ODOM_CMD_ID)
    {
        return 2;
    }    
    return 0;
}

//发送协议
void get_protocol_send_MiniPC_data(Minipc_Send_s  tx_data, uint8_t  tx_buf,uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    data_len = 33;
    / 帧头部分 /
    tx_buf[0]=tx_data->header;
    / 数据段 /
    memcpy(&tx_buf[1], &tx_data->Nav.vx, sizeof(float));
    memcpy(&tx_buf[5], &tx_data->Nav.vy, sizeof(float));
    memcpy(&tx_buf[9], &tx_data->Nav.yaw, sizeof(float));

    memcpy(&tx_buf[13], &tx_data->Nav.self_sentry_HP, sizeof(uint16_t));
    memcpy(&tx_buf[15], &tx_data->Nav.self_hero_HP, sizeof(uint16_t));
    memcpy(&tx_buf[17],  &tx_data->Nav.self_infantry_HP, sizeof(uint16_t));

    memcpy(&tx_buf[19], &tx_data->Nav.enemy_sentry_HP, sizeof(uint16_t));
    memcpy(&tx_buf[21], &tx_data->Nav.enemy_hero_HP, sizeof(uint16_t));
    memcpy(&tx_buf[23],  &tx_data->Nav.enemy_infantry_HP, sizeof(uint16_t));

    memcpy(&tx_buf[25], &tx_data->Nav.remain_time, sizeof(uint16_t));
    memcpy(&tx_buf[27], &tx_data->Nav.remain_bullet, sizeof(uint16_t));

    memcpy(&tx_buf[29],  &tx_data->Nav.game_progress, sizeof(uint8_t));
    memcpy(&tx_buf[30],  &tx_data->Nav.occupation, sizeof(uint8_t));
    tx_buf[31] =tx_data->Vision.detect_color;

    tx_buf[32]=tx_data->Nav.tail1;
    *tx_buf_len = data_len ;
}
//接收协议
void get_protocol_info_minipc(uint8_t  rx_buf, Minipc_Recv_s  recv_data)
{
    static protocol_rm_struct pro;
    static uint16_t date_length;
    if (protocol_heade_Check(rx_buf)==2) 
    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        {
        // 将接收到的数据复制到Nav_Recv_s结构体中
        recv_data->header = rx_buf[0];
        memcpy(&recv_data->Nav.vx, &rx_buf[1], sizeof(float));
        memcpy(&recv_data->Nav.vy, &rx_buf[5], sizeof(float));
        memcpy(&recv_data->Nav.wz, &rx_buf[9], sizeof(float));
        memcpy(&recv_data->Vision.yaw,&rx_buf[13],sizeof(float));
        memcpy(&recv_data->Vision.pitch,&rx_buf[17],sizeof(float));
        memcpy(&recv_data->Vision.deep,&rx_buf[21],sizeof(float));
        memcpy(&recv_data->Nav.patrol_mode,&rx_buf[25],sizeof(uint8_t));        
        }
    }
}

sart交互的指针
// 定义一个小电脑 看门狗 实例指针，用来和bsp层的daemon 联系
static DaemonInstance *minipc_daemon_instance;
// 定义一个小电脑串口实例指针，用来和bsp层的usart联系
//调用这个函数需要这个USARTInstance *_instance
//void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)

static USARTInstance *minipc_usart_instance;

离线回调函数
/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
  */
static void VisionOfflineCallback()
{
    USARTServiceInit(minipc_usart_instance);      
}


接收回调函数（接收到信息后调用这个函数）

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 */
static void DecodeMinpc()
{
    DaemonReload(minipc_daemon_instance); // 喂狗
get_protocol_info_vision(minipc_usart_instance->recv_buff,&minipc_recv_data);
}

Minipc_Recv_s  *minipcInit(UART_HandleTypeDef  _handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeMinpc;
    conf.recv_buff_size = Minipc_Recv_sIZE;
    conf.usart_handle = _handle;
    minipc_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = minipc_usart_instance,
        .reload_count = 10,
    };
    minipc_daemon_instance = DaemonRegister(&daemon_conf);

    return &minipc_recv_data;
}



/**
 * @brief 发送函数
 */
void SendMinipcData()
{
    static uint8_t send_buff[Minipc_Send_sIZE];
    static uint16_t tx_len;
    get_protocol_send_Vision_data(&minipc_send_data, 1, send_buff, &tx_len);
    USARTSend(minipc_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); 
}
