#include "master_process.h"
#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "memory.h"

static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;
/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0]; 
        return 1;
    }
    if (rx_buf[0] == ODOM_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        return 2;
    }    
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_Vision_data(
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    data_len =  16;
    /*帧头部分*/
    tx_buf[0] = SEND_VISION_ID;
    /*数据段*/
    tx_buf[1] =tx_data->Vision.detect_color;
    memcpy(&tx_data->Vision.roll, &tx_buf[2], sizeof(float));
    memcpy(&tx_data->Vision.pitch, &tx_buf[6], sizeof(float));
    memcpy(&tx_data->Vision.yaw, &tx_buf[10], sizeof(float));
    /*整包校验*/
    crc16 = crc_16(&tx_buf[0], data_len - 2); // 不包括最后两个字节的校验和
    tx_buf[data_len - 2] = crc16 & 0xff; // 低位在前
    tx_buf[data_len - 1] = (crc16 >> 8) & 0xff; // 高位在后
    *tx_buf_len = data_len ;
    //tx_buf[1] = data_len & 0xff;        // 低位在前
    //tx_buf[2] = (data_len >> 8) & 0xff; // 低位在前
    //tx_buf[3] = crc_8(&tx_buf[0], 3);   // 获取CRC8校验位

    /*数据的信号id*/
    //tx_buf[4] = send_id & 0xff;
    //tx_buf[5] = (send_id >> 8) & 0xff;

    /*建立16位寄存器*/
    //tx_buf[6] = flags_register & 0xff;
    //tx_buf[7] = (flags_register >> 8) & 0xff;

    /*float数据段*/
    //for (int i = 0; i < 4 * float_length; i++)
    //{
    //    tx_buf[i + 8] = ((uint8_t *)(&tx_data[i / 4]))[i % 4];
    //}

    /*整包校验*/
    //crc16 = crc_16(&tx_buf[0], data_len + 6);
    //tx_buf[data_len + 6] = crc16 & 0xff;
    //tx_buf[data_len + 7] = (crc16 >> 8) & 0xff;
}
void get_protocol_send_Nav_data(
                            Minipc_Send_s *tx_data,          
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    data_len = 32;
    /*帧头部分*/
    tx_buf[0]=tx_data->Nav.header;
    /*数据段*/
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

    tx_buf[31]=tx_data->Nav.tail1;
    *tx_buf_len = data_len ;
}
/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
void get_protocol_info_vision(uint8_t *rx_buf, 
                        Minipc_Recv_s *recv_data)
{
    static protocol_rm_struct pro;
    static uint16_t date_length;

    if (protocol_heade_Check(&pro, rx_buf)==1) 
    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        //if (CRC16_Check_Sum(rx_buf, date_length)) {
            // 将接收到的数据复制到Vision_Recv_s结构体中
            recv_data->Nav.header = rx_buf[0];
            memcpy(&recv_data->Vision.yaw, &rx_buf[1], sizeof(float));
            memcpy(&recv_data->Vision.pitch, &rx_buf[5], sizeof(float));
            memcpy(&recv_data->Vision.deep, &rx_buf[9], sizeof(float));
            recv_data->Vision.checksum = (rx_buf[date_length - 2] << 8) | rx_buf[date_length - 1];
    }
}
void get_protocol_info_odom(uint8_t *rx_buf, 
                           Minipc_Recv_s *recv_data)
{
    static protocol_rm_struct pro;
    static uint16_t date_length;

    if (protocol_heade_Check(&pro, rx_buf)==2) 
    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        //if (CRC16_Check_Sum(rx_buf, date_length)) 
        {
            // 将接收到的数据复制到Nav_Recv_s结构体中
            recv_data->Nav.header = rx_buf[0];
            memcpy(&recv_data->Nav.vx, &rx_buf[1], sizeof(float));
            memcpy(&recv_data->Nav.vy, &rx_buf[5], sizeof(float));
            memcpy(&recv_data->Nav.wz, &rx_buf[9], sizeof(float));
            recv_data->Nav.checksum = (rx_buf[10] << 8) | rx_buf[11];
        }
    }
}
