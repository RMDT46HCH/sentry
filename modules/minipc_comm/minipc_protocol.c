#include "minipc_comm.h"
#include "minipc_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "memory.h"

static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;

/*检验数据帧头*/
static uint8_t protocol_heade_Check(uint8_t *rx_buf)
{
    if (rx_buf[0] == MINIPC_CMD_ID)
    {
        return 2;
    }    
    return 0;
}


void get_protocol_send_MiniPC_data(Minipc_Send_s *tx_data, uint8_t *tx_buf,uint16_t *tx_buf_len)
{
    static uint16_t data_len;
    data_len = 33;
    /*帧头部分*/
    tx_buf[0]=tx_data->header;
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
    tx_buf[31] =tx_data->Vision.detect_color;

    tx_buf[32]=tx_data->Nav.tail1;
    *tx_buf_len = data_len ;
}

void get_protocol_info_minipc(uint8_t *rx_buf, Minipc_Recv_s *recv_data)
{
    static uint16_t date_length;
    if (protocol_heade_Check(rx_buf)==2) 
    {
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