#ifndef MINIPC_COMM_H
#define MINIPC_COMM_H

#include "bsp_usart.h"
#include "minipc_protocol.h"
#include "arm_math.h"
#define MINIPC_RECV_SIZE 36u // 当前为固定值,36字节
#define MINIPC_SEND_SIZE 36u

#pragma pack(1)

typedef struct
{
	uint8_t header;  // 帧头，固定为0x5A
	struct
    {
		float32_t yaw;       // 需要云台转动的相对 yaw 角
		float32_t pitch;     // 需要云台转动的相对 pitch 角
		float32_t deep;     // 物体距离
	}Vision;
	struct
	{
		float32_t vx;       // 
		float32_t vy;     // 
		float32_t wz;      // 
		uint8_t patrol_mode;
	}Nav;
} __attribute__((packed)) Minipc_Recv_s;

typedef enum
{
	COLOR_BLUE = 1,
	COLOR_RED = 0,
} Enemy_Color_e;


typedef struct
{
	uint8_t header;  // 帧头，固定为0x5A
	struct
	{
		uint8_t detect_color;
	}Vision;
	struct
	{
		float32_t vx;
		float32_t vy;
		float32_t yaw;
		uint16_t self_sentry_HP; 
		uint16_t self_hero_HP; 
		uint16_t self_infantry_HP; 
		uint16_t enemy_sentry_HP;
		uint16_t enemy_hero_HP; 
		uint16_t enemy_infantry_HP; 
		uint16_t remain_time;
		uint16_t remain_bullet;
		uint8_t occupation;
		uint8_t game_progress;
		uint8_t tail1; 
	}Nav;
} __attribute__((packed)) Minipc_Send_s;







#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送小电脑数据
 *
 */
void SendMinipcData();


/**
 * @brief 设置巡航的信息
 *
 * @param vx
 * @param vy
 */
void NavSetMessage(float vx, float vy, float yaw,uint8_t occupation,
					uint16_t self_sentry_HP,uint16_t self_infantry_HP,uint16_t self_hero_HP,
					uint16_t enermy_sentry_HP,uint16_t enermy_infantry_HP,uint16_t enermy_hero_HP,
                    uint16_t remain_time,uint16_t remain_bullet,uint8_t game_progress,uint8_t detect_color
					);

void get_protocol_send_MiniPC_data(Minipc_Send_s *tx_data,uint8_t *tx_buf,uint16_t *tx_buf_len);  
void get_protocol_info_minipc(uint8_t *rx_buf, Minipc_Recv_s *recv_data);



#endif // !MINIPC_COMM_H