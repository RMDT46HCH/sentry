#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"
#include "arm_math.h"
#define MINIPC_RECV_SIZE 18u // 当前为固定值,36字节
#define MINIPC_SEND_SIZE 36u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;



typedef struct
{
	struct
    {
		uint8_t header;  // 帧头，固定为0x5A
		float32_t yaw;       // 需要云台转动的相对 yaw 角
		float32_t pitch;     // 需要云台转动的相对 pitch 角
		float32_t deep;     // 物体距离
		uint16_t checksum; // 校验和
	}Vision;
	struct
	{
		uint8_t header;  // 帧头，固定为0x5A
		float32_t vx;       // 
		float32_t vy;     // 
		float32_t wz;      // 
		uint16_t checksum; // 校验和
	}Odom;
} __attribute__((packed)) Minipc_Recv_s;

typedef enum
{
	COLOR_BLUE = 1,
	COLOR_RED = 0,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2,
} Vision_Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	struct
	{
		uint8_t header;  // 帧头，固定为0x5A
		uint8_t detect_color;
		float roll;
		float pitch;
		float yaw;
		uint16_t checksum; // 校验和
	}Vision;
	struct
	{
		uint8_t header;  
		float vx;
		float vy;
		float wz;
		float gyro[3];
		uint16_t checksum; // 校验和
	}Odom;
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
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag();

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

/**
 * @brief 设置巡航的信息
 *
 * @param vx
 * @param vy
 */
void OdomSetMessage(float* gyro, float vx, float vy, float wz);


/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_Vision_data(
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len) ;   // 待发送的数据帧长度

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_Odom_data(
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len);    // 待发送的数据帧长度

void get_protocol_info_vision(uint8_t *rx_buf, 
                           Minipc_Recv_s *recv_data);

void get_protocol_info_odom(uint8_t *rx_buf, 
                           Minipc_Recv_s *recv_data);



#endif // !MASTER_PROCESS_H