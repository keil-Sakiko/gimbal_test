#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 14u // 当前为固定值
#define VISION_SEND_SIZE 28u

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
	uint8_t header;
	uint8_t tracking: 1;
	uint8_t id: 3;          // 0-outpost 6-guard 7-base
	uint8_t armors_num: 3;  // 2-balance 3-outpost 4-normal
	uint8_t reserved: 1;
	float pitch;
	float yaw;
	int16_t shoot_mode;
	int16_t checksum;
}Vision_Recv_s;

typedef enum
{
	// COLOR_NONE = 0,
	// COLOR_BLUE = 1,
	// COLOR_RED = 2,

	COLOR_RED = 0,
	COLOR_BLUE = 1,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_25 = 25,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	// Enemy_Color_e enemy_color;
	// Work_Mode_e work_mode;.
	// Bullet_Speed_e bullet_speed;

	// float yaw;
	// float pitch;
	// float roll;

	uint8_t header;
	uint8_t detect_color: 1;  // 0-red 1-blue
	uint8_t reset_tracker: 1;
	uint8_t reserved: 6;
	float roll;
	float pitch;
	float yaw;
	float speed;
	float yaw_motor_angle;
	float pitch_motor_angle;
	uint16_t checksum;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);//IMU反馈角度
void VisionSetMotorAngle(float yaw, float pitch);//电机机械角度反馈

#endif // !MASTER_PROCESS_H