/**
 * @file flysky.h
 * @brief [SBUS模式(反相)，波特率100_000，数据位9位，停止位2位，偶检验]
 * @author Univs_he
 * @date 2025-12-15
 */

#ifndef MC6C_H
#define MC6C_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

#define MC_SBUS

#define MC_SBUS_USER_CHANNELS		6
/* User configuration */


/* ----------------------- RC Switch Definition----------------------------- */
#define MC_DATA_UP ((uint16_t)200)   // 开关向上时的值
#define MC_DATA_MID ((uint16_t)1000)  // 开关中间时的值
#define MC_DATA_DOWN ((uint16_t)1800) // 开关向下时的值

#define MC_DATA_MAX ((uint16_t)1800) // 最小值
#define MC_DATA_MIN ((uint16_t)200) // 最大值

/* ----------------------- RC Switch Definition----------------------------- */
#define MC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define MC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define MC_SW_DOWN ((uint16_t)2) // 开关向下时的值

// 三个判断开关状态的宏
#define switch_is_down(s) (s == MC_SW_DOWN)
#define switch_is_mid(s) (s == MC_SW_MID)
#define switch_is_up(s) (s == MC_SW_UP)

#define mc_data_dead_limit(data, dealine)        			\
    {                                                    	\
        if((data) > (dealine) || (data) < -(dealine)) 		\
        {                                                	\
            (data) = (data);                          		\
        }                                                	\
        else                                             	\
        {                                                	\
            (data) = 0;                                		\
        }                                                	\
    }
	

#define mc_data_change(data)                                \
	{                                                    	\
		switch(data)										\
		{													\
			case MC_DATA_DOWN:							    \
				data = MC_SW_DOWN;							\
				break;										\
			case MC_DATA_MID:								\
				data = MC_SW_MID;							\
				break;										\
			case MC_DATA_UP:								\
				data = MC_SW_UP;							\
				break;										\
			default:										\
				break;										\
		}													\
	}

/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
    int16_t rocker_r_;      // 左水平，左前侧拨杆保持上拨，[-800~800]
    int16_t rocker_r1;      // 左竖直，左前侧拨杆保持上拨，[-800~800]
    int16_t rocker_l1;      // 右水平，左前侧拨杆保持上拨，[-800~800]
    int16_t rocker_l_;      // 右竖直，左前侧拨杆保持上拨，[-800~800]

    uint16_t switch_left;  // 最左侧拨杆
    uint16_t switch_right;  // 最右侧开关

    uint16_t none[2];  // 保留
     
    
} MC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
MC_ctrl_t *MCControlInit(UART_HandleTypeDef *mc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t MCControlIsOnline();

#endif
