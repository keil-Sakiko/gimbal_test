#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define ECD_ANGLE_COEF_DM 0.043945f     // (360/8192),将编码器值转化为角度制
#define REDUCTION_RATIO_DM 10           // 自身减速比

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;         // 速度,逆时针为正（转子正视方向）,单位：rad/s
    float last_position;
    float position;         // 位置,逆时针为正（转子正视方向）,单位：°
    float torque;           // 转矩,注意方向,单位：N·s
    float T_Mos;
    float T_Rotor;
    
    float total_angle;   // 总角度,逆时针为正（转子正视方向）,单位：rad
    int32_t total_round; // 总圈数,注意方向
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;

typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;    // 电机控制器

    float feedforward;  // 前馈
    uint8_t pos_limit_enable;
    float pos_limit_min;    // 位置限位，点击反馈值
    float pos_limit_max;    // 位置限位，点击反馈值

    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
    uint16_t trget_t;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

typedef enum
{
    DM_STATE_OVER_VOLTAGE     = 0x08,    //过压
    DM_STATE_UNDER_VOLTAGE    = 0x09,    //欠压
    DM_STATE_OVER_CURRENT     = 0x0A,    //过流
    DM_STATE_MOS_OVER_TEM     = 0x0B,    //mos过温
    DM_STATE_COIL_OVER_TEM    = 0x0C,    //线圈过温
    DM_STATE_LOST_COM         = 0x0D,    //通信丢失
    DM_STATE_OVER_LOAD        = 0x0E    //过载
}DMMotor_State_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorTask(void);

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorSetFed(DMMotorInstance *motor, float fed);
void DMMotorSetReverse(DMMotorInstance *motor, uint8_t reverse_flag);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);

#endif // !DMMOTOR