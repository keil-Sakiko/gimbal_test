// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "mc6c.h"
#include "user_lib.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static MC_ctrl_t *mc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static PIDInstance Vision_yaw_PID={
    .Kp = 0.5,
    .Ki = 0.002,
    .Kd = 0.0,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 50,
    .MaxOut = 150,
};

static PIDInstance Vision_pitch_PID={
    .Kp = 0.0,
    .Ki = 0.0,
    .Kd = 0.0,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 30,
    .MaxOut = 150,
};

static Robot_Status_e robot_state; // 机器人整体工作状态

static uint16_t vision_yaw,vision_pitch;
static float vision_yaw_ref, vision_pitch_ref;

static uint8_t vision_tracing;

void RobotCMDInit()
{
    mc_data = MCControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    // static float angle;
    // angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度

    // if (angle > YAW_ALIGN_ANGLE)
    //     gimbal_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    // else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    //     gimbal_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    // else
    //     gimbal_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
}

//yaw方向需要根据机械角度进行限幅，这里只是为了代码简洁做封装
static void RemoteYawConstrain()
{
    //由于不知道yaw初始位置，以下根据yaw电机当前角度做了软件限位。TODO:在CalcOffsetAngle中获取yaw电机偏移角度更直接
    //yaw正向前是gimbal_fetch_data.yaw_motor_single_round_angle在0和360跳变的位置，所以处理较复杂
    //限位在315°到360°之间和0°到45°之间
    if(gimbal_fetch_data.yaw_motor_single_round_angle<45.0f || gimbal_fetch_data.yaw_motor_single_round_angle>315.0f)
    {
        gimbal_cmd_send.yaw += -0.003f * (float)mc_data[TEMP].rocker_l_;
    }
    if(gimbal_fetch_data.yaw_motor_single_round_angle>=280.0f && gimbal_fetch_data.yaw_motor_single_round_angle<=315.0f)
    {
        if(mc_data[TEMP].rocker_l_<0)
            gimbal_cmd_send.yaw += 0.0;//此时还向右打杆则没有反应
        else gimbal_cmd_send.yaw += -0.003f * (float)mc_data[TEMP].rocker_l_;
    }
        if(gimbal_fetch_data.yaw_motor_single_round_angle>=45.0f && gimbal_fetch_data.yaw_motor_single_round_angle<=80.0f)
    {
        if(mc_data[TEMP].rocker_l_>0)
            gimbal_cmd_send.yaw += 0.0;//此时还向左打杆则没有反应
        else
            gimbal_cmd_send.yaw += -0.003f * (float)mc_data[TEMP].rocker_l_;
    }
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(mc_data[TEMP].switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(mc_data[TEMP].switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }

    // 云台参数,确定云台控制数据
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    // if (switch_is_down(mc_data[TEMP].switch_left) || vision_recv_data->target_state == NO_TARGET)
    if (switch_is_down(mc_data[TEMP].switch_left) && (switch_is_down(mc_data[TEMP].switch_right) || switch_is_mid(mc_data[TEMP].switch_right)) || vision_recv_data->tracking == 0)
    {
        mc_data[TEMP].rocker_l_=float_deadband((float)mc_data[TEMP].rocker_l_, -30, 30);//遥控器拨杆死区处理
        mc_data[TEMP].rocker_l1=float_deadband((float)mc_data[TEMP].rocker_l1, -30, 30);//遥控器拨杆死区处理
        gimbal_cmd_send.pitch += -0.0015f * (float)mc_data[TEMP].rocker_l1;
        RemoteYawConstrain();
    }
    // 云台软件限位
    gimbal_cmd_send.pitch = float_constrain(gimbal_cmd_send.pitch,-10,20.0);//pitch限位
    // gimbal_cmd_send.yaw = float_constrain(gimbal_cmd_send.yaw,-52.0,52.0);
    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (mc_data[TEMP].rocker_r1 > 200) // 向上超过100,打开摩擦轮
    {    
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    else
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (mc_data[TEMP].rocker_r1 > 500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 15;
}

//yaw方向需要根据机械角度进行限幅，这里只是为了代码简洁做封装
static void VisionYawConstrain()
{
    if(gimbal_fetch_data.yaw_motor_single_round_angle<45.0f || gimbal_fetch_data.yaw_motor_single_round_angle>315.0f)
    {
        gimbal_cmd_send.yaw = Vision_yaw_PID.Output;
    }
    if(gimbal_fetch_data.yaw_motor_single_round_angle>=280.0f && gimbal_fetch_data.yaw_motor_single_round_angle<=315.0f)
    {
        if(Vision_yaw_PID.Output > 0)
            gimbal_cmd_send.yaw += 0.0;//此时视觉控制输出大于零则没有反应
        else gimbal_cmd_send.yaw = Vision_yaw_PID.Output;
    }
    if(gimbal_fetch_data.yaw_motor_single_round_angle>=45.0f && gimbal_fetch_data.yaw_motor_single_round_angle<=80.0f)
    {
        if(Vision_yaw_PID.Output < 0)
            gimbal_cmd_send.yaw += 0.0;//此时视觉控制输出小于零则没有反应
        else
            gimbal_cmd_send.yaw = Vision_yaw_PID.Output;
    }
}

static void VisionControlSet()
{
    if (switch_is_mid(mc_data[TEMP].switch_left) && vision_recv_data->tracking == 1) // 左侧开关状态为[中],视觉模式
    {
        vision_yaw_ref = (((float)vision_recv_data->yaw)/1000.0f)*RAD_2_DEGREE;
        vision_pitch_ref = -(((float)vision_recv_data->pitch)/1000.0f)*RAD_2_DEGREE;

        gimbal_cmd_send.yaw = PIDCalculate(&Vision_yaw_PID, gimbal_fetch_data.gimbal_imu_data.Yaw, vision_yaw_ref);
        // PIDCalculate(&Vision_pitch_PID, gimbal_fetch_data.gimbal_imu_data.Roll, vision_pitch_ref);
        gimbal_cmd_send.pitch = 0;
        
        // 云台软件限位
        // VisionYawConstrain();
        gimbal_cmd_send.pitch = float_constrain(gimbal_cmd_send.pitch,-10,20.0);
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
/*
static void MouseKeySet()
{
    chassis_cmd_send.vx = mc_data[TEMP].key[KEY_PRESS].w * 300 - mc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = mc_data[TEMP].key[KEY_PRESS].s * 300 - mc_data[TEMP].key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)mc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)mc_data[TEMP].mouse.y / 660 * 10;

    switch (mc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (mc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (mc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (mc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (mc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (mc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}
*/
/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         开关控制.
 *
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (switch_is_up(mc_data[TEMP].switch_right) || robot_state == ROBOT_STOP)
    {
        robot_state = ROBOT_STOP;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;

        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;// 云台急停模式
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[中/下],恢复正常工作状态
    if (switch_is_down(mc_data[TEMP].switch_right) || switch_is_mid(mc_data[TEMP].switch_right))
    {
        robot_state = ROBOT_READY;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据s
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(mc_data[TEMP].switch_left) || switch_is_mid(mc_data[TEMP].switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
    {
        RemoteControlSet();
        VisionControlSet();
    }
    else if (switch_is_up(mc_data[TEMP].switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        // MouseKeySet();
        ;
        

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    vision_tracing = vision_recv_data->tracking;
    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    VisionSend(&vision_send_data);
}

