#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "daemon.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖
static DaemonInstance* shoot_daemon;
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// static uint8_t lockflag;
// static float speed;
// static uint16_t count;

// dwt定时,计算冷却用
// static float hibernate_time = 0, dead_time = 0;

// void LoaderLockCallback(void *motor)
// {
//     lockflag = 1;
// }


void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 3; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 6,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 150, // 10
                .Ki = 10,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 5000, // 10
                .Ki = 100, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 10000,
            },
            .current_PID = {
                .Kp = 800.0, // 0.7
                .Ki = 10.0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 10000, //10000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, 
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    
    // // 注册拨盘电机堵转守护线程
    // Daemon_Init_Config_s daemon_config = {
    //     .callback = LoaderLockCallback,
    //     .owner_id = shoot_daemon,
    //     .reload_count = 50, //堵转0.5s丢失
    // };
    // loader->daemon = DaemonRegister(&daemon_config);
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // count = shoot_daemon->temp_count;

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;
    // if(lockflag==1)
    // shoot_cmd_recv.load_mode=LOAD_REVERSE;
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        DaemonReload(shoot_daemon); // 重载守护线程计数
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        // speed = DJIMotorGetSpeed(loader);
        // if(fabs(DJIMotorGetSpeed(loader)) > 3000.0) // 速度过低<10°/s说明卡弹,触发堵转保护
        // {
        //     DaemonReload(shoot_daemon); // 重载守护线程计数
        // }
        // break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)

    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        // lockflag = 0;
        // DJIMotorOuterLoop(loader, SPEED_LOOP);
        // DJIMotorSetRef(loader, -shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 7);
        // DWT_Delay(10);
        // DaemonReload(shoot_daemon);
        // ...
        break;
    default:
        break;
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 24000);
            DJIMotorSetRef(friction_r, 24000);
            break;
        case SMALL_AMU_25:
            DJIMotorSetRef(friction_l, 35000);
            DJIMotorSetRef(friction_r, 35000);
        case SMALL_AMU_27:
            DJIMotorSetRef(friction_l, 38000);
            DJIMotorSetRef(friction_r, 38000);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 43000);
            DJIMotorSetRef(friction_r, 43000);
            break;
        default: // 默认弹速为15m/s,根据裁判系统反馈的值得到的数据，是比较精准的
            DJIMotorSetRef(friction_l, 24000);
            DJIMotorSetRef(friction_r, 24000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        // DJIMotorSetRef(friction_l, 0);
        // DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}