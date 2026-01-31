/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    // send_data.enemy_color = enemy_color;
    // send_data.work_mode = work_mode;
    // send_data.bullet_speed = bullet_speed;

    send_data.header = 0xA5;
    send_data.detect_color = (enemy_color == COLOR_BLUE) ? 1 : 0;
    send_data.reset_tracker = 0;
    send_data.reserved = 0;
    send_data.speed = (float)bullet_speed; 
    send_data.checksum = 0;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = -yaw;
    send_data.pitch = roll;
    send_data.roll = pitch;//C板安装坐标roll和pitch调换
}

void VisionSetMotorAngle(float yaw, float pitch)
{
    send_data.yaw_motor_angle = yaw;
    send_data.pitch_motor_angle = pitch;
}
/// @brief 计算发送的checksum数值，不包含header,把每个字节相加，取低16位
static void VisionCalCheckSum()
{
    uint8_t buf[VISION_SEND_SIZE];
    memcpy(buf, &send_data, VISION_SEND_SIZE);

    // checksum 不包含 header 字段
    buf[0] = 0;

    // 计算时不包含 checksum 字段本身
    buf[VISION_SEND_SIZE - 2] = 0;
    buf[VISION_SEND_SIZE - 1] = 0;

    uint16_t sumraw = 0;
    for (uint32_t i = 0; i < VISION_SEND_SIZE; i++)
    {
        sumraw = (uint16_t)(sumraw + buf[i]);
    }
    send_data.checksum = ((sumraw & 0x00FF) << 8) | ((sumraw & 0xFF00) >> 8);
}

/// @brief 处理接收的CheckSum数据
/// @return checksum是否正确，1正确，0错误
static uint8_t VisionCheckSum(const uint8_t *frame, uint32_t len)
{
    if (frame == NULL || len < 4)
        return 0;

    uint8_t buf[VISION_RECV_SIZE];
    if (len > VISION_RECV_SIZE)
        len = VISION_RECV_SIZE;

    memcpy(buf, frame, len);

    // checksum 不包含 header 字段
    buf[0] = 0;

    // 计算时不包含 checksum 字段本身
    buf[len - 2] = 0;
    buf[len - 1] = 0;

    uint16_t sumraw = 0;
    for (uint32_t i = 0; i < len; i++)
    {
        sumraw = (uint16_t)(sumraw + buf[i]);
    }

    // 与发送端 VisionCalCheckSum() 保持一致：对低16位做高低字节互换
    uint16_t sum = (uint16_t)(((sumraw & 0x00FF) << 8) | ((sumraw & 0xFF00) >> 8));

    uint16_t rx_sum = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    return (sum == rx_sum) ? 1 : 0;
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    if(!DaemonIsOnline(vision_daemon_instance))
    {
        LOGWARNING("[vision] vision offline, restart communication.");
    }
}


/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    // uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    // get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
    if (vision_usart_instance->recv_buff[0] == 0xA5 &&
        VisionCheckSum(vision_usart_instance->recv_buff, VISION_RECV_SIZE))
        memcpy(&recv_data, &vision_usart_instance->recv_buff, VISION_RECV_SIZE);
    else
        return;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    VisionSetFlag(COLOR_RED, VISION_MODE_AIM, SMALL_AMU_15); // 默认值

    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 100,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    // static uint16_t flag_register;
    // static uint8_t send_buff[VISION_SEND_SIZE];
    // static uint16_t tx_len;
    // static uint8_t arr[11]="hello RDK\n";
    // TODO: code to set flag_register
    // flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    // get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    // USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_IT); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // USARTSend(vision_usart_instance, arr, 11, USART_TRANSFER_IT); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.

    static uint8_t send_buf[VISION_SEND_SIZE];
    
    VisionCalCheckSum();
    memcpy(&send_buf[0], &send_data, VISION_SEND_SIZE);
    USARTSend(vision_usart_instance, send_buf, VISION_SEND_SIZE, USART_TRANSFER_IT);
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    // uint16_t flag_register;
    // get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗

    if (vis_recv_buff[0] == 0xA5 &&
        VisionCheckSum(vis_recv_buff, VISION_RECV_SIZE))
        memcpy(&recv_data, vis_recv_buff, VISION_RECV_SIZE);
    else
        return;

}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);
    
    VisionSetFlag(COLOR_RED, VISION_MODE_AIM, SMALL_AMU_15); // 默认值

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    // static uint16_t flag_register;
    // static uint8_t send_buff[VISION_SEND_SIZE];
    // static uint16_t tx_len;
    // // TODO: code to set flag_register
    // flag_register = 30 << 8 | 0b00000001;
    // // 将数据转化为seasky协议的数据包
    // get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    // USBTransmit(send_buff, tx_len);
    static uint8_t send_buf[VISION_SEND_SIZE];
    
    VisionCalCheckSum();
    memcpy(&send_buf[0], &send_data, VISION_SEND_SIZE);
    USBTransmit(send_buf, VISION_SEND_SIZE);
}

#endif // VISION_USE_VCP
