#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 2           // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1
// 如果只有1个CAN,还需要把bsp_can.c中所有的hcan2变量改为hcan1(别担心,主要是总线和FIFO的负载均衡,不影响功能)

/* can instance typedef, every module registered to CAN should have this variable */
#pragma pack(1)
typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;
#pragma pack()

/* CAN1错误历史：保存在RAM，正常通信不会清空；复位后重新计数。 */
typedef struct
{
    uint32_t enabled;                         // 1：CAN1错误通知已成功开启
    uint32_t irq_count;                       // CAN1接收/错误中断入口累计次数，用于确认记录入口在工作
    uint32_t error_event_count;               // 捕捉到的错误中断事件数，不等同于线上错误帧总数
    uint32_t bus_off_count;                   // 捕捉到的Bus-Off进入次数；同一关闭状态不重复计数
    uint32_t last_esr;                        // 最近错误事件的ESR原始值，恢复后保留
    uint32_t last_tsr;                        // 同一错误事件的TSR原始值
    uint32_t last_error_ms;                   // 最近错误发生时的系统毫秒计数
    uint32_t last_lec;                        // 最近捕捉到的有效错误类型1～6，不被0或7覆盖
    uint32_t lec_count[8];                    // 下标1～6分别累计各类错误事件；0和7不计数
    uint32_t max_tec_low8;                    // 观察到的发送错误计数低8位最大值，不是完整9位计数
    uint32_t max_rec;                         // 观察到的接收错误计数最大值
} CAN1_Debug_s;

extern volatile CAN1_Debug_s can1_debug;
void CAN1RecordError(void);                   // 仅由CAN1各中断入口在HAL处理之前调用

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        注册(初始化)一个can实例,需要传入初始化配置的指针.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 * 
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 * 
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param _instance* can instance owned by module
 */
uint8_t CANTransmit(CANInstance *_instance,float timeout);

#endif
