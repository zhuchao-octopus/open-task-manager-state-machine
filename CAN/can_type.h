#ifndef CAN_TYPE_DEF_H_
#define CAN_TYPE_DEF_H_

#include <stdint.h>
#include <stdbool.h>


#define CAN_TX_STATUS_CONFIRMED (0x01U)
#define CAN_TX_STATUS_REQ       (0x02U)
#define CAN_TX_STATUS_ACTIVE    (0x03U)

#define CAN_MSG_TYPE_EVENT      (0x01U)
#define CAN_MSG_TYPE_PERIOD     (0x02U)
#define CAN_MSG_TYPE_MIXED      (0x03U)
#define CAN_MSG_TYPE_NM         (0x04U)  //Network Manager
#define CAN_MSG_TYPE_SUM        (0x80U)  //是否带滚动计数和校验和
#define CAN_MSG_TYPE_MASK       (0x02U)


//for dubug
#define CANMSG_NORMAL_PRINT 0
#define CANMSG_TIMEROUT_PRINT 0

#if CANMSG_NORMAL_PRINT
#define CANMSG_NORMAL_DBG(x) (x)
#else
#define CANMSG_NORMAL_DBG(x)
#endif



#if CANMSG_TIMEROUT_PRINT
#define CANMSG_TIMEROUT_DBG(x) (x)
#else
#define CANMSG_TIMEROUT_DBG(x)
#endif


#define CAM_MSG_DATA_LEN (8)

typedef enum {
    CAN_MSG_FORMAT_MOTOROLA,
    CAN_MSG_FORMAT_INTEL,
}CanMsgFormat_e;


typedef struct {
    uint32_t id; /* CAN ID */
    uint32_t message_name; /* CAN MESSAGE NAME */
    uint8_t byte_pos; /* 字节位置 */
    uint8_t bit_pos;
    uint8_t sig_len;
    uint32_t default_val;
    uint32_t invalid_val;
} CAN_signal_config;



typedef struct can_msg_tx_config{
    uint32_t instance;
    uint32_t id;

    uint8_t type;
    uint8_t data_len;
    uint16_t period;
    uint8_t data[CAM_MSG_DATA_LEN];
    uint16_t period_cnt;
    uint32_t mailbox;
    uint8_t status;
    uint8_t roll;
    uint8_t tx_enable;
    uint8_t format;
    volatile uint16_t timeout;
    void (*callback)(struct can_msg_tx_config *can_info); /* 回调函数 */
}CAN_msg_tx_config;


typedef struct can_msg_rx_config
{
    uint32_t id;                                            /* CAN ID */
    uint8_t type;                                           /* 事件类型 */
    uint8_t data_len;                                       /* 数据长度 */
    uint16_t period;                                        /* 周期 */
    volatile uint8_t data[CAM_MSG_DATA_LEN];                /* 数据 */
    volatile uint8_t run_data[CAM_MSG_DATA_LEN];            /* 数据 */
    volatile uint8_t is_data_update;                        /* 数据是否更新 */
    volatile uint8_t error_status;                          /* CAN状态 BIT0:1超时错误 */
    volatile uint16_t timeout;                              /* CAN ID 接受超时 单位10ms */
    uint16_t timeout_max;                                   /* 超时时间 */
    uint8_t node;                                           /* 所属节点 */
    uint8_t format;                                         /* 消息格式 */
    void (*callback)(struct can_msg_rx_config *can_info);   /* 回调函数 */
    void (*timeoutCallback)();                              /* 回调函数 */
    void (*normalCallback) (struct can_msg_rx_config *can_info, uint8_t timeout);               /* 回调函数 */
}CAN_msg_rx_config;



#endif //CAN_TYPE_DEF_H_

