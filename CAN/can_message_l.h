#ifndef CAN_ML_H_
#define CAN_ML_H_

#include <stdint.h>
#include "can_type.h"
#include "can_queue.h"

/*can工作使能*/
/*enable 1:使能，0：失能*/
extern void can_ml_set_can_standby(uint8_t enable);

//CAN设备初始化
extern void can_ml_init(void);
//CAN设备卸载
extern void can_ml_deinit(void);


//注册接收CANID指针
extern void can_ml_Register_CAN_id_rx_handle(CAN_msg_rx_config* temp,uint8_t size);
//注册发送CANID指针
extern void can_ml_Register_CAN_id_tx_handle(CAN_msg_tx_config* temp,uint8_t size);


//发送CAN数据
extern bool can_ml_transmit(void);

//接收CAN数据
extern bool can_ml_receive(void);


//重置信号超时状态
extern void can_ml_Reset_rx_timeout_status(void);
//超时管理函数
extern void can_ml_rx_timeout_manager(void);

//读取CAN数据，并检测是否更新，返回值为1即为有更新
extern uint8_t can_ml_readCanDataByMessageName(uint32_t message_name,uint32_t* sig_val,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len);
//写入CAN数据
extern uint8_t can_ml_writeCanDataByCANID(uint32_t can_id,uint32_t sig_val,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len);
extern uint8_t can_ml_writeCanData(uint32_t can_id, uint8_t *data, uint8_t len);

//重置接收到的CAN数据
extern uint8_t can_ml_resetCanDataByCANID(uint32_t can_id,uint32_t sig_val,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len);


extern void can_ml_set_user_tx_enable(uint32_t channel);
extern void can_ml_set_user_tx_disable(uint32_t channel);


///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
extern CanQueue_t CAN_rx_msg_queue;

extern void CanMessageCase_Init(void);

extern CanQueueMsg_t *Can_GetMsg(void);

extern uint16_t Can_GetMsgQueueSize(void);

#endif
