
#ifndef CAN_PTL_H_
#define CAN_PTL_H_

#include <stdint.h>
#include <stdbool.h>
#include "can_type.h"

#include "can_message_rx.h"
#include "can_message_tx.h"
#include "can_signal_rx/can_signal_rx.h"
#include "can_signal_tx/can_signal_tx.h"

//for dubug
#define CANTASK_PRINT 0

#if CANTASK_PRINT
#include <stdio.h>
#define CANTASK_DBG(x) (x)
#else
#define CANTASK_DBG(x)
#endif


//读取CAN数据，用于can_signal_rx_[xxx]调用
uint8_t can_fuction_read_signal_value_callback(CAN_signal_config config, uint32_t *sig_val);
//复位can数据，用于can_signal_rx_[xxx]调用
uint8_t can_fuction_timeout_reset_candata_callback(CAN_signal_config config);
uint8_t can_fuction_read_signal_timeout_flag(CanReceiveID canReceiveID);
void can_fuction_read_signal_value(CanReceiveID sigId,uint8_t sig,uint32_t *sig_val);

//写入发送的CAN数据
void can_fuction_write_signal_value(uint8_t sig_id, uint32_t sig_val);

//can初始化
void can_fuction_init(void);

void can_fuction_load_data(void);
//can卸载
void can_fuction_deinit(void);

//接收CAN数据
void can_fuction_loop_rt(void);

//发送CAN数据
void can_fuction_loop_2ms(void);

//超时管理函数
void can_fuction_loop_10ms(void);

void can_fuction_PowerOn(void);

void can_fuction_PowerOff(void);

bool can_fuction_isCanWKTimeout(void);

void can_fuction_tx_enable(bool enable);

void can_message_case_init(void);

void can_fuction_app_register(void);

#endif //CAN_PTL_H_

