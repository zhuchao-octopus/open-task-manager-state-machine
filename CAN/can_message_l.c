
/*****************************************************************************/
#include "can_message_l.h"

#include <stdio.h>
#include <string.h>
#include "can_queue.h"
#include "can_id_find.h"
#include "../octopus_tickcounter.h" 
#include "../octopus_can_hal.h"

//#include "../OTSM/octopus_can.h"
//#include "timer.h"
//#include "dio.h"
//#include "uds_port.h"

#define CAN_TIMEOUT_MASK        (0x02U)
#define CAN_TX_PERIOD_MASK      (2U)   /* 2MS */
#define CAN_HAL_CONFIG_NUM      (2U)

/* CAN_msg_tx_config */
CAN_msg_tx_config *CAN_msg_tx_config_info;
uint8_t CAN_msg_tx_config_count = 0;

/* CAN_msg_rx_config */
CAN_msg_rx_config *CAN_msg_rx_config_info;
uint8_t CAN_msg_rx_config_count = 0;

CanQueue_t CAN_rx_msg_queue;

static CAN_msg_rx_config* FindCanMsgRxConfigByMessageName(uint32_t message_name);
//static void runDataUpdateByCANID(uint32_t can_id);
static void runDataUpdate(uint32_t idx);
static void can_ml_receive_app_data(CanQueueMsg_t* msg);

static uint32_t getCanDataMotorola(volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len);
static uint8_t writeCanDataMotorola(uint32_t sig_val,volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len);

static uint32_t getCanDataIntel(volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len);
static uint8_t writeCanDataIntel(uint32_t sig_val,volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len);

static uint32_t l_t_2ms_period_tmr = 0;

/*公can工作使能*/
/*enable 1:使能，0：失能*/
void can_ml_set_can_standby(uint8_t enable) {
    if(enable) {
        //Dio_WriteChannel(DIO_O_CAN_STB, DIO_LOW);
    } else {
        //Dio_WriteChannel(DIO_O_CAN_STB, DIO_HIGH);
    }
}

void can_ml_init(void)
{
    can_ml_set_can_standby(1);
    StartTickCounter(&l_t_2ms_period_tmr);
    can_ml_set_user_tx_enable(0);
}

void can_ml_deinit(void)
{
    //FLEXCAN_HAL_EnterFreezeMode(CAN1);
}

uint8_t can_ml_readCanDataByMessageName(uint32_t message_name,uint32_t* sig_val,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len) {
    //uint8_t ret = 0;
    uint32_t run_sig_val;
    volatile CAN_msg_rx_config *info = FindCanMsgRxConfigByMessageName(message_name);
    if(info)
    {
        if(info->format == CAN_MSG_FORMAT_INTEL) {
            *sig_val = getCanDataIntel(info->data, 8, byte_pos, bit_pos, sig_len);
            run_sig_val = getCanDataIntel(info->data, 8, byte_pos, bit_pos, sig_len);
        } else {
            *sig_val = getCanDataMotorola(info->data, 8, byte_pos, bit_pos, sig_len);
            run_sig_val = getCanDataMotorola(info->data, 8, byte_pos, bit_pos, sig_len);
        }
    
        if(*sig_val != run_sig_val||info->is_data_update)
        {
            return 1;
        }
    }
    return 0;
}

uint8_t can_ml_writeCanDataByCANID(uint32_t can_id,uint32_t sig_val,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len)
{
    uint8_t ret = 0;
    //uint8_t i;

    int idx = findCanIdMessageNameTx(can_id);
    if(idx < 0 || idx >= CAN_msg_tx_config_count) {
        return 0;
    }

    CAN_msg_tx_config* info = &CAN_msg_tx_config_info[idx];
    
    if(info->format == CAN_MSG_FORMAT_INTEL) {
        ret = writeCanDataIntel(sig_val, info->data, info->data_len, byte_pos, bit_pos, sig_len);
    } else {
        ret = writeCanDataMotorola(sig_val, info->data, info->data_len, byte_pos, bit_pos, sig_len);
    }
    
    if (((info->type & CAN_MSG_TYPE_EVENT) == CAN_MSG_TYPE_EVENT))
    {
        info->status = CAN_TX_STATUS_REQ;
    }
    return ret;
}

uint8_t can_ml_writeCanData(uint32_t can_id, uint8_t *data, uint8_t len)
{
    uint8_t ret = 0;
    //uint8_t i;

    int idx = findCanIdMessageNameTx(can_id);
    if(idx < 0 || idx >= CAN_msg_tx_config_count) {
        return 0;
    }

    CAN_msg_tx_config* info = &CAN_msg_tx_config_info[idx];
    
    len = len < CAM_MSG_DATA_LEN ? len : CAM_MSG_DATA_LEN;
    for(int i = 0; i < len; i++)
    {
        info->data[i] = data[i];
    }
    
    if (((info->type & CAN_MSG_TYPE_EVENT) == CAN_MSG_TYPE_EVENT))
    {
        info->status = CAN_TX_STATUS_REQ;
    }
    return ret;
}

uint8_t can_ml_resetCanDataByCANID(uint32_t can_id,uint32_t sig_val,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len)
{
    uint8_t ret = 0;
    //uint8_t i;

    int idx = findCanIdMessageNameRx(can_id);
    if(idx < 0 || idx >= CAN_msg_rx_config_count) {
        return 0;
    }


    CAN_msg_rx_config* info = &CAN_msg_rx_config_info[idx];
    if(info->format == CAN_MSG_FORMAT_INTEL) {
        ret = writeCanDataIntel(sig_val, info->data, info->data_len, byte_pos, bit_pos, sig_len);
    } else {
        ret = writeCanDataMotorola(sig_val, info->data, info->data_len, byte_pos, bit_pos, sig_len);
    }
        
    return ret;
}

void can_ml_Register_CAN_id_rx_handle(CAN_msg_rx_config* temp,uint8_t size)
{
    CAN_msg_rx_config_info = temp;
    CAN_msg_rx_config_count = size;
}

void can_ml_Register_CAN_id_tx_handle(CAN_msg_tx_config* temp,uint8_t size)
{
    CAN_msg_tx_config_info = temp;
    CAN_msg_tx_config_count = size;
}

/* CAN_TX_PERIOD_MASK ms */
bool can_ml_transmit(void)
{
    uint8_t idx = 0;
    //static uint8_t reserved = 0;
    uint32_t time = 0;
    if (GetTickCounter(&l_t_2ms_period_tmr) >= 2)
    {

        time = GetTickCounter(&l_t_2ms_period_tmr)>>1;
        StartTickCounter(&l_t_2ms_period_tmr);
    }

    for (idx = 0; idx < CAN_msg_tx_config_count; idx++)
    {
        if (CAN_msg_tx_config_info[idx].tx_enable)
        {
            //处理普通应用报文
            /*prd*/
            /* 周期管理 */

            if ((CAN_msg_tx_config_info[idx].type & CAN_MSG_TYPE_MASK) ==CAN_MSG_TYPE_PERIOD&&CAN_msg_tx_config_info[idx].status != CAN_TX_STATUS_REQ)
            {
                if(CAN_msg_tx_config_info[idx].period_cnt < time){
                    CAN_msg_tx_config_info[idx].period_cnt = 0;
                }else{
                    CAN_msg_tx_config_info[idx].period_cnt -= time;
                    
                }

                if (CAN_msg_tx_config_info[idx].period_cnt == 0)
                {
                    CAN_msg_tx_config_info[idx].period_cnt = CAN_msg_tx_config_info[idx].period / CAN_TX_PERIOD_MASK;

                    CAN_msg_tx_config_info[idx].status = CAN_TX_STATUS_REQ;
                }

            }else{
                CAN_msg_tx_config_info[idx].period_cnt = CAN_msg_tx_config_info[idx].period / CAN_TX_PERIOD_MASK;
            }
        
            /* 发送管理 */
            if (CAN_msg_tx_config_info[idx].status == CAN_TX_STATUS_REQ) {
                if (Can_SendMsg(CAN_msg_tx_config_info[idx].mailbox,
                                        CAN_msg_tx_config_info[idx].id,
                                        CAN_msg_tx_config_info[idx].data,
                                        CAN_msg_tx_config_info[idx].data_len)) {
                    CAN_msg_tx_config_info[idx].status = CAN_TX_STATUS_ACTIVE;
                    //CAN_msg_tx_config_info[idx].roll++;
                } else {

                }
            }
        }
    }
    return true;
}


static void can_ml_receive_app_data(CanQueueMsg_t* msg){
    //DEV_ASSERT(msg);
    uint8_t i;

    int idx = findCanIdMessageNameRx(msg->id);
    if(idx < 0 || idx >= CAN_msg_rx_config_count) {
        /*没有查询到can id*/
        return;
    }

    //PRINTF("can rev:");
        for (i = 0; i < msg->data_len; i++) {
            CAN_msg_rx_config_info[idx].data[i] = msg->data[i];
            //PRINTF("%02x ",msg->data[i]);
        }
        //PRINTF("\n");

        //////uds_recv_frame(CAN_msg_rx_config_info[idx].id, (uint8_t*)CAN_msg_rx_config_info[idx].data, CAN_msg_rx_config_info[idx].data_len);

        if (CAN_msg_rx_config_info[idx].data_len == msg->data_len )
        {
            if (CAN_msg_rx_config_info[idx].type == CAN_MSG_TYPE_PERIOD || CAN_msg_rx_config_info[idx].type == CAN_MSG_TYPE_NM)
            {
                CAN_msg_rx_config_info[idx].timeout = CAN_msg_rx_config_info[idx].timeout_max;
                CAN_msg_rx_config_info[idx].error_status &= ~0x01;
            }

            if (CAN_msg_rx_config_info[idx].normalCallback != NULL) {
                CAN_msg_rx_config_info[idx].normalCallback(&(CAN_msg_rx_config_info[idx]), 0);
            }
            CAN_msg_rx_config_info[idx].is_data_update = 0;
            runDataUpdate(idx);
            //刷新完成
        } else {
            /* 长度不匹配 */
        }
}

bool can_ml_receive(void)
{
    if(Can_GetMsgQueueSize())
    {
        CanQueueMsg_t* msg = Can_GetMsg();
    
        if (NULL != msg)
        {
            can_ml_receive_app_data(msg);
            return true;
        }
    }
    return false;
}

void can_ml_Reset_rx_timeout_status()
{
    uint8_t idx;
    for (idx = 0; idx < CAN_msg_rx_config_count; idx++)
    {
        if (CAN_msg_rx_config_info[idx].type == CAN_MSG_TYPE_PERIOD)
        {
            CAN_msg_rx_config_info[idx].error_status &= ~0x01;
            CAN_msg_rx_config_info[idx].is_data_update = 1;
        }
    }
}

/* 10ms period */
void can_ml_rx_timeout_manager(void) {
    uint8_t idx = 0;
    for (idx = 0; idx < CAN_msg_rx_config_count; idx++) {
        if (CAN_msg_rx_config_info[idx].timeout > 0) {
            --CAN_msg_rx_config_info[idx].timeout;
        }

        if (0 == CAN_msg_rx_config_info[idx].timeout) {
            if ((CAN_msg_rx_config_info[idx].error_status & 0x01) == 0x00) {

                if (CAN_msg_rx_config_info[idx].timeoutCallback != NULL) {
                    CAN_msg_rx_config_info[idx].timeoutCallback();

                    CAN_msg_rx_config_info[idx].is_data_update = 1;//超时强制刷新标志
                    
                    if (CAN_msg_rx_config_info[idx].normalCallback != NULL) {
                        CAN_msg_rx_config_info[idx].normalCallback(&(CAN_msg_rx_config_info[idx]), 1);
                    }

                    runDataUpdate(idx);
                }
                
                CAN_msg_rx_config_info[idx].error_status |= 0x01;
            }
        }
    }
}

void CAN_hal_tx_timeout_manager(void)
{

}

void can_ml_set_user_tx_enable(uint32_t channel)
{
    uint32_t instance = channel;
    uint8_t idx = 0;
    for (idx = 0; idx < CAN_msg_tx_config_count; idx++)
    {
        if(CAN_msg_tx_config_info[idx].instance == instance) {
            CAN_msg_tx_config_info[idx].tx_enable = 1;
        }
    }
}

void can_ml_set_user_tx_disable(uint32_t channel)
{
    uint32_t instance = channel;
    uint8_t idx = 0;
    for (idx = 0; idx < CAN_msg_tx_config_count; idx++)
    {
        if(CAN_msg_tx_config_info[idx].instance == instance) {
            CAN_msg_tx_config_info[idx].tx_enable = 0;
        }
    }
}

static CAN_msg_rx_config* FindCanMsgRxConfigByMessageName(uint32_t message_name)
{
    if(message_name < CAN_msg_rx_config_count) {
        return &CAN_msg_rx_config_info[message_name];
    }
    return NULL;
}

#if 0
static void runDataUpdateByCANID(uint32_t can_id)
{
    int idx = findCanIdMessageNameRx(can_id);
    if(idx < 0 || idx >= CAN_msg_rx_config_count) {
        for(int i = 0; i < CAM_MSG_DATA_LEN; i++)
        {
            CAN_msg_rx_config_info[idx].run_data[i] = CAN_msg_rx_config_info[idx].data[i];
        }
    }
}
#endif

static void runDataUpdate(uint32_t idx)
{
    for(int i = 0; i < CAM_MSG_DATA_LEN; i++)
    {
        CAN_msg_rx_config_info[idx].run_data[i] = CAN_msg_rx_config_info[idx].data[i];
    }
}

static uint32_t getCanDataMotorola(volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len)
{
    //DEV_ASSERT(data_len);

    uint32_t sig_val;
    uint8_t tmp_len;
    uint32_t tmp_mask;
    uint8_t i;
#if 1//def FORMAT_MOTOROLA

    if((sig_len > 8) && ((byte_pos +1)*8 < sig_len))
    {
        //PRINTF("%s  ++++ warn,invalid byte_pos!!! +++\n", __FUNCTION__);
        return 0;
    }
    
    sig_val = 0x0;
    tmp_len = 8 - (bit_pos & 0x7);
    tmp_len = (sig_len > tmp_len) ? tmp_len : sig_len;
    tmp_mask = (1 << tmp_len) - 1;
    sig_val |= (uint32_t) (data[byte_pos] >> (bit_pos & 0x7)) & tmp_mask;

    sig_len -= tmp_len;
    --byte_pos;
    i = 0;

    while (sig_len >= 8) {
        sig_val |= (uint32_t) data[byte_pos] << (tmp_len + 8 * i);
        sig_len -= 8;
        --byte_pos;
        ++i;
    }

    if (sig_len > 0) {
        tmp_mask = (1 << sig_len) - 1;
        sig_val |= (uint32_t) (data[byte_pos] & tmp_mask) << (tmp_len + 8 * i);
    }

#endif
    return sig_val;
}

static uint8_t writeCanDataMotorola(uint32_t sig_val,volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len)
{
    //DEV_ASSERT(data_len);

    uint8_t ret = 0;
    uint8_t tmp_len;
    uint32_t tmp_mask;
    uint8_t tmp_val;
    uint8_t tmp_bit_pos;
    uint8_t i;

#if 1//def FORMAT_MOTOROLA
    
    if((sig_len > 8) && ((byte_pos +1)*8 < sig_len))
    {
        //PRINTF("%s  ++++ warn,invalid byte_pos!!! +++\n", __FUNCTION__);
        return 0;
    }
       
    tmp_bit_pos = bit_pos & 0x7;
    tmp_len = 8 - tmp_bit_pos;
    tmp_len = (sig_len > tmp_len) ? tmp_len : sig_len;
    tmp_mask = (1 << tmp_len) - 1;

    tmp_val = (data[byte_pos] >> tmp_bit_pos) & tmp_mask;
    tmp_val <<= tmp_bit_pos;

    data[byte_pos] -= tmp_val;
    data[byte_pos] |= (uint8_t) ((sig_val & tmp_mask) << tmp_bit_pos);

    sig_len -= tmp_len;
    --byte_pos;
    i = 0;

    while (sig_len >= 8) {
        data[byte_pos] = (uint8_t) (sig_val >> (tmp_len + 8 * i));

        sig_len -= 8;
        --byte_pos;
        ++i;
    }

    if (sig_len > 0) {
        tmp_mask = (1 << sig_len) - 1;

        tmp_val = data[byte_pos] & tmp_mask;

        data[byte_pos] -= tmp_val;
        data[byte_pos] |= (uint8_t) (sig_val >> (tmp_len + 8 * i)) & tmp_mask;
    }
#endif
    return ret;
}


static uint32_t getCanDataIntel(volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len)
{
    //DEV_ASSERT(data_len);
    uint32_t sig_val;
    uint8_t tmp_len;
    uint32_t tmp_mask;
    uint8_t i;
#if 1//def FORMAT_INTEL

    if((sig_len > 8) && ((data_len - byte_pos +1)*8 < sig_len))
    {
        //PRINTF("%s  ++++ warn,invalid byte_pos!!! +++\n", __FUNCTION__);
        return 0;
    }
      
    sig_val = 0x0;
    tmp_len = 8 - (bit_pos & 0x7);
    tmp_len = (sig_len > tmp_len) ? tmp_len : sig_len;
    tmp_mask = (1 << tmp_len) - 1;
    sig_val |= (uint32_t) (data[byte_pos] >> (bit_pos & 0x7)) & tmp_mask;

    sig_len -= tmp_len;
    ++byte_pos;
    i = 0;

    while (sig_len >= 8) {
        sig_val |= (uint32_t) data[byte_pos] << (tmp_len + 8 * i);
        sig_len -= 8;
        ++byte_pos;
        ++i;
    }

    if (sig_len > 0) {
        tmp_mask = (1 << sig_len) - 1;
        sig_val |= (uint32_t) (data[byte_pos] & tmp_mask) << (tmp_len + 8 * i);
    }

#endif
    return sig_val;
}


static uint8_t writeCanDataIntel(uint32_t sig_val,volatile uint8_t data[],uint8_t data_len,uint8_t byte_pos,uint8_t bit_pos,uint8_t sig_len)
{
    //DEV_ASSERT(data_len);

    uint8_t ret = 0;
    uint8_t tmp_len;
    uint32_t tmp_mask;
    uint8_t tmp_val;
    uint8_t tmp_bit_pos;
    uint8_t i;

#if 1//def FORMAT_INTEL
    
    if((sig_len > 8) && ((data_len - byte_pos +1)*8 < sig_len))
    {
        //PRINTF("%s  ++++ warn,invalid byte_pos!!! +++\n", __FUNCTION__);
        return 0;
    }
    
    tmp_bit_pos = bit_pos & 0x7;
    tmp_len = 8 - tmp_bit_pos;
    tmp_len = (sig_len > tmp_len) ? tmp_len : sig_len;
    tmp_mask = (1 << tmp_len) - 1;

    tmp_val = (data[byte_pos] >> tmp_bit_pos) & tmp_mask;
    tmp_val <<= tmp_bit_pos;

    data[byte_pos] -= tmp_val;
    data[byte_pos] |= (uint8_t) ((sig_val & tmp_mask) << tmp_bit_pos);

    sig_len -= tmp_len;
    ++byte_pos;
    i = 0;

    while (sig_len >= 8) {
        data[byte_pos] = (uint8_t) (sig_val >> (tmp_len + 8 * i));

        sig_len -= 8;
        ++byte_pos;
        ++i;
    }

    if (sig_len > 0) {
        tmp_mask = (1 << sig_len) - 1;

        tmp_val = data[byte_pos] & tmp_mask;

        data[byte_pos] -= tmp_val;
        data[byte_pos] |= (uint8_t) (sig_val >> (tmp_len + 8 * i)) & tmp_mask;
    }
#endif
    return ret;
}


CanQueueMsg_t *Can_GetMsg(void)
{
    static CanQueueMsg_t s_msg;
    if(CanQueue_Pop(&CAN_rx_msg_queue, &s_msg))
    {
        return &s_msg;
    }
    return NULL;
} 

uint16_t Can_GetMsgQueueSize(void)
{
    return CanQueue_Length(&CAN_rx_msg_queue);
}
