/******************************************************************************
* 文件名称: SID22_ReadDataByIdentifier.c
* 内容摘要: 根据标识符读取数据
* 创建者の: 孔佳伟
* 个人主页: https://gitee.com/thin-wind/jump
* 修改记录: 
******************************************************************************/

#include "SID22_ReadDataByIdentifier.h"
#include "service_cfg.h"
#include "uds_service.h"
#include "SID27_SecurityAccess.h"
#include "SID10_SessionControl.h"
//#include "verstring.h"
//#include "mcu.h"
//#include "data_carinfo.h"
//#include "sys_pwr_status.h"
//#include "task_app_system.h"
#include "assistlib.h"

/******************************************************************************
* 函数名称: bool_t service_22_check_len(const uint8_t* msg_buf, uint16_t msg_dlc)
* 功能说明: 检查 22 服务数据长度是否合法
* 输入参数: uint16_t msg_dlc         --数据长度
* 输出参数: 无
* 函数返回: TRUE: 合法; FALSE: 非法
* 其它说明: 无
******************************************************************************/
bool_t service_22_check_len(const uint8_t* msg_buf, uint16_t msg_dlc)
{
    bool_t ret = FALSE;
    
    (void)msg_buf;
    if(3 == msg_dlc)
        ret = TRUE;

    return ret;
}


/******************************************************************************
* 函数名称: void service_22_ReadDataByIdentifier(const uint8_t* msg_buf, uint16_t msg_dlc)
* 功能说明: 22 服务 - 根据标识符读取数据
* 输入参数: uint8_t*    msg_buf         --数据首地址
    　　　　uint8_t     msg_dlc         --数据长度
* 输出参数: 无
* 函数返回: 无
* 其它说明: 无
******************************************************************************/
void service_22_ReadDataByIdentifier(const uint8_t* msg_buf, uint16_t msg_dlc)
{
    uint8_t rsp_buf[128];
    uint16_t did;

    rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_22);
    rsp_buf[1] = msg_buf[1];
    rsp_buf[2] = msg_buf[2];

    did = ((uint16_t)msg_buf[1]) << 8;
    did |= msg_buf[2];

    switch (did)
    {
        case 0xF185: //当前安全访问等级
        {
            rsp_buf[3] = get_current_sa_lv();
            uds_positive_rsp(rsp_buf, 4);
            break;
        }
        case 0xF186: //当前诊断任务模式
        {
            rsp_buf[3] = get_current_session();
            uds_positive_rsp(rsp_buf, 4);
            break;
        }
        case 0xF180: //Bootloader软件版本号
        {
            uint8_t ver[16] = {0};
            uint32_t* ptr = (uint32_t*)((void*)ver);
            ptr[0] = Mcu_GetRetentionRam(RAM_BOOT_OFFSET + 0);
            ptr[1] = Mcu_GetRetentionRam(RAM_BOOT_OFFSET + 1);
            ptr[2] = Mcu_GetRetentionRam(RAM_BOOT_OFFSET + 2);
            ptr[3] = Mcu_GetRetentionRam(RAM_BOOT_OFFSET + 3);
            for(int i = 0; i < 16; i++)
            {
                rsp_buf[3 + i] = ver[i];
            }
            uds_positive_rsp(rsp_buf, 3 + 16);
            break;
        }
        case 0xF188: //应用软件版本号（MCU）
        {
            uint8_t ver[16] = {APP_VER_STR};
            for(int i = 0; i < 16; i++)
            {
                rsp_buf[3 + i] = ver[i];
            }
            uds_positive_rsp(rsp_buf, 3 + 16);
            break;
        }
        case 0xF1B0: //应用软件版本号（SOC）
        {
            for(int i = 0; i < 16; i++)
            {
                rsp_buf[3 + i] = System_Get_Mpu_Ver()[i+1];
            }
            uds_positive_rsp(rsp_buf, 3 + 16);
            break;
        }
        case 0xF191: //硬件版本号
        {
            for(int i = 0; i < 16; i++)
            {
                rsp_buf[3 + i] = System_Get_Hw_Ver()[i+1];
            }
            uds_positive_rsp(rsp_buf, 3 + 16);
            break;
        }
        case 0x0834: //里程显示值
        {
            rsp_buf[3] = MSB(MSBWORD(data_carinfo_get_carinfo()->Total_ODO));
            rsp_buf[4] = LSB(MSBWORD(data_carinfo_get_carinfo()->Total_ODO));
            rsp_buf[5] = MSB(LSBWORD(data_carinfo_get_carinfo()->Total_ODO));
            rsp_buf[6] = LSB(LSBWORD(data_carinfo_get_carinfo()->Total_ODO));
            uds_positive_rsp(rsp_buf, 7);
            break;
        }
        case 0x0830: //KL15状态
        {
            rsp_buf[3] = sys_pwr_mark_get_acc();
            uds_positive_rsp(rsp_buf, 4);
            break;
        }
        case 0x0831: //蓄电池电压值
        {
            rsp_buf[3] = MSB(data_carinfo_get_carinfo()->battery_voltage);
            rsp_buf[4] = LSB(data_carinfo_get_carinfo()->battery_voltage);
            uds_positive_rsp(rsp_buf, 5);
            break;
        }
        case 0x0838: //档位
        {
            uint8_t gear = 0x05;
            switch(data_carinfo_get_carinfo()->ActualGear)
            {
            case 0x08: gear = 0x03; break;
            case 0x09: gear = 0x04; break;
            case 0x0C: gear = 0x02; break;
            case 0x0A: gear = 0x01; break;
            case 0x0B: gear = 0x00; break;
            default:   gear = 0x03; break;
            }
            rsp_buf[3] = gear;
            uds_positive_rsp(rsp_buf, 4);
            break;
        }
        case 0x0833: //外部温度
        {
            rsp_buf[3] = MSB(data_carinfo_get_carinfo()->outside_temperature);
            rsp_buf[4] = LSB(data_carinfo_get_carinfo()->outside_temperature);
            uds_positive_rsp(rsp_buf, 5);
            break;
        }
        default:
            uds_negative_rsp(SID_22, NRC_REQUEST_OUT_OF_RANGE);
            break;
    }
}


/****************EOF****************/
