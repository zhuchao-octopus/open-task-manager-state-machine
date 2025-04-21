/******************************************************************************
* 文件名称: SID2E_WriteDataByIdentifier.c
* 内容摘要: 根据标识符写入数据
* 创建者の: 孔佳伟
* 个人主页: https://gitee.com/thin-wind/jump
* 修改记录: 
******************************************************************************/


#include "SID2E_WriteDataByIdentifier.h"
#include "service_cfg.h"
#include "uds_service.h"
#include "uds_data.h"
#include "assistlib.h"
//#include "sys_app_status.h"
//#include "boot_jump.h"

/******************************************************************************
* 函数名称: bool_t service_2E_check_len(const uint8_t* msg_buf, uint16_t msg_dlc)
* 功能说明: 检查 2E 服务数据长度是否合法
* 输入参数: uint16_t msg_dlc         --数据长度
* 输出参数: 无
* 函数返回: TRUE: 合法; FALSE: 非法
* 其它说明: 无
******************************************************************************/
bool_t service_2E_check_len(const uint8_t* msg_buf, uint16_t msg_dlc)
{
    bool_t ret = FALSE;
    
    (void)msg_buf;
    if(msg_dlc > 4)
        ret = TRUE;

    return ret;
}


/******************************************************************************
* 函数名称: void service_2E_WriteDataByIdentifier(const uint8_t* msg_buf, uint16_t msg_dlc)
* 功能说明: 2E 服务 - 根据标识符写入数据
* 输入参数: uint8_t*    msg_buf         --数据首地址
    　　　　uint8_t     msg_dlc         --数据长度
* 输出参数: 无
* 函数返回: 无
* 其它说明: 无
******************************************************************************/
void service_2E_WriteDataByIdentifier(const uint8_t* msg_buf, uint16_t msg_dlc)
{
    uint8_t rsp_buf[8];
    uint16_t did;
    bool_t write_ret = 0;

    did = ((uint16_t)msg_buf[1]) << 8;
    did |= msg_buf[2];

    switch(did)
    {
        case 0x0101: //行驶里程复位
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            if(msg_buf[3] == 0x03)
            {
                switch(msg_buf[4])
                {
                case 0x11: sys_app_mark_set_clear_odo();                   break;  //复位ODO里程
                case 0x22: sys_app_mark_set_clear_engine_hours();          break;  //复位发动机时间
                case 0x33: sys_app_mark_set_clear_trip();                  break;  //复位小计里程
                case 0x44: sys_app_mark_set_clear_trip_time();             break;  //复位小计里程时间
                case 0x55: sys_app_mark_set_clear_inst_fuel_consumption(); break;  //复位瞬时油耗/电耗
                case 0x66: sys_app_mark_set_clear_avg_fuel_consumption();  break;  //复位平均油耗/电耗
                }
            }
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0201: //LCD屏点亮测试
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            uds->lcd_ctrl = msg_buf[3] == 0x03;
            uds->lcd_value = msg_buf[4];
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0202: //背光等级
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            uds->back_light_ctrl = msg_buf[3] == 0x03;
            uds->back_light_value = msg_buf[4];
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0203: //扬声器测试
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            uds->buzzer_ctrl = msg_buf[3] == 0x03;
            uds->buzzer_value = msg_buf[4];
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0310: //车速表测试
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            uds->vehicle_speed_ctrl = msg_buf[3] == 0x03;
            uds->vehicle_speed_value = msg_buf[4];
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0311: //转速表测试
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            uds->engine_speed_rpm_ctrl = msg_buf[3] == 0x03;
            uds->engine_speed_rpm_value = WORD(msg_buf[4],msg_buf[5]);
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0313: //水温表测试
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            uds->engine_coolant_temp_ctrl = msg_buf[3] == 0x03;
            uds->engine_coolant_temp_value = msg_buf[4];
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0314: //燃油表测试
        {
            UdsInfo* uds = data_usdinfo_get_usdinfo();
            uds->residual_fuel_ctrl = msg_buf[3] == 0x03;
            uds->residual_fuel_value = msg_buf[4];
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        case 0x0F01: //打开JTAG
        {
            if(msg_buf[3] == 0x03)
            {
                switch(msg_buf[4])
                {
                case 0x11: Flash_Unlock_JTAG();                   break;  //打开JTAG
                }
            }
            rsp_buf[0] = USD_GET_POSITIVE_RSP(SID_2E);
            rsp_buf[1] = msg_buf[1];
            rsp_buf[2] = msg_buf[2];
            uds_positive_rsp(rsp_buf, 3);
            break;
        }
        default:
            uds_negative_rsp(SID_2E, NRC_REQUEST_OUT_OF_RANGE);
            break;
    }
}


/****************EOF****************/
