/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 *
 */
#ifndef __OCTOPUS_TASK_MANAGER_COM_UART_PTL_H__
#define __OCTOPUS_TASK_MANAGER_COM_UART_PTL_H__

/*******************************************************************************
 * INCLUDES
 */
 
#include "octopus_platform.h"

#ifdef __cplusplus
extern "C"{
#endif

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

/*******************************************************************************
 * MACROS
 */
#define PTL_UART_CHANNEL            		1
#define PTL_FRAME_HEADER_SIZE           5
#define PTL_FRAME_MIN_SIZE              7
#define PTL_FRAME_MAX_SIZE              255
#define PTL_FRAME_DATA_START          	5
#define PTL_HEADER             				  0xFA

/*******************************************************************************
 * TYPEDEFS
 */
typedef enum UartChannel
{
  UartChannel_Uart0 = 0,
  UartChannel_Uart1 = 1,
	UartChannel_Uart2 = 2,
} UartChannel;

typedef enum{
    /*MCU -> APP*/
    M2A_PTL_HEADER    = 0x55,        //(MCU -> APP)FRAME HEADER
    /*APP -> MCU*/
    A2M_PTL_HEADER    = 0xAA,        //(APP -> MCU)FRAME HEADER
}ptl_frame_header_t;

typedef enum{ //module id
    /*MCU -> APP*/
    M2A_MOD_SYSTEM    = 0x00,        //(MCU -> APP)系统初始化
    M2A_MOD_UPDATE    = 0x01,        //(MCU -> APP)系统升级
    M2A_MOD_TRANSFER  = 0x02,        //(MCU -> APP)数据透传
    M2A_MOD_METER     = 0x03,        //(MCU -> APP)仪表
    M2A_MOD_INDICATOR = 0x04,        //(MCU -> APP)指示灯
    M2A_MOD_DRIV_INFO = 0x05,        //(MCU -> APP)行车信息
    M2A_MOD_SETUP     = 0x06,        //(MCU -> APP)设置
    /*APP -> MCU*/
    A2M_MOD_SYSTEM    = 0x80,        //(APP -> MCU)系统初始化
    A2M_MOD_UPDATE    = 0x81,        //(APP -> MCU)系统升级
    A2M_MOD_TRANSFER  = 0x82,        //(APP -> MCU)数据透传
    A2M_MOD_METER     = 0x83,        //(APP -> MCU)仪表
    A2M_MOD_INDICATOR = 0x84,        //(APP -> MCU)指示灯
    A2M_MOD_DRIV_INFO = 0x85,        //(APP -> MCU)行车信息
    A2M_MOD_SETUP     = 0x86,        //(APP -> MCU)设置

    /*用于判断MODULE的有效性*/
    /*MCU -> APP*/
    M2A_MOD_START   = M2A_MOD_SYSTEM,
    M2A_MOD_END     = M2A_MOD_SETUP,
    /*APP -> MCU*/
    A2M_MOD_START   = A2M_MOD_SYSTEM,
    A2M_MOD_END     = A2M_MOD_SETUP,
}ptl_frame_type_t;

typedef enum{
    /*MOD_SYSTEM*/
    CMD_MODSYSTEM_HANDSHAKE         = 0x00,      //系统握手
    CMD_MODSYSTEM_ACC_STATE         = 0x01,      //ACC状态
    CMD_MODSYSTEM_APP_STATE         = 0x02,      //应用画面状态
    CMD_MODSYSTEM_POWER_ON          = 0x03,       //电源
	CMD_MODSYSTEM_POWER_OFF         = 0x04,      //关闭电源

    /*MOD_UPDATE*/
    CMD_MODUPDATE_CHECK_FW_STATE    = 0x06,      //查询固件状态
    CMD_MODUPDATE_UPDATE_FW_STATE   = 0x07,      //固件状态更新
    CMD_MODUPDATE_ENTER_FW_UPDATE   = 0x08,      //进入固件更新
    CMD_MODUPDATE_EXIT_FW_UPDATE    = 0x09,      //完成固件更新
    CMD_MODUPDATE_SEND_FW_DATA      = 0x0A,      //发送固件数据
    CMD_MODUPDATE_REBOOT            = 0x0B,      //系统复位
    /*MOD_TRANSFER*/
    CMD_MODTRANSFER_A2M             = 0x0C,      //A2M数据透传
    CMD_MODTRANSFER_M2A             = 0x0D,      //M2A数据透传
    /*MOD_METER*/
    CMD_MODMETER_RPM_SPEED          = 0x0E,      //转速表/车速表
    CMD_MODMETER_FUEL_TEMPTER       = 0x0F,      //燃油表/水温表
    CMD_MODMETER_SOC                = 0x10,      //SOC表
    /*MOD_INDICATOR*/
    CMD_MODINDICATOR_INDICATOR      = 0x11,      //指示灯
    CMD_MODINDICATOR_ERROR_INFO     = 0x12,      //故障信息
    /*MOD_DRIV_INFO*/
    CMD_MODDRIVINFO_ODO             = 0x14,      //行驶里程
    CMD_MODDRIVINFO_DRIV_DATA       = 0x15,      //行驶数据
    CMD_MODDRIVINFO_GEAR            = 0x16,      //档位信息
    CMD_MODDRIVINFO_NAVI            = 0x17,      //简易导航
    CMD_MODDRIVINFO_DRIV_DATA_CLEAR = 0x18,      //行驶数据复位
    /*MOD_SETUP*/
    CMD_MODSETUP_UPDATE_TIME        = 0x19,      //时间更新
    CMD_MODSETUP_SET_TIME           = 0x1A,      //设置时间
    CMD_MODSETUP_KEY                = 0x1B,      //按键
	
}ptl_frame_cmd_t;

typedef struct
{
    uint16_t 						size;
    uint8_t 						buff[PTL_FRAME_MAX_SIZE];
}ptl_proc_buff_t;

typedef struct{
    ptl_frame_type_t 		frame_type;
    ptl_frame_cmd_t 		cmd;
    uint8_t 						data_len;
    uint8_t 					 *data;
}ptl_frame_payload_t;

/**
 * module handler for receive and send
 */
typedef bool (*module_send_handler_t)			(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
typedef bool (*module_receive_handler_t)	(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * GLOBAL VARIABLES DECLEAR
 */

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

void com_uart_init_running(void);
void com_uart_start_running(void);
void com_uart_assert_running(void);
void com_uart_running(void);
void com_uart_post_running(void);
void com_uart_stop_running(void);

bool com_uart_reqest_running(uint8_t source);
bool com_uart_release_running(uint8_t source);

void com_uart_set_opposite_running(bool running);
bool com_uart_is_com_error(void);

void ptl_com_uart_register_module(ptl_frame_type_t frame_type, module_send_handler_t send_handler, module_receive_handler_t receive_handler);
void ptl_com_uart_build_frame(ptl_frame_type_t frame_type, ptl_frame_cmd_t cmd, uint8_t *data, uint8_t datelen, ptl_proc_buff_t *framebuff);
void ptl_com_uart_build_frame_header(ptl_frame_type_t frame_type, ptl_frame_cmd_t cmd, uint8_t datalen, ptl_proc_buff_t *buff);
void ptl_com_uart_receive_handler(uint8_t data);
void ptl_frame_analysis_handler(void);
uint8_t ptl_com_uart_get_checksum(uint8_t *data, uint8_t len);


#ifdef __cplusplus
}
#endif


#endif
