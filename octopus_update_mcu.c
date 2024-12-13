
/*******************************************************************************
 * INCLUDES
 */
/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"  // Include platform-specific header for hardware platform details
#include "octopus_log.h"       // Include logging functions for debugging
#include "octopus_task_manager.h" // Include task manager for scheduling tasks

#include "octopus_update_mcu.h" 
#include "octopus_tickcounter.h" // Include tick counter for timing operations
#include "octopus_msgqueue.h"  // Include message queue for inter-process communication
#include "octopus_flash.h"     // Include flash memory handling functions

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

#ifdef TASK_MANAGER_STATE_MACHINE_UPDATE

/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
    uint32_t addr;      // 写入地址
    uint8_t  buff[48];  // 缓冲区
    uint8_t count;      // 缓冲数据长度
}program_buf_t;

typedef enum {
    MCU_REBOOT_ST_IDLE       = (0x00),  //IDLE状态
    MCU_REBOOT_ST_INIT       = (0x01),  //初始化状态
    MCU_REBOOT_ST_SOC_OFF    = (0x02),  //关闭SOC
    MCU_REBOOT_ST_SOC_WAIT   = (0x03),  //等待时间
    MCU_REBOOT_ST_SOC_ON     = (0x04),  //打开SOC
    MCU_REBOOT_ST_MCU_RESET  = (0x05),  //MCU复位
}mcu_reboot_state_t;
/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
static bool update_module_send_handler(ptl_frame_type_t module, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool update_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

static void mcu_reboot_state_proc(void);

/*******************************************************************************
 * GLOBAL VARIABLES
 */


/*******************************************************************************
 * STATIC VARIABLES
 */
//static uint8_t l_u8_acc_status = 0;

#define MCU_REBOOT_TAG_IDLE  (0xFF)//0xFF:空闲
#define MCU_REBOOT_TAG_MCU   (0x00)//0x00:对主控单独复位
#define MCU_REBOOT_TAG_MAC   (0x01)//0x01:对整机断电复位
#define MCU_REBOOT_TAG_SOC   (0x02)//0x02:对核心板单独断电复位

static uint8_t l_u8_reboot = MCU_REBOOT_TAG_IDLE;
static mcu_reboot_state_t l_t_reboot_status = MCU_REBOOT_ST_IDLE;
static uint32_t l_tmr_reboot = 0;

/*******************************************************************************
 * EXTERNAL VARIABLES
 */


/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_update_mcu_init_running(void)
{
    ptl_register_module(M2A_MOD_UPDATE, update_module_send_handler, update_module_receive_handler);
    OTMS(TASK_ID_UPDATE_MCU, OTMS_S_INVALID);
}

void app_update_mcu_start_running(void)
{
    OTMS(TASK_ID_UPDATE_MCU, OTMS_S_ASSERT_RUN);
}

void app_update_mcu_assert_running(void)
{
    ptl_reqest_running(M2A_MOD_UPDATE);
    OTMS(TASK_ID_UPDATE_MCU, OTMS_S_RUNNING);
}

void app_update_mcu_running(void)
{
    mcu_reboot_state_proc();
}

void app_update_mcu_post_running(void)
{
    //PRINT("%s\n", __FUNCTION__);
    ptl_release_running(M2A_MOD_UPDATE);
   // l_u8_mpu_status = SYSTEM_MPU_STATE_INIT;
   // if(MB_ST_OFF != app_power_state_get_mb_state())
    {
        OTMS(TASK_ID_UPDATE_MCU, OTMS_S_ASSERT_RUN);
    }
}


void app_update_mcu_stop_running(void)
{
    //PRINT("%s\n", __FUNCTION__);
    OTMS(TASK_ID_UPDATE_MCU,OTMS_S_INVALID);
}


/*******************************************************************************
 * LOCAL FUNCTIONS IMPLEMENTATION
 */
bool update_module_send_handler(ptl_frame_type_t module, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    //DEV_ASSERT(buff);
    uint8_t tmp[8] = {0};
    //PRINT("update_module_send_handler  MOD %02x  CMD %02x PRARM %04x\n", module, cmd, param);
    if(M2A_MOD_UPDATE == module)
    {
        switch(param1)
        {
        case CMD_MODUPDATE_UPDATE_FW_STATE:  //固件状态更新
        {
            tmp[0] = 0x00;  //0x00：MCU正常运行
            ptl_build_frame(M2A_MOD_UPDATE, CMD_MODUPDATE_UPDATE_FW_STATE, tmp, 1, buff);
            return true;
        }
        default:
            break;
        }
    }
    return false;
}


bool update_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    //DEV_ASSERT(payload);
    //DEV_ASSERT(ackbuff);
    uint8_t tmp = 0;
    if(A2M_MOD_UPDATE == payload->frame_type)
    {
        //PRINTF("MOD %02x CMD %02x pm1 %02x pm2 %02x\n", payload->module, payload->cmd,payload->data[0],payload->data[1]);

        switch(payload->cmd)
        {
        case CMD_MODUPDATE_CHECK_FW_STATE:   //查询固件状态
        {
            printf("CMD_MODUPDATE_CHECK_FW_STATE\n");
            tmp = 0x00; //0x00：MCU正常运行
            ptl_build_frame(M2A_MOD_UPDATE, CMD_MODUPDATE_CHECK_FW_STATE, &tmp, 1, ackbuff);
            return true;
        }
        case CMD_MODUPDATE_UPDATE_FW_STATE:  //固件状态更新
        {
            printf("CMD_MODUPDATE_UPDATE_FW_STATE \n");
            //ACK, no thing to do
            return false;
        }
        case CMD_MODUPDATE_ENTER_FW_UPDATE:  //进入固件更新
        {
            //进入升级模式
            printf("CMD_MODUPDATE_ENTER_FW_UPDATE \n");

            /*printf("MCU_UPDATE_ST_ERASE start erase sector\n");
            INT_SYS_DisableIRQGlobal();
            uint32_t data[4];
            // Write minimum 2 words
            data[0] = 0x12345678;
            data[1] = 0;
            FLASH_DRV_Program(MEMORY_FLAG_START, 2*4, &data);
            INT_SYS_EnableIRQGlobal();
            OSIF_TimeDelay(5);
            printf("MCU_UPDATE_ST_ERASE end erase sector\n");
            //重新开启串口接收
            Uart_Protocol_StartReceive();
            */

            l_u8_reboot = MCU_REBOOT_TAG_MCU;
            l_t_reboot_status = MCU_REBOOT_ST_INIT;

            StartTickCounter(&l_tmr_reboot);
            tmp = 0x01;
            ptl_build_frame(M2A_MOD_UPDATE, CMD_MODUPDATE_ENTER_FW_UPDATE, &tmp, 1, ackbuff);

            return true;
        }
        case CMD_MODUPDATE_EXIT_FW_UPDATE:   //完成固件更新
        {
            printf("CMD_MODUPDATE_EXIT_FW_UPDATE\n");
            tmp = 0x00; //升级正常，后续操作进行
            ptl_build_frame(M2A_MOD_UPDATE, CMD_MODUPDATE_EXIT_FW_UPDATE, &tmp, 1, ackbuff);
            return true;
        }
        case CMD_MODUPDATE_REBOOT:           //系统复位
        {
            printf("CMD_MODUPDATE_REBOOT\n");
            #if 0
            0x00:对主控单独复位
            0x01:对整机断电复位
            0x02:对核心板单独断电复位
            #endif
            l_u8_reboot = payload->data[0]; //重启状态
            l_t_reboot_status = MCU_REBOOT_ST_INIT;
            StartTickCounter(&l_tmr_reboot);
            
            tmp = 0x01; //ack success
            ptl_build_frame(M2A_MOD_UPDATE, CMD_MODUPDATE_REBOOT, &tmp, 1, ackbuff);
            return true;
        }
        default:
            break;
        }
    }
    return false;
}

static void mcu_reboot_state_proc(void)
{
    switch(l_t_reboot_status) {
    case MCU_REBOOT_ST_IDLE:
        break;
    case MCU_REBOOT_ST_INIT:
        if(GetTickCounter(&l_tmr_reboot) > 200)
        {
            if(MCU_REBOOT_TAG_MCU == l_u8_reboot)
            {
                l_t_reboot_status = MCU_REBOOT_ST_MCU_RESET;
           }
            else
            {
                l_t_reboot_status = MCU_REBOOT_ST_SOC_OFF;
            }
        }
        break;
    case MCU_REBOOT_ST_SOC_OFF:
        //soc off
        //TODO 关闭SOC电源
        StartTickCounter(&l_tmr_reboot);
        l_t_reboot_status = MCU_REBOOT_ST_SOC_WAIT;
        break;
    case MCU_REBOOT_ST_SOC_WAIT:
        if(GetTickCounter(&l_tmr_reboot) > 200)
        {
            if(MCU_REBOOT_TAG_MAC == l_u8_reboot)
            {
                l_t_reboot_status = MCU_REBOOT_ST_MCU_RESET;
            }
            else
            {
                l_t_reboot_status = MCU_REBOOT_ST_SOC_ON;
            }
        }
        break;
    case MCU_REBOOT_ST_SOC_ON:
        //soc on
        //TODO 打开SOC电源
        l_t_reboot_status = MCU_REBOOT_ST_IDLE;
        break;
    case MCU_REBOOT_ST_MCU_RESET:
        //System_Reset();
        //printf("MCU_REBOOT_ST_MCU_RESET\n");
        //IAP_Reboot();
    	//IAP_RebootToBootloader();
        break;
    }
}

#endif
