/**
 * @file	halPeripheral.c
 * @author	chipsea
 * @brief	
 * @version	0.1
 * @date	2020-11-30
 * @copyright Copyright (c) 2020, CHIPSEA Co., Ltd.
 * @note
 */

/*********************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"
#include "octopus_flash_hal.h"
#include "octopus_log.h"

void hal_flash_init(uint8_t task_id)
{	
#ifdef PLATFORM_CST_OSAL_RTOS
	///HalDMAInit();
	///HalDMAInitChannel(dma_cfg);
 #endif
	LOG_LEVEL(F_NAME,"hal flash init\r\n");
}

void hal_flash_read_to_buff(uint32_t addr, uint8_t *buf, uint32_t len)
{
     #ifdef PLATFORM_CST_OSAL_RTOS
	 HalFlashRead(addr, buf, len);
     #endif
}

void hal_flash_write_buff(uint32_t addr, uint8_t *buf, uint32_t len)
{
     #ifdef PLATFORM_CST_OSAL_RTOS
	 HalFlashWrite(addr, buf, len);
     #endif
}

void hal_flash_write_buff_dma(uint32_t addr, uint8_t *buf, uint32_t len)
{
     #ifdef PLATFORM_CST_OSAL_RTOS
	 HalFlashWriteByDma(addr, buf, len);
     #endif
}

