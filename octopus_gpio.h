/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 *
 */
#ifndef __OCTOPUS_TASK_MANAGER_GPIO_H__
#define __OCTOPUS_TASK_MANAGER_GPIO_H__

/*******************************************************************************
 * INCLUDES
 */
 
#include "octopus_platform.h" 
#include "octopus_gpio_hal.h"

#ifdef __cplusplus
extern "C"{
#endif


/*******************************************************************************
 * MACROS
 */
#define GPIO_STATUS_REDUNDANCY 						8
#define GPIO_KEY_STATUS_REDUNDANCY 				5
#define GPIO_KEY_STATUS_MAX_REDUNDANCY		255

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * TYPEDEFS
 */

typedef struct
{
    uint8_t key;
	bool pressed;
	bool pressed_l;	//long_pressed
	bool released;
	bool dispatched;
	uint8_t count1;
} GPIO_KEY_STATUS;

typedef struct
{
  bool offon; //0/1
  bool changed;
  uint8_t count1;
  uint8_t count2;
} GPIO_STATUS;

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

void app_gpio_init_running(void);
void app_gpio_start_running(void);
void app_gpio_assert_running(void);
void app_gpio_running(void);
void app_gpio_post_running(void);
void app_gpio_stop_running(void);

bool IsAccOn(void);


#ifdef __cplusplus
}
#endif


#endif
