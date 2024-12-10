/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * @file    octopus_task_manager.c
 * @brief   task state machine
 * @version V1.0.0
 *
 */

/*******************************************************************************
 * INCLUDES
 */

#include "octopus_task_manager.h"
#include "octopus_msgqueue.h"
/*******************************************************************************
 * DEBUG SWITCH MACROS
 */


/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/**
 * \defgroup GROUP_LOCAL_FUNCTIONS OS:KSM:LOCAL_FUNCTIONS
 */


/*******************************************************************************
 * CONSTANTS
 */
/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
inline static void otms_exec_state(otms_id_t task_id, otms_state_t state);

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * STATIC VARIABLES
 */

static otms_state_t lat_otms_state[TASK_ID_MAX_NUM];


/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */

const otms_t *otms_get_config(void)
{
    return lat_otms_config;
}

void task_manager_init(void)
{
	otms_id_t i;
  message_queue_init();//task mssage queue init;
	
	for(i = 0; i < TASK_ID_MAX_NUM; i++)
	{
     otms_exec_state(i, OTMS_S_INIT);
  }
}

void task_manager_start(void)
{
	otms_id_t i;
	for(i = 0; i < TASK_ID_MAX_NUM; i++)
	{
     otms_exec_state(i, OTMS_S_START);
	}
}

void task_manager_stop(void)
{
	otms_id_t i;

	for(i = 0; i < TASK_ID_MAX_NUM; i++)
	{
     otms_exec_state(i, OTMS_S_STOP);
	}
}

void task_manager_run(void)
{
	otms_id_t i;

	for(i = 0; i < TASK_ID_MAX_NUM; i++)
	{
     otms_exec_state(i, lat_otms_state[i]);
	}
}

void otms_set_state(otms_id_t task_id, otms_state_t state)
{
	if(task_id < TASK_ID_MAX_NUM)
	{
		lat_otms_state[task_id] = state;
	}
	else
	{
	    assert(0);
	}
}

otms_state_t otms_get_state(otms_id_t task_id)
{
	otms_state_t state;

	if(task_id < TASK_ID_MAX_NUM)
	{
		state = lat_otms_state[task_id];
	}
	else
	{
		state = OTMS_S_INVALID;
		assert(0);
	}
	return state;
}

void otms_on_enter_run(void)
{
    otms_id_t i = 0;

    for (i = 0; i < TASK_ID_MAX_NUM; i ++)
    {
        if (otms_get_state(i) > OTMS_S_POST_RUN)
        {
            otms_set_state(i, OTMS_S_START);
        }
    }
}

void otms_on_exit_post_run(void)
{
    otms_id_t i = 0;

    for (i = 0; i < TASK_ID_MAX_NUM; i ++)
    {
        otms_set_state(i, OTMS_S_STOP);
    }
}

static void otms_exec_state(otms_id_t task_id, otms_state_t state)
{
	const otms_t *cfg = otms_get_config();

    if ((task_id < TASK_ID_MAX_NUM) && (NULL != cfg) && (cfg[task_id].state_limit > state))
    {
        if (NULL != cfg[task_id].func[state])
        {
        	cfg[task_id].func[state]();
        }
    }
}


