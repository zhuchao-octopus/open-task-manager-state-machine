
/*****************************************************************************/
#include "can_message_rx.h"
#include <stdlib.h>
#include "./can_signal_rx/can_signal_rx.h"
    
    
CAN_msg_rx_config CAN_msg_rx_config_case[CAN_MSG_RX_NUM] = {


    //0x111
    [CAN_MSG_RX_ECU3]  = {.id = 0x111, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 10,
                          .error_status = 0, .is_data_update = 0, .timeout = 10, .timeout_max = 10, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_ECU, .callback = NULL, .timeoutCallback = can_msg_0x111_timeout_function, .normalCallback = can_msg_0x111_normal_function,}, 

    //0x170
    [CAN_MSG_RX_BCM2]  = {.id = 0x170, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 50,
                          .error_status = 0, .is_data_update = 0, .timeout = 50, .timeout_max = 50, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_BCM, .callback = NULL, .timeoutCallback = can_msg_0x170_timeout_function, .normalCallback = can_msg_0x170_normal_function,},
    //0x172
    [CAN_MSG_RX_BCM3]  = {.id = 0x172, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 50,
                          .error_status = 0, .is_data_update = 0, .timeout = 50, .timeout_max = 50, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_BCM, .callback = NULL, .timeoutCallback = can_msg_0x172_timeout_function, .normalCallback = can_msg_0x172_normal_function,},
    //0x173
    [CAN_MSG_RX_BCM4]  = {.id = 0x173, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 50,
                        .error_status = 0, .is_data_update = 0, .timeout = 50, .timeout_max = 50, .format = CAN_MSG_FORMAT_INTEL,
                        .node = CAN_NODE_BCM, .callback = NULL, .timeoutCallback = can_msg_0x173_timeout_function, .normalCallback = can_msg_0x173_normal_function,},
    //0x195
    [CAN_MSG_RX_BCM5]  = {.id = 0x195, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 50,
                          .error_status = 0, .is_data_update = 0, .timeout = 50, .timeout_max = 100, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_BCM, .callback = NULL, .timeoutCallback = can_msg_0x195_timeout_function, .normalCallback = can_msg_0x195_normal_function,},
    //0x312
    [CAN_MSG_RX_CCS1]  = {.id = 0x312, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 40,
                          .error_status = 0, .is_data_update = 0, .timeout = 40, .timeout_max = 40, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_CCS, .callback = NULL, .timeoutCallback = can_msg_0x312_timeout_function, .normalCallback = can_msg_0x312_normal_function,},
    //0x313
    [CAN_MSG_RX_CCS2]  = {.id = 0x313, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 1000,
                          .error_status = 0, .is_data_update = 0, .timeout = 1000, .timeout_max = 1000, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_CCS, .callback = NULL, .timeoutCallback = can_msg_0x313_timeout_function, .normalCallback = can_msg_0x313_normal_function,},
    //0x781ÇëÇó ID
    [CAN_MSG_RX_REQUEST_ID] = {.id = 0x781, .type = CAN_MSG_TYPE_EVENT, .data_len = 8, .period = 500,
                          .error_status = 0, .is_data_update = 0, .timeout = 500, .timeout_max = 500, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_UDS, .callback = NULL, .timeoutCallback = can_msg_0x781_timeout_function, .normalCallback = can_msg_0x781_normal_function,},
    //0x7DF¹¦ÄÜ ID
    [CAN_MSG_RX_FUNCTION_ID] = {.id = 0x7DF, .type = CAN_MSG_TYPE_EVENT, .data_len = 8, .period = 500,
                          .error_status = 0, .is_data_update = 0, .timeout = 500, .timeout_max = 500, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_UDS, .callback = NULL, .timeoutCallback = can_msg_0x7df_timeout_function, .normalCallback = can_msg_0x7df_normal_function,},
    //0x180
    [CAN_MSG_RX_MSC] = {.id = 0x180, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 50,
                          .error_status = 0, .is_data_update = 0, .timeout = 50, .timeout_max = 50, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_MSC, .callback = NULL, .timeoutCallback = can_msg_0x180_timeout_function, .normalCallback = can_msg_0x180_normal_function,},
     //0x188
    [CAN_MSG_RX_SL1] = {.id = 0x188, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 50,
                          .error_status = 0, .is_data_update = 0, .timeout = 50, .timeout_max = 50, .format = CAN_MSG_FORMAT_INTEL,
                          .node = CAN_NODE_SL, .callback = NULL, .timeoutCallback = can_msg_0x188_timeout_function, .normalCallback = can_msg_0x188_normal_function,},
                        
    //0x185
    [CAN_MSG_RX_BCU1] = {.id = 0x185, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 10,
                          .error_status = 0, .is_data_update = 0, .timeout = 10, .timeout_max = 10, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_BCU, .callback = NULL, .timeoutCallback = can_msg_0x185_timeout_function, .normalCallback = can_msg_0x185_normal_function,},
                          
    #if 0
    //0x203
    [CAN_MSG_RX_BCU2] = {.id = 0x203, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 20,
                          .error_status = 0, .is_data_update = 0, .timeout = 20, .timeout_max = 20, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_BCU, .callback = NULL, .timeoutCallback = can_msg_0x203_timeout_function, .normalCallback = can_msg_0x203_normal_function,},

    //0x20B
    [CAN_MSG_RX_BCU3] = {.id = 0x20B, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_BCU, .callback = NULL, .timeoutCallback = can_msg_0x20B_timeout_function, .normalCallback = can_msg_0x20B_normal_function,},
    #endif

    //0x305
    [CAN_MSG_RX_BCU4] = {.id = 0x305, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_BCU, .callback = NULL, .timeoutCallback = can_msg_0x305_timeout_function, .normalCallback = can_msg_0x305_normal_function,},

    //0x207
    [CAN_MSG_RX_BCU5] = {.id = 0x207, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_BCU, .callback = NULL, .timeoutCallback = can_msg_0x207_timeout_function, .normalCallback = can_msg_0x207_normal_function,},

    //0x502
    [CAN_MSG_RX_BCU6] = {.id = 0x502, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 1000,
                          .error_status = 0, .is_data_update = 0, .timeout = 1000, .timeout_max = 1000, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_BCU, .callback = NULL, .timeoutCallback = can_msg_0x502_timeout_function, .normalCallback = can_msg_0x502_normal_function,},

    //0x209
    [CAN_MSG_RX_BCU7] = {.id = 0x209, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_BCU, .callback = NULL, .timeoutCallback = can_msg_0x209_timeout_function, .normalCallback = can_msg_0x209_normal_function,},

    //0x202
    [CAN_MSG_RX_IPU6] = {.id = 0x202, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 10,
                          .error_status = 0, .is_data_update = 0, .timeout = 10, .timeout_max = 50, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_IPU, .callback = NULL, .timeoutCallback = can_msg_0x202_timeout_function, .normalCallback = can_msg_0x202_normal_function,},

    //0x171
    [CAN_MSG_RX_IPU7] = {.id = 0x171, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_IPU, .callback = NULL, .timeoutCallback = can_msg_0x171_timeout_function, .normalCallback = can_msg_0x171_normal_function,},

    //0x204
    [CAN_MSG_RX_IPU1] = {.id = 0x204, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 500, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_IPU, .callback = NULL, .timeoutCallback = can_msg_0x204_timeout_function, .normalCallback = can_msg_0x204_normal_function,},                     

    //0x100
    [CAN_MSG_RX_IPU2] = {.id = 0x100, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 10,
                          .error_status = 0, .is_data_update = 0, .timeout = 10, .timeout_max = 50, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_IPU, .callback = NULL, .timeoutCallback = can_msg_0x100_timeout_function, .normalCallback = can_msg_0x100_normal_function,},

    //0x105
    [CAN_MSG_RX_IPU3] = {.id = 0x105, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 500, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_IPU, .callback = NULL, .timeoutCallback = can_msg_0x105_timeout_function, .normalCallback = can_msg_0x105_normal_function,},

    //0x106
    [CAN_MSG_RX_IPU4] = {.id = 0x106, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_IPU, .callback = NULL, .timeoutCallback = can_msg_0x106_timeout_function, .normalCallback = can_msg_0x106_normal_function,},

    //0x103
    [CAN_MSG_RX_IPU5] = {.id = 0x103, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_IPU, .callback = NULL, .timeoutCallback = can_msg_0x103_timeout_function, .normalCallback = can_msg_0x103_normal_function,},                     

                          
    //0x319
    [CAN_MSG_RX_OBC1] = {.id = 0x319, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_OBC, .callback = NULL, .timeoutCallback = can_msg_0x319_timeout_function, .normalCallback = can_msg_0x319_normal_function,},

     //0x349
    [CAN_MSG_RX_OBC2] = {.id = 0x349, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_OBC, .callback = NULL, .timeoutCallback = can_msg_0x349_timeout_function, .normalCallback = can_msg_0x349_normal_function,},

    //0x3a5
    [CAN_MSG_RX_OBC3] = {.id = 0x3A5, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_OBC, .callback = NULL, .timeoutCallback = can_msg_0x3a5_timeout_function, .normalCallback = can_msg_0x3a5_normal_function,},                      
    
    //0x299
    [CAN_MSG_RX_DCDC1] = {.id = 0x299, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 100,
                          .error_status = 0, .is_data_update = 0, .timeout = 100, .timeout_max = 100, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_DCDC, .callback = NULL, .timeoutCallback = can_msg_0x299_timeout_function, .normalCallback = can_msg_0x299_normal_function,},

    //0x590
    [CAN_MSG_RX_PC1] = {.id = 0x590, .type = CAN_MSG_TYPE_EVENT, .data_len = 8, .period = 40,
                          .error_status = 0, .is_data_update = 0, .timeout = 40, .timeout_max = 40, .format = CAN_MSG_FORMAT_MOTOROLA,
                          .node = CAN_NODE_PC, .callback = NULL, .timeoutCallback = can_msg_0x590_timeout_function, .normalCallback = can_msg_0x590_normal_function,},                     
                     
                          
};
