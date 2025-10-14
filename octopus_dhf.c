/******************************************************************************
 * File Name    : Octopus_dhf.c
 * Author       : Octopus Embedded Systems Team
 * Description  : 披萨大黄蜂一线通（2B方案）协议上层实现
 *                - 帧组包（含加密+校验）
 *                - 流水号自动递增
 *                - 完全兼容 SIF 2.0 通信规范
 * Created on   : 2025-10-13
 ******************************************************************************/

#include "Octopus_dhf.h"
#include "Octopus_sif.h"
#include "Octopus_utils.h"
#include "octopus_tickcounter.h" // Include tick counter for timing operations
#include "octopus_msgqueue.h"    // Include message queue header for task communication
#include "octopus_message.h"     // Include message id for inter-task communication
#include "octopus_uart_ptl.h"    // Include UART protocol header
#include "octopus_vehicle.h"

#if defined(TASK_MANAGER_STATE_MACHINE_SIF) && defined(TASK_MANAGER_STATE_MACHINE_DHFSIF)
static uint8_t sif_buff[12] = {0}; // Buffer for storing SIF data

/******************************************************************************
 * @brief  计算加密码 PlusCode
 ******************************************************************************/
static uint8_t SIF_Calc_PlusCode(uint8_t seq_l, uint8_t seq_h)
{
    uint8_t plus;
    plus = seq_l + 0x2C;
    plus ^= 0x6F;
    plus += 0xDF;
    plus ^= 0x7A;
    plus += (seq_h & 0x0F);
    plus ^= 0x2B;
    plus += 0x0D;
    plus ^= 0xC2;
    plus &= 0x7F;
    return plus;
}

/******************************************************************************
 * @brief  构建披萨大黄蜂通信帧（MCU→液晶仪表）
 ******************************************************************************/
uint8_t SIF_Build_Frame(uint8_t *buf, const SIF_StatusFrame_t *st)
{
    static uint16_t seq = 0; // 12-bit 流水号
    uint8_t seq_l = seq & 0xFF;
    uint8_t seq_h = (seq >> 8) & 0x0F;
    uint8_t plus;

    if (buf == NULL || st == NULL)
        return 0;

    /* Step 1. 计算加密码 */
    plus = SIF_Calc_PlusCode(seq_l, seq_h);

    /* Step 2. 组装帧内容 */
    buf[0] = 0x2B;                                // Device_code 固定
    buf[1] = seq_l;                               // 流水号低 8 位
    buf[2] = (seq_h << 4) | (st->status1 & 0x0F); // 高4位流水号 + 低4位状态
    buf[3] = st->status2 + plus;
    buf[4] = st->status3 + plus;
    buf[5] = st->status4 + plus;
    buf[6] = st->status5; // 电流不加密
    buf[7] = st->status6 + plus;
    buf[8] = st->status7 + plus;
    buf[9] = st->status8 + plus;
    buf[10] = st->status9 + plus;

    /* Step 3. 校验（异或和） */
    uint8_t checksum = 0;
    for (int i = 0; i <= 10; i++)
        checksum ^= buf[i];
    buf[11] = checksum;

    /* Step 4. 流水号递增 */
    seq++;
    if (seq >= 0x0FFF)
        seq = 0;

    return 12;
}

/******************************************************************************
 * @brief  解析披萨大黄蜂通信帧（液晶仪表→MCU）
 ******************************************************************************/
uint8_t SIF_Parse_Frame(const uint8_t *buf, SIF_StatusFrame_t *st)
{
    if (buf == NULL || st == NULL)
        return 0;

    /* Step 1. 校验 Device_code */
    if (buf[0] != 0x2B)
        return 0;

    /* Step 2. 提取流水号 */
    uint8_t seq_l = buf[1];
    uint8_t seq_h = (buf[2] >> 4) & 0x0F;

    /* Step 3. 计算 PlusCode */
    uint8_t plus = SIF_Calc_PlusCode(seq_l, seq_h);

    /* Step 4. 计算校验和 */
    uint8_t checksum = 0;
    for (int i = 0; i <= 10; i++)
        checksum ^= buf[i];

    if (checksum != buf[11])
        return 0; // 校验失败

    /* Step 5. 解密还原状态字段 */
    st->status1 = buf[2] & 0x0F;
    st->status2 = buf[3] - plus;
    st->status3 = buf[4] - plus;
    st->status4 = buf[5] - plus;
    st->status5 = buf[6]; // 不加密
    st->status6 = buf[7] - plus;
    st->status7 = buf[8] - plus;
    st->status8 = buf[9] - plus;
    st->status9 = buf[10] - plus;

    return 1; // 成功解析
}

/******************************************************************************
 * @brief  根据 status6/status7 解码车速 (km/h)
 * @param  st   已解析好的 SIF_StatusFrame_t
 * @param  hall_num_per_turn 每转霍尔触发次数 (6/12 等)
 * @param  wheel_circum_m    轮胎周长 (m)
 * @retval 浮点型速度 km/h
 ******************************************************************************/
float SIF_Get_SpeedKmh(const SIF_StatusFrame_t *st, uint8_t hall_num_per_turn, float wheel_circum_m)
{
    if (!st || hall_num_per_turn == 0 || wheel_circum_m <= 0)
        return 0.0f;

    uint16_t hall_count = ((uint16_t)st->status6 << 8) | st->status7;
    float speed = (hall_count * wheel_circum_m * 7.2f) / hall_num_per_turn;
    return speed;
}

/******************************************************************************
 * @brief  解析披萨大黄蜂协议中的三速档位状态
 * @param  st  已解析的状态结构
 * @retval 0=无三速控制，1=低速，2=中速，3=高速
 ******************************************************************************/
uint8_t SIF_Get_SpeedMode(const SIF_StatusFrame_t *st)
{
    if (!st)
        return 0;

    uint8_t s3 = st->status3;

    // 判断匹配关系
    if ((s3 & 0xE0) == 0x80)
        return 3; // 高速（D7=1）
    if ((s3 & 0xE0) == 0x60)
        return 2; // 中速（D6,D5=1）
    if ((s3 & 0xE0) == 0x50)
        return 1; // 低速（D6,D4=1）
    return 0;     // 无三速控制
}

/**
 * @brief Read and update SIF (Smart Interface Frame) data.
 *
 * This function is periodically called (e.g. every 50~100ms) to fetch
 * the latest SIF data packet from the controller, decode its fields,
 * and update the system metrics such as speed, current, gear, etc.
 */
void sif_controller_updating(void)
{
    uint8_t read_count = sif_read_data(sif_buff, sizeof(sif_buff));
    carinfo_indicator_t temp_carinfo_indicator = {0};
    carinfo_meter_t temp_carinfo_meter = {0};

    SIF_StatusFrame_t frame;
    if (read_count <= 0)
        return;

    // 从 SIF 接收到一帧并解析
    if (SIF_Parse_Frame(sif_buff, &frame))
    {
        temp_carinfo_meter.speed_actual = SIF_Get_SpeedKmh(&frame, 6, 1.8f) * 10;
        temp_carinfo_meter.gear = SIF_Get_SpeedMode(&frame);

        SETBIT(temp_carinfo_indicator.brake, GETBIT(sif_buff[3], 5));
        SETBIT(temp_carinfo_indicator.brake, GETBIT(sif_buff[4], 1));

        temp_carinfo_meter.speed_limit = GETBIT(sif_buff[4], 0);
        temp_carinfo_indicator.reverse = GETBIT(sif_buff[4], 2);
        temp_carinfo_indicator.ready = GETBIT(sif_buff[4], 7);
			  temp_carinfo_indicator.cruise_control = GETBIT(sif_buff[2], 2);
				temp_carinfo_indicator.walk_assist = GETBIT(sif_buff[2], 1);
			
        carinfo_add_error_code(ERROR_CODE_HALLSENSOR_ABNORMALITY, GETBIT(sif_buff[2], 6), false);
        carinfo_add_error_code(ERROR_CODE_THROTTLE_HALLSENSOR_ABNORMALITY, GETBIT(sif_buff[2], 5), false);
        carinfo_add_error_code(ERROR_CODE_CONTROLLER_ABNORMALITY, GETBIT(sif_buff[2], 4), false);
        carinfo_add_error_code(ERROR_CODE_LOW_VOLTAGE_PROTECTION, GETBIT(sif_buff[2], 3), false);
        carinfo_add_error_code(ERROR_CODE_MOTOR_ABNORMALITY, GETBIT(sif_buff[2], 0), false);
				carinfo_add_error_code(ERROR_CODE_CONTROLLER_PROTECTION_TRIGGER, GETBIT(sif_buff[3], 4), false);
			  
			  carinfo_add_error_code(ERROR_CODE_COMMUNICATION_ABNORMALITY,GETBIT(sif_buff[9], 7), true);
			
        if (!is_struct_equal(&temp_carinfo_meter, &lt_carinfo_meter, sizeof(carinfo_meter_t)))
        {
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_METER, 0);
        }

        if (!is_struct_equal(&temp_carinfo_indicator, &lt_carinfo_indicator, sizeof(carinfo_indicator_t)))
        {
            send_message(TASK_MODULE_IPC, MSG_OTSM_DEVICE_CAR_EVENT, MSG_IPC_CMD_CAR_GET_INDICATOR_INFO, 0);
        }
    }
}

#endif
