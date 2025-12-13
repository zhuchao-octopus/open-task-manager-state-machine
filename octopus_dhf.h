#ifndef __OCTOPUS_DHF_H__
#define __OCTOPUS_DHF_H__

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_base.h" //  Base include file for the Octopus project.
#include "octopus_vehicle.h"

#if defined(TASK_MANAGER_STATE_MACHINE_SIF) && defined(TASK_MANAGER_STATE_MACHINE_DHFSIF)

#ifdef __cplusplus
extern "C"
{
#endif

    /******************************************************************************
     * 披萨大黄蜂 2B 协议帧状态结构
     * 对应文档中 Status1~Status9
     ******************************************************************************/
    typedef struct
    {
        uint8_t status1; // 备用或自定义
        uint8_t status2; // 霍尔故障、控制器故障、助力、巡航等
        uint8_t status3; // 电机运行、刹车、三速档等
        uint8_t status4; // P档、倒车、限速等
        uint8_t status5; // 运行电流（单位 0.2A，无符号）
        uint8_t status6; // 速度高字节
        uint8_t status7; // 速度低字节
        uint8_t status8; // 电压百分比(1–100)
        uint8_t status9; // 电量百分比(1–100, +0x80 通信故障)
    } SIF_StatusFrame_t;

    /******************************************************************************
     * @brief  构建一线通（披萨大黄蜂）通信帧
     * @param  buf   输出缓冲区（长度≥12字节）
     * @param  st    状态结构体指针
     * @retval 实际组装的字节数（固定12）
     ******************************************************************************/
    uint8_t SIF_Build_Frame(uint8_t *buf, const SIF_StatusFrame_t *st);
    /******************************************************************************
     * @brief  解析披萨大黄蜂通信帧（液晶仪表→MCU）
     * @param  buf 输入数据缓冲区（12字节）
     * @param  st  输出状态结构体
     * @retval 1=成功解析并通过校验；0=校验失败或非法数据
     ******************************************************************************/
    uint8_t SIF_Parse_Frame(const uint8_t *buf, SIF_StatusFrame_t *st);
    float SIF_Get_SpeedKmh(const SIF_StatusFrame_t *st, uint8_t hall_num_per_turn, float wheel_circum_m);
    void sif_controller_updating(void);

#ifdef __cplusplus
}
#endif

#endif
#endif // __OCTOPUS_DHF_H__
