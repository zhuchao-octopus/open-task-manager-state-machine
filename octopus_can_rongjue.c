/******************************************************************************
 * @file    octopus_can_rongjue.c
 * @author  Octopus Embedded Team
 * @version V1.0
 * @date    2025-10-14
 * @brief   CAN communication module for Rongjue 500 vehicle model.
 *
 * This module defines the CAN message dispatcher and sender framework
 * for the Rongjue 500 model. It handles message routing, decoding,
 * and update of vehicle information received from the CAN bus.
 *
 * ----------------------------------------------------------------------------
 * Revision History:
 * Date         Author              Notes
 * 2025-10-14   Macky Lee           Initial version
 * ----------------------------------------------------------------------------
 ******************************************************************************/

/*==============================================================================
 * Includes
 *============================================================================*/

/**
 * @file octopus_can_rongjue.h
 * @brief Header defining Rongjue CAN message structures and constants.
 */
#include "octopus_can_rongjue.h"

/**
 * @file octopus_platform.h
 * @brief Platform-specific definitions and initialization utilities.
 */
#include "octopus_platform.h"

/**
 * @file octopus_tickcounter.h
 * @brief Tick Counter: provides timing and delay utilities.
 */
#include "octopus_tickcounter.h"

/**
 * @file octopus_message.h
 * @brief Message IDs and inter-task communication definitions.
 */
#include "octopus_message.h"

/**
 * @file octopus_msgqueue.h
 * @brief Message Queue API for inter-task message passing.
 */
#include "octopus_msgqueue.h"

/**
 * @file octopus_uart_ptl.h
 * @brief UART Protocol Layer: handles protocol-level UART operations.
 */
#include "octopus_uart_ptl.h"

/**
 * @file octopus_uart_upf.h
 * @brief UART UPF Layer: manages UART data framing and parsing.
 */
#include "octopus_uart_upf.h"
#include "Octopus_utils.h"
/*==============================================================================
 * Configuration Section
 *============================================================================*/

#ifdef CUSTOMER_MODEL_RONGJUE_500

static RongjueVehicleInfo_t vehicle_info;

/* helper: combine two bytes where high is first (big-endian) */
static inline uint16_t be_u16(const uint8_t high, const uint8_t low)
{
   return (uint16_t)((high << 8) | low);
}

/* helper: interpret signed 16-bit from two bytes (big-endian) */
static inline int16_t be_i16(const uint8_t high, const uint8_t low)
{
   return (int16_t)be_u16(high, low);
}
/**
 * @brief Calculate speed from rpm example stub (user should adapt).
 * @note This is placeholder — replace with actual calibration.
 */
static float Calculate_Speed_From_RPM(uint32_t rpm, uint16_t tire_radius_mm, uint8_t gear_ratio)
{
   /* Wheel circumference (m) = 2*pi*r (r in meters) */
   float r_m = tire_radius_mm / 1000.0f;
   float circumference_m = 2.0f * 3.14159265f * r_m;
   /* wheel rev per minute = rpm / gear_ratio (if rpm is motor rpm) */
   float wheel_rev_per_min = (gear_ratio == 0) ? 0.0f : (rpm / (float)gear_ratio);
   float speed_m_per_min = wheel_rev_per_min * circumference_m;
   float speed_kmh = speed_m_per_min * 60.0f / 1000.0f;
   return speed_kmh;
}

/**
 * @brief Parse CAN message according to Rongjue EV04 protocol.
 *
 * Supported IDs:
 *  - 0x100 : Battery current / charge status (byte2-3)
 *  - 0x101 : Battery SOC (byte5)
 *  - 0x102 : Battery fault (byte4-5)
 *  - 0x105 : Battery temperature (byte2-3)
 *  - 0x201 : Controller: READY (byte0 bit0), RPM (byte5 high, byte4 low), ODO (byte7 high, byte6 low)
 *  - 0x202 : Controller: mode/gear (byte1 bits 6..4)
 *  - 0x2FF : Controller/motor faults (byte0 bit0 -> byte1 code ; byte2 bit0 -> byte3 code)
 *  - 0x602 : Instrument speed (byte0)
 *  - 0x603 : Instrument ODO digits (byte0..5)
 *  - 0x604 : Instrument TRIP digits (byte0..3)
 *
 * Returns true if handled (ID recognized), false otherwise.
 */
bool can_message_dispatcher(const CanQueueMsg_t *queue_msg)
{
   if (queue_msg == NULL)
      return false;

   const uint32_t id = queue_msg->std_id;
   const uint8_t *d = queue_msg->data;
   uint8_t dlc = queue_msg->data_len;
   bool handled = true;

   switch (id)
   {
   /* ------------------ Battery replies (queried by RTR) ------------------ */
   case 0x100:
      /* Battery current: byte2-3 (high byte first -> byte2 high, byte3 low)
         Protocol text said "byte2-3计算出电流". Use signed if current can be negative.
         Here we interpret as signed 16-bit; scale depends on BMS spec (assume 0.1A per unit as example). */
      if (dlc >= 4) /* need at least bytes 0..3 present */
      {
         int16_t cur_raw = be_i16(d[2], d[3]);
         ////g_rongjue_info.battery_current = cur_raw * 0.1f; /* example scale: 0.1 A/LSB */
      }
      break;

   case 0x101:
      /* Battery SOC: byte5 = percentage (0-100) */
      if (dlc >= 6)
      {
         ////g_rongjue_info.battery_soc = d[5];
      }
      break;

   case 0x102:
      /* Battery fault: byte4-5 -> fault code (big-endian) */
      if (dlc >= 6)
      {
         //// g_rongjue_info.battery_fault = be_u16(d[4], d[5]);
      }
      break;

   case 0x105:
      /* Battery temperature: byte2-3 -> temp (interpret signed or unsigned per BMS).
         Example: raw * 0.1 => degC */
      if (dlc >= 4)
      {
         int16_t t_raw = be_i16(d[2], d[3]);
         ////g_rongjue_info.battery_temp = t_raw * 0.1f;
      }
      break;

   /* ------------------ Controller periodic frames ------------------ */
   case 0x201:
      /* byte0 bit0 -> READY lamp (1 = on)
         RPM: "Byte4-5为转速值，byte5为高位，byte4为低位" => rpm = (byte5<<8)|byte4
         ODO: "Byte6-7为里程值，byte7为高位，byte6为低位" => odo = (byte7<<8)|byte6 */
      if (dlc >= 8)
      {
         //// g_rongjue_info.ready_on = (d[0] & 0x01) ? true : false;

         uint16_t rpm = be_u16(d[5], d[4]); /* byte5 high, byte4 low */
         /* Example conversion: if rpm is motor rpm, convert to speed with stub */
         ////g_rongjue_info.speed_kmh = Calculate_Speed_From_RPM(rpm, 300 /*mm assume*/, 10 /*gear example*/);

         uint16_t odo_raw = be_u16(d[7], d[6]); /* byte7 high, byte6 low */
         /* protocol doesn't state unit: store raw and as km */
         ////g_rongjue_info.odometer_km = (uint32_t)odo_raw;
      }
      break;

   case 0x202:
      /* byte1 bits 6..4 -> value
         Mapping per doc:
           001 -> ECO mode
           010 -> Sports mode
           011 -> R (reverse)
           111 -> P
           100 -> Normal
      */
      if (dlc >= 2)
      {
         uint8_t v = (d[1] >> 4) & 0x07; /* bits 6..4 shifted down */
         switch (v)
         {
         case 0x1:
            ////g_rongjue_info.drive_mode = 1;
            ////g_rongjue_info.gear = 0;
            break; /* ECO */
         case 0x2:
            /// g_rongjue_info.drive_mode = 2;
            /// g_rongjue_info.gear = 0;
            break; /* SPORT */
         case 0x3:
            /// g_rongjue_info.gear = 'R';
            break;
         case 0x7:
            /// g_rongjue_info.gear = 'P';
            break;
         case 0x4:
            ////g_rongjue_info.drive_mode = 0;
            //// g_rongjue_info.gear = 0;
            break; /* Normal */
         default:  /* unknown */
            break;
         }
      }
      break;

   case 0x2FF:
      /* ECU and motor faults:
         byte0 bit0 == 1 -> ECU fault present, byte1 = ECU fault code
         byte2 bit0 == 1 -> motor fault present, byte3 = motor fault code */
      if (dlc >= 4)
      {
         if (d[0] & 0x01)
         {
            ////g_rongjue_info.ecu_fault_code = d[1];
         }
         else
         {
            ////g_rongjue_info.ecu_fault_code = 0;
         }

         if (d[2] & 0x01)
         {
            ////g_rongjue_info.motor_fault_code = d[3];
         }
         else
         {
            ////g_rongjue_info.motor_fault_code = 0;
         }
      }
      break;

   /* ------------------ Instrument frames ------------------ */
   case 0x602:
      /* Instrument speed: byte0 unsigned, in current chosen unit */
      if (dlc >= 1)
      {
         /* store as km/h assuming instrument is in km/h; convert if needed */
         ////g_rongjue_info.speed_kmh = (float)d[0];
      }
      break;

   case 0x603:
      /* ODO digits: byte0..5 = decimal, units, tens, hundreds, thousands, ten-thousands
         byte0 = decimal (tenths), byte1 = units, byte2 = tens, ...
         Example: bytes = {1,2,3,4,5,6} -> ODO = 6 5 4 3 2 .1 => 65432.1
      */
      if (dlc >= 6)
      {
         uint32_t digits = 0;
         digits += (uint32_t)d[5] * 10000u;              /* byte5 ten-thousands */
         digits += (uint32_t)d[4] * 1000u;               /* byte4 thousands */
         digits += (uint32_t)d[3] * 100u;                /* byte3 hundreds */
         digits += (uint32_t)d[2] * 10u;                 /* byte2 tens */
         digits += (uint32_t)d[1];                       /* byte1 units */
         float odo_full = digits + ((float)d[0] * 0.1f); /* add decimal */
         ////g_rongjue_info.odometer_float = odo_full;
         ////g_rongjue_info.odometer_km = digits; /* integer part */
      }
      break;

   case 0x604:
      /* TRIP: byte0..3 = decimal, units, tens, hundreds (unit = km, with decimal)
         Example: bytes {x, u, t, h} -> trip = h*100 + t*10 + u + x*0.1
      */
      if (dlc >= 4)
      {
         uint32_t trip_whole = (uint32_t)d[3] * 100u + (uint32_t)d[2] * 10u + (uint32_t)d[1];
         float trip = trip_whole + ((float)d[0] * 0.1f);
         /* store to odometer_float temporarily or a separate trip field */
         /* for demo, store into odometer_float (replace with real trip field) */
         ////g_rongjue_info.odometer_float = trip;
      }
      break;

   default:
      handled = false;
      break;
   }

   return handled;
}

/**
 * @brief Send Rongjue 500 battery query frame (remote frame).
 * @param message_id One of {0x100, 0x101, 0x102, 0x105}
 * @return true if the frame was sent successfully, false otherwise.
 */
bool can_message_sender(const uint16_t message_id)
{
   uint8_t g_bms1[8] = {0};

   switch (message_id)
   {
      // ===== Rongjue Battery Query Frames (Remote Frames) =====
   case 0x100: // Battery current / charging status
      CAN_SEND_STRUCT_STD(0x100, &g_bms1);
      break;

   case 0x101: // Battery SOC
      CAN_SEND_STRUCT_STD(0x101, &g_bms1);
      break;

   case 0x102: // Battery fault
      CAN_SEND_STRUCT_STD(0x102, &g_bms1);
      break;

   case 0x105: // Battery temperature
      CAN_SEND_STRUCT_STD(0x105, &g_bms1);
      break;

   default:
      // Unknown message ID
      return false;
   }

   return true;
}

static void CAN_update_vehicle_infor(const CanQueueMsg_t *msg)
{
   send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_METER, 0);
   send_message(TASK_MODULE_IPC, MSG_OTSM_DEVICE_CAR_EVENT, MSG_IPC_CMD_CAR_GET_INDICATOR_INFO, 0);
}

#endif /* CUSTOMER_MODEL_RONGJUE_500 */

/************************ (C) COPYRIGHT Octopus Embedded Team *****END OF FILE****/
