/********************************** (C) COPYRIGHT *******************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * @file    octopus_task_manager_carinfo.h
 * @brief   Definitions for car information management in Octopus Task Manager.
 *
 * @details This file defines structures, enums, and function declarations used to
 *          represent and manage the state of vehicle components and status indicators.
 *          It includes driving status, mechanical faults, battery and motor states,
 *          and light indicators. It supports embedded vehicle control applications.
 *
 * @note    Intended for microcontrollers by Nanjing Qinheng Microelectronics.
 * @author  Octopus Team
 * @version 1.0.0
 * @date    2024-12-09
 *******************************************************************************/

#ifndef __OCTOPUS_TASK_MANAGER_CARINFO_H__
#define __OCTOPUS_TASK_MANAGER_CARINFO_H__

#include "octopus_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/**
 * @brief Driving statistical form
 */
typedef struct {
    uint16_t avgEnergyConsumption;  // Average energy consumption (0.1 kWh/100km or L/100km depending on fuel type)
    uint16_t travelTime;            // Total travel time in minutes
    uint16_t avgSpeed;              // Average speed (unit: 0.1 km/h)
} carinfo_drivinfo_form_t;

/**
 * @brief Enumeration for gear positions
 */
typedef enum {
    DRIVINFO_GEAR_UNKNOWN = 0x00,  // Unknown gear
    DRIVINFO_GEAR_MANUAL_1 = 0x01, // Manual gear 1
    DRIVINFO_GEAR_MANUAL_2 = 0x02, // Manual gear 2
    DRIVINFO_GEAR_MANUAL_3 = 0x03, // Manual gear 3
    DRIVINFO_GEAR_MANUAL_4 = 0x04, // Manual gear 4
    DRIVINFO_GEAR_MANUAL_5 = 0x05, // Manual gear 5
    DRIVINFO_GEAR_MANUAL_6 = 0x06, // Manual gear 6
    DRIVINFO_GEAR_MANUAL_7 = 0x07, // Manual gear 7
    DRIVINFO_GEAR_REVERSE  = 0x0A, // Reverse
    DRIVINFO_GEAR_PARK     = 0x0B, // Park
    DRIVINFO_GEAR_NEUTRAL  = 0x0C, // Neutral
    DRIVINFO_GEAR_DRIVE    = 0x0D, // Drive
    DRIVINFO_GEAR_SPORT    = 0x0E, // Sport mode
    DRIVINFO_GEAR_FAULT    = 0x0F  // Gear fault
} carinfo_drivinfo_gear_t;

/**
 * @brief Enumeration for driving modes
 */
typedef enum {
    DRIVINFO_DRIVEMODE_Comfort   = 0x00, // Comfort mode
    DRIVINFO_DRIVEMODE_ECO       = 0x01, // Economic (Eco) mode
    DRIVINFO_DRIVEMODE_AUTO      = 0x02, // Auto-adjusting mode
    DRIVINFO_DRIVEMODE_Sport     = 0x03, // Sport mode
    DRIVINFO_DRIVEMODE_SNOW      = 0x04, // Snow/icy conditions
    DRIVINFO_DRIVEMODE_OFF_ROAD  = 0x05  // Off-road mode
} carinfo_drivinfo_drivemode_t;

/**
 * @brief Indicator types across dashboard or vehicle system
 */
typedef enum {
    kHighBeam = 0x00,                           // High beam headlight indicator
    kLowBeam,                                   // Low beam headlight indicator
    kPositionLight,                             // Position light indicator
    kDrlLight,                                  // Daytime running light (DRL) indicator

    kFrontFogLight,                             // Front fog light indicator
    kRearFogLight,                              // Rear fog light indicator
    kLeftTurnSignal,                            // Left turn signal indicator
    kRightTurnSignal,                           // Right turn signal indicator

    kHazardSignal,                              // Hazard warning light indicator (both left/right turn lights blinking)
    kEngineFault,                               // Engine fault warning indicator (MIL - Malfunction Indicator Lamp)
    kDrvSeatBeltRst,                            // Driver seat belt warning light (seat belt not fastened)
    kFrontRowSeatBeltRst,                       // Front passenger seat belt warning light

    kDrvSeatBeltRstFlickering,                  // Driver seat belt warning light blinking (typically at 1Hz)
    kAssistantSeatBeltRstFlickering,            // Front passenger seat belt warning light blinking (1Hz)
    kParkingSignal,                             // Mechanical parking brake indicator
    kFuelWarning,                               // Low fuel warning light

    kEngineCoolantTemperature,                  // High engine coolant temperature warning
    kBrakeFluidLow,                             // Brake fluid level low warning
    kCharging,                                  // Battery charging/discharging indicator
    kOilPressure,                               // Engine oil pressure warning

    kOilTemperature,                            // High engine oil temperature warning
    kEpsFault,                                  // Electric power steering (EPS) fault indicator
    kTwoWheelDrive,                             // Two-wheel drive (2WD) mode indicator
    kFourWheelDrive,                            // Four-wheel drive (4WD) mode indicator

    kFrontAxleDifferentialLock,                 // Front axle differential lock indicator
    kRearAxleDifferentialLock,                  // Rear axle differential lock indicator
    kBattSOCUnder,                              // Battery SOC (State of Charge) low warning
    kBattSOCChrg,                               // Battery charging indicator

    kFrntMotTOver,                              // Front motor temperature too high warning
    kNormal,                                    // Driving mode: Normal mode indicator
    kSport,                                     // Driving mode: Sport mode indicator
    kEco,                                       // Driving mode: Economy mode indicator

    kObcCpValVld,                               // On-board charger CP (control pilot) signal valid (charging cable connected)
    kBCUChrgSts,                                // Battery Control Unit (BCU) charging/preheating status indicator
    kMotSysPwrLmtLp,                            // Motor system power limitation warning
    kFrntIpuFltRnk,                             // Front inverter/motor controller fault indicator

    kBCUMILReq,                                 // BCU malfunction indication request (battery fault warning)
    kBCUCellTOver,                              // Battery cell over-temperature warning
    kReadyLp,                                   // Ready-to-drive indicator (vehicle is powered on and ready)
    kMotTempHight,                              // High motor temperature warning (blinking)

    // kCurrentG,                                // Current gear heavy-load mode indicator (commented out)
    kCurrentGFlash,                             // Current gear blinking (gear position display flicker)
    KChrgStsErr,                                // Charging status error (charging fault indicator)

    kIndicatorTypeCount,                        // Total number of indicator types (used for array sizing, bounds checking, etc.)
} IndicatorType;

/**
 * @brief SIF system (status input flags), representing low-level system signals
 */
/**
 * @brief Structure representing SIF (System Information Flags) data from the motor controller.
 * 
 * This structure contains various flags and status indicators related to the motor, controller,
 * and vehicle system. All members are 8-bit unsigned integers (uint8_t), unless otherwise noted.
 * Typically, a value of 1 indicates the status is active or faulted, and 0 means inactive or normal.
 */
typedef struct {
    uint8_t sideStand;                 // Side stand switch status (1 = deployed)
    uint8_t bootGuard;                // Boot guard (safety interlock) active
    uint8_t hallFault;                // Hall sensor fault
    uint8_t throttleFault;            // Throttle input fault
    uint8_t controllerFault;          // General controller fault
    uint8_t lowVoltageProtection;     // Low voltage protection triggered
    uint8_t cruise;                   // Cruise control active
    uint8_t assist;                   // Pedal assist system active
    uint8_t motorFault;               // Motor-related fault
    uint8_t gear;                     // Current gear level (e.g., 1/2/3)
    uint8_t motorRunning;             // Motor is currently running
    uint8_t brake;                    // Brake signal active
    uint8_t controllerProtection;     // Controller over-temperature or overcurrent protection active
    uint8_t coastCharging;            // Regenerative braking (coast charging) active
    uint8_t antiSpeedProtection;      // Overspeed protection triggered
    uint8_t seventyPercentCurrent;    // Current output limited to 70%
    uint8_t pushToTalk;               // Push-to-talk communication signal (if supported)
    uint8_t ekkBackupPower;           // EKK backup power supply active
    uint8_t overCurrentProtection;    // Overcurrent protection triggered
    uint8_t motorShaftLockProtection; // Motor shaft lock protection triggered
    uint8_t reverse;                  // Reverse mode active
    uint8_t electronicBrake;          // Electronic brake engaged
    uint8_t speedLimit;               // Speed limiting function active
    uint8_t current;                  // Current output level (0~255 scale, or interpreted elsewhere)

    uint16_t hallCounter;             // Hall sensor counter (e.g., pulse count)
    
    uint8_t soc;                      // State of Charge (SOC) percentage (0~100)
    uint8_t voltageSystem;            // System voltage level (unit depends on protocol)
} carinfo_sif_t;


/**
 * @brief Meter reading and motor stats
 */
#pragma pack(push, 1)
typedef struct {
    uint16_t current;         // Motor current (0.1 A)
    uint16_t voltage;         // System voltage (0.1 V)
    uint8_t soc;              // State of Charge (0-100%)
    uint16_t speed;           // Displayed speed (0.1 km/h)
    uint16_t speed_real;      // Actual speed (0.1 km/h)
    uint16_t rpm;             // Motor RPM (offset -20000)
    uint8_t voltageSystem;    // Voltage level identifier
    uint8_t gear;             // Current gear position
    uint32_t odo;             // Odometer reading (0.1 km)
} carinfo_meter_t;
#pragma pack(pop)

/**
 * @brief Indicator light status and system faults
 */
#pragma pack(push, 1)
/**
 * @brief Structure representing the status of various vehicle indicator lights and system flags.
 * 
 * Each member is an 8-bit unsigned integer (uint8_t), where typically:
 * - 0 indicates OFF or inactive
 * - 1 indicates ON or active
 * 
 * This structure is compact and suitable for communication or status reporting
 * in embedded or automotive systems.
 */
typedef struct {
    uint8_t highBeam;        // High beam headlights status
    uint8_t lowBeam;         // Low beam headlights status
    uint8_t position;        // Position (parking) light status
    uint8_t frontFog;        // Front fog light status
    uint8_t rearFog;         // Rear fog light status
    uint8_t leftTurn;        // Left turn signal status
    uint8_t rightTurn;       // Right turn signal status
    uint8_t ready;           // Vehicle ready status (e.g., system initialized)
    uint8_t charge;          // Charging status indicator
    uint8_t parking;         // Parking status indicator

    uint8_t ecuFault;        // ECU (Electronic Control Unit) fault status
    uint8_t sensorFault;     // Sensor fault status
    uint8_t motorFault;      // Motor fault status

    uint8_t fuse_fault;      // Fuse fault status
    uint8_t plug_fault;      // Charging plug fault status
    uint8_t battery_fault;   // Battery fault status
    uint8_t brake_fault;     // Brake system fault status
    uint8_t throttle_fault;  // Throttle fault status

    uint8_t bt;              // Bluetooth status (e.g., connected or not)
    uint8_t wifi;            // Wi-Fi status (e.g., connected or not)
} carinfo_indicator_t;


#pragma pack(pop)

/**
 * @brief Drive information including trip and gear mode
 */
#pragma pack(push, 1)
typedef struct {
    uint32_t odo;                          // Total odometer (0.1 km)
    uint16_t tripA;                        // Trip A distance
    uint16_t tripB;                        // Trip B distance
    uint8_t energyType;                    // 0x00: Fuel, 0x01: Electric
    uint16_t enduranceMileage;             // Remaining mileage
    uint16_t insEnergyConsumption;         // Instantaneous energy consumption

    carinfo_drivinfo_form_t odoForm;
    carinfo_drivinfo_form_t tripAForm;
    carinfo_drivinfo_form_t tripBForm;

    carinfo_drivinfo_gear_t gear;
    carinfo_drivinfo_drivemode_t driveMode;
} carinfo_drivinfo_t;
#pragma pack(pop)

/*******************************************************************************
 * FUNCTION DECLARATIONS
 ******************************************************************************/

/**
 * @brief Initialize the car information module (pre-run configuration).
 */
void app_carinfo_init_running(void);

/**
 * @brief Begin tracking and updating car information.
 */
void app_carinfo_start_running(void);

/**
 * @brief Assert car info states (e.g., watchdog or runtime checks).
 */
void app_carinfo_assert_running(void);

/**
 * @brief Runtime update of car information module.
 */
void app_carinfo_running(void);

/**
 * @brief Finalize updates after main run loop (e.g., save logs).
 */
void app_carinfo_post_running(void);

/**
 * @brief Stop the car information module and clear states if needed.
 */
void app_carinfo_stop_running(void);

/**
 * @brief Retrieve current vehicle speed (0.1 km/h).
 * @return uint16_t Speed in 0.1 km/h.
 */
uint16_t app_carinfo_getSpeed(void);

/**
 * @brief Retrieve meter information pointer.
 * @return Pointer to current carinfo_meter_t.
 */
carinfo_meter_t *app_carinfo_get_meter_info(void);

/**
 * @brief Retrieve indicator status structure.
 * @return Pointer to current carinfo_indicator_t.
 */
carinfo_indicator_t *app_carinfo_get_indicator_info(void);

/**
 * @brief Retrieve drive information.
 * @return Pointer to current carinfo_drivinfo_t.
 */
carinfo_drivinfo_t *app_carinfo_get_drivinfo_info(void);


/**
 * @brief Initializes the car indicator module.
 *        This function resets all indicator flags, change flags, and warning flags.
 */
void car_indicator_init(void);

/**
 * @brief Gets the value of the indicator flag array at the specified index.
 * 
 * @param index The index to retrieve (0 to INDICATOR_TYPE_ARRAY_COUNT - 1).
 * @param indicator Pointer to store the value at the specified index.
 * @return uint8_t Returns 1 if the index is valid and the value is retrieved, 0 otherwise.
 */
uint8_t car_indicator_get_indicator(uint8_t index, uint8_t *indicator);

/**
 * @brief Returns the number of bytes used to store all indicator flags.
 * 
 * @return uint8_t The count of indicator flag array elements.
 */
uint8_t data_indicator_get_indicator_type_array_count(void);

/**
 * @brief Gets the status of a specific indicator.
 * 
 * @param type The indicator type to check.
 * @return uint8_t Returns 1 if the indicator is on, 0 if off.
 */
uint8_t car_indicator_get_indicatorflag(IndicatorType type);

/**
 * @brief Sets the status of a specific indicator.
 *        If the status has changed, the change flag will be updated accordingly.
 * 
 * @param type The indicator type to set.
 * @param flag The new status (1 for on, 0 for off).
 */
void car_indicator_set_indicatorflag(IndicatorType type, uint8_t flag);

/**
 * @brief Checks whether any indicator flag has changed since last clear.
 * 
 * @return uint8_t Returns 1 if any change has occurred, 0 otherwise.
 */
uint8_t car_indicator_get_indicatorflag_changed(void);

/**
 * @brief Clears all indicator flag change records.
 */
void car_indicator_clear_indicatorflag_changed(void);

/**
 * @brief Gets the current status of the hazard warning flag.
 * 
 * @return uint8_t Returns 1 if hazard is active, 0 otherwise.
 */
uint8_t car_indicator_get_hazard(void);

/**
 * @brief Sets the hazard warning flag status.
 * 
 * @param flag 1 to enable hazard, 0 to disable.
 */
void car_indicator_set_hazard(uint8_t flag);

/**
 * @brief Gets the warning flag status for a specific indicator.
 * 
 * @param type The indicator type to check.
 * @return uint8_t Returns 1 if warning flag is set, 0 otherwise.
 */
uint8_t car_indicator_get_warnflag(IndicatorType type);

/**
 * @brief Sets or clears the warning flag for a specific indicator.
 *        Setting the flag also starts the warning timer.
 * 
 * @param type The indicator type to set.
 * @param flag 1 to set the warning, 0 to clear.
 */
void car_indicator_set_warnflag(IndicatorType type, uint8_t flag);

/**
 * @brief Checks whether there is an active warning event.
 *        This can be based on current warning flags or timer condition.
 * 
 * @return uint8_t Returns 1 if a warning event is active, 0 otherwise.
 */
uint8_t car_indicator_get_warnflag_event(void);

void car_indicator_proc_turn_signal(void);
void car_meter_proc_speed_rpm(void);
#ifdef __cplusplus
}
#endif

#endif // __OCTOPUS_TASK_MANAGER_CARINFO_H__
