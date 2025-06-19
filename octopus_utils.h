/*******************************************************************************
 * @file    octopus_utils.h
 * @brief   Header file for the Octopus Task Manager Update module.
 *          This file declares the functions and types required for managing
 *          MCU firmware updates.
 *
 * @details This module provides functionality for initializing, starting,
 *          and monitoring the MCU firmware update process. It defines the
 *          update states and provides interfaces to query the update status
 *          and progress.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team

 ******************************************************************************/

#ifndef ___OCTOPUS_TASK_MANAGER_UTILS_H___
#define ___OCTOPUS_TASK_MANAGER_UTILS_H___

/*******************************************************************************
 * INCLUDE FILES
 * Include standard libraries and platform-specific headers.
 ******************************************************************************/
#include "octopus_platform.h" // Platform-specific definitions
#include "octopus_system.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @defgroup APP_SETTING APP:SETTING
	 * @brief    Application settings and configurations.
	 * @{
	 */

	/*******************************************************************************
	 * DEBUG SWITCH MACROS
	 * Define macros to enable or disable debug functionality.
	 ******************************************************************************/

	/*******************************************************************************
	 * MACROS
	 * Define commonly used macros for this module.
	 ******************************************************************************/

	/*******************************************************************************
	 * TYPEDEFS
	 * Define types used in the MCU update process.
	 ******************************************************************************/
	typedef enum
	{
		FILE_READ_OK = 0,
		FILE_READ_EOF = 1,
		FILE_READ_INVALID = -1,
		FILE_READ_ERROR = -2
	} file_read_status_t;

	typedef struct
	{
		//uint32_t bank_address;
		uint32_t address;
		uint8_t data[64];
		uint8_t length;
	} hex_record_t;

	typedef enum
	{
		FILE_TYPE_UNKNOWN = 0, // Unknown file type
		FILE_TYPE_HEX,		   // Intel HEX file format
		FILE_TYPE_BIN		   // Raw binary file format
	} file_type_t;

	typedef struct
	{
		file_type_t file_type;
		uint32_t file_size;
		uint32_t file_version;
		uint32_t crc_32;
		uint32_t reset_handler;
	} file_info_t;

	/*******************************************************************************
	 * CONSTANTS
	 * Define any module-specific constants.
	 ******************************************************************************/

	/*******************************************************************************
	 * GLOBAL VARIABLES DECLARATION
	 * Declare external variables used across the module.
	 ******************************************************************************/

	/*******************************************************************************
	 * GLOBAL FUNCTIONS DECLARATION
	 * Declare the public functions provided by this module.
	 ******************************************************************************/
	uint32_t calculate_crc_32(uint8_t *data, uint32_t length);
	uint32_t calculate_crc_32_step(uint32_t current_crc, uint8_t *data, uint32_t length);

	void decode_datetime_version(uint32_t encoded,
								 uint16_t *year, uint8_t *month, uint8_t *day,
								 uint8_t *hour, uint8_t *minute,
								 uint8_t *version_code);
	void build_version_string(char *out_str, size_t max_len);
	uint32_t build_version_code(void);
	int32_t compare_versions(uint32_t v1, uint32_t v2);
	bool is_version_code_valid(uint32_t version_code);

	file_info_t parse_firmware_file(const char *filename);
	file_read_status_t read_next_hex_record(FILE *hex_file, long *file_pos, hex_record_t *hex_record);
	int search_and_copy_oupg_files(const char* dir_path, char* out_path, size_t out_path_size);
	int file_exists(const char *file_path_name);
	/**
	 * @brief Get the error code from the update process.
	 * @return The error code indicating the reason for failure.
	 */

#ifdef __cplusplus
}
#endif

#endif // ___OCTOPUS_TASK_MANAGER_UPDATE_H___
