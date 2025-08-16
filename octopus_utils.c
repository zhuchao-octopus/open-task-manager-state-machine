/*******************************************************************************
 * @file    octopus_utils.c
 * @brief   This file contains functions and handlers for the MCU firmware update process,
 *          including communication with the MCU through UART, checking the firmware file,
 *          and handling the update process state machine.
 *          It also includes mechanisms to manage the upgrade cycle, error handling,
 *          and interaction with the application layer.
 *
 * @details This file is part of the Octopus platform, which is designed to handle MCU firmware
 *          updates over a communication interface. The update process is divided into several
 *          states, including initialization, file verification, confirmation, data transfer,
 *          and completion. The communication between the host and the MCU is done via UART.
 *          The firmware update file is in Intel Hex format, and data is transmitted in packets.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team
 *
 * @note    Ensure that the MCU is in a proper state before starting the update process.
 *          Timeouts and retries are implemented to handle communication issues.
 *
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_utils.h"

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

/*******************************************************************************
 * MACROS
 */

// #define MAX_LINE_LEN 1024
// #define MAX_RECORDS  65536
#define CRC32_POLYNOMIAL (0x04C11DB7) // Standard CRC32 polynomial
#define DEFAULT_OUPG_FILENAME_MAX_LENGTH 50
#define DEFAULT_OUPG_BINFILE_READ_MAX_SIZE 48
/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * LOCAL FUNCTIONS DECLARATION
 */
// Declare function to handle the MCU firmware update state
/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * STATIC VARIABLES
 */

/*******************************************************************************
 * EXTERNAL VARIABLES
 */
/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * CRC Calculation
 *******************************************************************************/
uint32_t calculate_crc_32(uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= (uint32_t)data[i] << 24;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ CRC32_POLYNOMIAL;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

uint32_t calculate_crc_32_step(uint32_t current_crc, uint8_t *data, uint32_t length)
{
    uint32_t crc = current_crc;
    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= (uint32_t)data[i] << 24;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ CRC32_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static int hex_char_to_int(char c)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('A' <= c && c <= 'F')
        return c - 'A' + 10;
    if ('a' <= c && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

static int parse_hex_byte(const char *hex, uint8_t *out)
{
    int high = hex_char_to_int(hex[0]);
    int low = hex_char_to_int(hex[1]);
    if (high < 0 || low < 0)
        return -1;
    *out = (uint8_t)((high << 4) | low);
    return 0;
}

// Convert month string to numeric index (1-12)
uint8_t get_month_index(const char *month_str)
{
    static const char *months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (uint8_t i = 0; i < 12; i++)
    {
        if (month_str[0] == months[i][0] &&
            month_str[1] == months[i][1] &&
            month_str[2] == months[i][2])
        {
            return i + 1;
        }
    }
    return 0; // Invalid month
}

// Parse build date (__DATE__) and time (__TIME__)
void parse_build_date_time(uint16_t *year, uint8_t *month, uint8_t *day,
                           uint8_t *hour, uint8_t *minute)
{
    char month_str[4];
    int y, d, h, m, s;

    sscanf(__DATE__, "%3s %d %d", month_str, &d, &y);
    *year = (uint16_t)y;
    *month = get_month_index(month_str);
    *day = (uint8_t)d;

    sscanf(__TIME__, "%d:%d:%d", &h, &m, &s);
    *hour = (uint8_t)h;
    *minute = (uint8_t)m;
}

// Encode version info into 32-bit value
// Encode version info into a 32-bit value
uint32_t encode_datetime_version(uint16_t year, uint8_t month, uint8_t day,
                                 uint8_t hour, uint8_t minute,
                                 uint8_t version_code)
{
    // Ensure the year is within the valid range (2000-2063)
    if (year < 2000)
        year = 2000;
    if (year > 2063)
        year = 2063; // 6 bits: stores 0-63 (2000-2063)

    // Ensure month, day, hour, minute, and version_code are within valid ranges
    month = (month > 12) ? 12 : month;                        // 1-12 months
    day = (day > 31) ? 31 : day;                              // 1-31 days
    hour = (hour > 23) ? 23 : hour;                           // 0-23 hours
    minute = (minute > 59) ? 59 : minute;                     // 0-59 minutes
    version_code = (version_code > 127) ? 127 : version_code; // 1-127 version codes

    uint32_t val = 0;
    val |= ((year - 2000) & 0x3F) << 26; // 6 bits for year (2000-2063)
    val |= (month & 0x0F) << 22;         // 4 bits for month (1-12)
    val |= (day & 0x1F) << 17;           // 5 bits for day (1-31)
    val |= (hour & 0x1F) << 12;          // 5 bits for hour (0-23)
    val |= (minute & 0x1F) << 7;         // 5 bits for minute (0-59)
    val |= (version_code & 0x7F);        // 7 bits for version code (1-127)

    return val;
}

// Decode 32-bit value back to version info
void decode_datetime_version(uint32_t encoded,
                             uint16_t *year, uint8_t *month, uint8_t *day,
                             uint8_t *hour, uint8_t *minute,
                             uint8_t *version_code)
{
    *year = 2000 + ((encoded >> 26) & 0x3F);
    *month = (encoded >> 22) & 0x0F;
    *day = (encoded >> 17) & 0x1F;
    *hour = (encoded >> 12) & 0x1F;
    *minute = (encoded >> 7) & 0x1F;
    *version_code = encoded & 0x7F;
}

// Build encoded version number using current compile time
uint32_t build_version_code(void)
{
    uint16_t y;
    uint8_t m, d, h, min;
    parse_build_date_time(&y, &m, &d, &h, &min);
    return encode_datetime_version(y, m, d, h, min, OTMS_VERSION_CODE);
}

// Format version info as string: YYYYMMDDHHMM_VER
void build_version_string(char *out_str, size_t max_len)
{
    uint16_t y;
    uint8_t m, d, h, min;
    parse_build_date_time(&y, &m, &d, &h, &min);
    snprintf(out_str, max_len, "%04u%02u%02u%02u%02u_%03u", y, m, d, h, min, OTMS_VERSION_CODE);
}

/**
 * @brief Compare two encoded version values.
 *
 * @param v1 First version (encoded uint32_t)
 * @param v2 Second version (encoded uint32_t)
 * @return int
 *        < 0 if v1 < v2 (v2 is newer)
 *        = 0 if v1 == v2
 *        > 0 if v1 > v2 (v1 is newer)
 */
int32_t compare_versions(uint32_t v1, uint32_t v2)
{
    uint16_t y1, y2;
    uint8_t m1, d1, h1, min1, code1;
    uint8_t m2, d2, h2, min2, code2;

    decode_datetime_version(v1, &y1, &m1, &d1, &h1, &min1, &code1);
    decode_datetime_version(v2, &y2, &m2, &d2, &h2, &min2, &code2);

    LOG_LEVEL("Bank Slot A Version: %04d%02d%02d%02d%02d (Code: %d)\n", y1, m1, d1, h1, min1, code1);
    LOG_LEVEL("Bank Slot B Version: %04d%02d%02d%02d%02d (Code: %d)\n", y2, m2, d2, h2, min2, code2);

    return (int32_t)(v1 - v2);
}

// Validate the encoded version information
bool check_encoded_version_valid(uint32_t encoded_version)
{
    uint16_t year, month, day, hour, minute, version_code;

    // Decode fields from the 32-bit version code
    year = 2000 + ((encoded_version >> 26) & 0x3F); // 6 bits for year (2000-2063)
    month = (encoded_version >> 22) & 0x0F;         // 4 bits for month (1-12)
    day = (encoded_version >> 17) & 0x1F;           // 5 bits for day (1-31)
    hour = (encoded_version >> 12) & 0x1F;          // 5 bits for hour (0-23)
    minute = (encoded_version >> 7) & 0x1F;         // 5 bits for minute (0-59)
    version_code = encoded_version & 0x7F;          // 7 bits for version code (1-127)

    // Validate ranges for year, month, day, hour, minute, and version code
    if (year < 2000 || year > 2063)
        return false; // Year should be between 2000 and 2063
    if (month == 0 || month > 12)
        return false; // Month should be between 1 and 12
    if (day == 0 || day > 31)
        return false; // Day should be between 1 and 31
    if (hour > 23)
        return false; // Hour should be between 0 and 23
    if (minute > 59)
        return false; // Minute should be between 0 and 59
    if (version_code == 0 || version_code > 127)
        return false; // Version code should be between 1 and 127

    return true;
}

/**
 * Extracts and encodes version information from a firmware file name.
 * Expected file name format example: "mcu_202506141612_100.oupg"
 *
 * - Prefix can be anything (e.g., "mcu_", "ble_", etc.)
 * - DateTime format: yyyymmddHHMM (12 digits)
 * - Version code: 3-digit number after an underscore
 *
 * @param filepath Full file path or just file name.
 * @return Encoded 32-bit version value, or 0 on failure.
 */
uint32_t decode_version_from_filename(const char *filepath)
{
    if (!filepath)
        return 0;

    // Step 1: Extract the file name from path (remove directory prefix)
    const char *filename = strrchr(filepath, '/');
    if (!filename)
        filename = strrchr(filepath, '\\');        // For Windows-style paths
    filename = filename ? filename + 1 : filepath; // If no separator, use full string

    // Step 2: Find the last underscore, which precedes the version code
    const char *underscore = strrchr(filename, '_');
    if (!underscore || strlen(underscore) < 4)
        return 0;

    // Step 3: Extract 12-digit datetime string (must be at least 16 characters long)
    // Format expected: xxxx_YYYYMMDDHHMM_VVV.ext
    if (strlen(filename) < 16)
        return 0;

    char datetime_str[13] = {0};             // 12 chars + null terminator
    strncpy(datetime_str, filename + 4, 12); // Skip prefix (e.g., "mcu_")

    // Step 4: Validate that datetime string is all digits
    for (int i = 0; i < 12; ++i)
    {
        if (!isdigit((unsigned char)datetime_str[i]))
            return 0;
    }

    // Step 5: Parse datetime fields from the string
    uint16_t year = atoi(datetime_str);       // YYYY
    uint8_t month = atoi(datetime_str + 4);   // MM
    uint8_t day = atoi(datetime_str + 6);     // DD
    uint8_t hour = atoi(datetime_str + 8);    // HH
    uint8_t minute = atoi(datetime_str + 10); // MM

    // Step 6: Extract 3-digit version code after the underscore
    char version_str[4] = {0};
    strncpy(version_str, underscore + 1, 3);

    // Step 7: Validate version code is numeric
    for (int i = 0; i < 3; ++i)
    {
        if (!isdigit((unsigned char)version_str[i]))
            return 0;
    }

    uint8_t version_code = atoi(version_str); // VVV

    // Step 8: Encode all components into a compact 32-bit value
    return encode_datetime_version(year, month, day, hour, minute, version_code);
}

// Helper: Validate HEX file line CRC (Standard Intel Hex CRC)
bool validate_hex_crc(const char *line)
{
    if (!line || line[0] != ':')
        return false;

    // 拷贝并清除结尾换行符
    char clean_line[128];
    snprintf(clean_line, sizeof(clean_line), "%s", line);
    size_t len = strlen(clean_line);
    while (len > 0 && (clean_line[len - 1] == '\n' || clean_line[len - 1] == '\r'))
        clean_line[--len] = '\0';

    if (len < 11 || (len % 2) != 1) // 必须是冒号 + 偶数个 hex 字节 + checksum 共奇数个字符
        return false;

    uint8_t sum = 0;

    // 计算除最后一个字节（checksum）外的和
    for (size_t i = 1; i < len - 2; i += 2)
    {
        char byte_str[3] = {clean_line[i], clean_line[i + 1], '\0'};

        if (!isxdigit(byte_str[0]) || !isxdigit(byte_str[1]))
            return false; // 非法十六进制字符

        uint8_t byte = (uint8_t)strtoul(byte_str, NULL, 16);
        sum += byte;
    }

    // 提取 checksum
    char checksum_str[3] = {clean_line[len - 2], clean_line[len - 1], '\0'};
    if (!isxdigit(checksum_str[0]) || !isxdigit(checksum_str[1]))
        return false;

    uint8_t checksum = (uint8_t)strtoul(checksum_str, NULL, 16);

    // Intel HEX checksum: 所有字节加总 + checksum 应该为 0x00
    return ((sum + checksum) & 0xFF) == 0x00;
}

int parse_hex_line(const char *line, hex_record_t *record_out)
{
    if (!line || line[0] != ':')
        return -1;

    uint8_t len, addr_hi, addr_lo, type;
    if (parse_hex_byte(&line[1], &len) < 0 ||
        parse_hex_byte(&line[3], &addr_hi) < 0 ||
        parse_hex_byte(&line[5], &addr_lo) < 0 ||
        parse_hex_byte(&line[7], &type) < 0)
        return -2;

    uint16_t offset = ((uint16_t)addr_hi << 8) | addr_lo;

    if (type == 0x00)
    {
        if (record_out)
        {
            record_out->address = offset;
            // LOG_LEVEL("%08x = %08x + %08x\r\n",record_out->address,record_out->bank_address,offset);
            record_out->length = len;
            for (int i = 0; i < len; ++i)
            {
                if (parse_hex_byte(&line[9 + i * 2], &record_out->data[i]) < 0)
                    return -3;
            }
        }
        return 0;
    }
    else if (type == 0x04)
    {
        uint8_t high, low;
        if (parse_hex_byte(&line[9], &high) < 0 || parse_hex_byte(&line[11], &low) < 0)
            return -4;
        // record_out->bank_address = (((uint32_t)high << 8) | low) << 16;
        LOG_LEVEL("record_out->bank_address=%08x\r\n", (((uint32_t)high << 8) | low) << 16);
        return 4;
    }
    else if (type == 0x05)
    {
        return 5;
    }
    else if (type == 0x01)
    {
        return 1;
    }

    return -5;
}

// Check if file is HEX
file_info_t is_valid_hex_file(const char *path_filename)
{
    file_info_t info = {
        .file_type = FILE_TYPE_UNKNOWN,
        .reset_handler = 0,
        .file_size = 0,
        .file_crc_32 = 0xFFFFFFFF};

    FILE *f = fopen(path_filename, "r");
    if (!f)
    {
        LOG_LEVEL("Can not open file %s\n", path_filename);
        return info;
    }
    char line[256];
    while (fgets(line, sizeof(line), f))
    {
        if (line[0] != ':')
        {
            // 非法开头，可能是 BIN 文件
            LOG_LEVEL("Line doesn't start with ':' -> Not a HEX file\n");
            fclose(f);
            return info;
        }

        if (!validate_hex_crc(line))
        {
            LOG_LEVEL("validate hex crc failed %s\n", line);
            fclose(f);
            return info;
        }

        char type_str[3] = {line[7], line[8], '\0'};
        uint8_t record_type = (uint8_t)strtol(type_str, NULL, 16);

        if (record_type == 0x00)
        {
            char len_str[3] = {line[1], line[2], '\0'};
            uint8_t byte_count = (uint8_t)strtol(len_str, NULL, 16);
            info.file_size += byte_count;

            uint8_t data_bytes[256];
            for (uint8_t i = 0; i < byte_count; ++i)
            {
                char byte_str[3] = {line[9 + i * 2], line[10 + i * 2], '\0'};
                data_bytes[i] = (uint8_t)strtol(byte_str, NULL, 16);
            }
            info.file_crc_32 = calculate_crc_32_step(info.file_crc_32, data_bytes, byte_count);
        }
        else if (record_type == 0x04)
        {
            char addr_str[5] = {0};
            strncpy(addr_str, &line[9], 4);
            info.reset_handler = (uint32_t)strtol(addr_str, NULL, 16) << 16;
        }
    }

    fclose(f);
    info.file_crc_32 ^= 0xFFFFFFFF;
    info.file_type = FILE_TYPE_HEX;
    info.file_version = decode_version_from_filename(path_filename);
    return info;
}

file_read_status_t read_next_hex_record(FILE *hex_file, long *file_offset, hex_record_t *hex_record)
{
    char line[256];
    uint8_t ret = 0;

    if (!hex_file)
        return FILE_READ_ERROR;

    if (fseek(hex_file, *file_offset, SEEK_SET) != 0)
        return FILE_READ_ERROR;

    if (fgets(line, sizeof(line), hex_file))
    {
        *file_offset = ftell(hex_file);

        if (line[0] != ':' || !validate_hex_crc(line))
        {
            return FILE_READ_ERROR;
        }

        ret = parse_hex_line(line, hex_record);
        if (ret == 0)
        {
            return FILE_READ_DATA_OK;
        }
        else if (ret == 1)
        {
            return FILE_READ_EOF;
        }
        else if (ret == 4)
        {
            return FILE_READ_CT_INFOR;
        }
        else if (ret == 5)
        {
            return FILE_READ_INVALID;
        }
        else
        {
            return FILE_READ_ERROR;
        }
    }

    return FILE_READ_EOF;
}

// Check if file is valid BIN with vector table
file_info_t is_valid_bin_file(uint32_t model_number, uint32_t target_bank_offset, const char *path_filename)
{
    file_info_t info = {
        .file_type = FILE_TYPE_UNKNOWN,
        .reset_handler = 0,
        .file_size = 0,
        .file_crc_32 = 0xFFFFFFFF};

    FILE *f = fopen(path_filename, "rb");
    if (!f)
        return info;

    // Step 1: Get total file size
    fseek(f, 0, SEEK_END);
    uint32_t file_size = ftell(f);

    // Ensure the file is large enough to contain at least header + meta
    if (file_size < (long)(target_bank_offset + 8 + sizeof(meta_info_t)))
    {
        fclose(f);
        return info;
    }

    // Step 2: Try reading meta info from the tail
    meta_info_t meta;
    int has_valid_meta = 0;
    fseek(f, file_size - sizeof(meta_info_t), SEEK_SET);
    if (fread(&meta, 1, sizeof(meta_info_t), f) == sizeof(meta_info_t))
    {
        if (meta.bank1.magic == APP_MATA_INFO_MAGIC && meta.bank2.magic == APP_MATA_INFO_MAGIC)
        {
            has_valid_meta = 1;
        }
        else
        {
            fclose(f);
            return info;
        }

        LOG_LEVEL("bank1.model: %08x bank2.model: %08x model_number:%08x\n ", meta.bank1.model, meta.bank2.model, model_number);

        if (meta.bank1.model != model_number || meta.bank2.model != model_number)
        {
            fclose(f);
            return info;
        }
    }

    // Step 3: Read first 8 bytes from target offset (SP + Reset Vector)
    if (fseek(f, target_bank_offset, SEEK_SET) != 0)
    {
        fclose(f);
        return info;
    }

    uint8_t header_buf[8];
    if (fread(header_buf, 1, 8, f) != 8)
    {
        fclose(f);
        return info;
    }

    uint32_t sp = header_buf[0] | (header_buf[1] << 8) | (header_buf[2] << 16) | (header_buf[3] << 24);
    uint32_t reset_vector = header_buf[4] | (header_buf[5] << 8) | (header_buf[6] << 16) | (header_buf[7] << 24);

    // Step 4: Validate stack pointer and reset handler address range
    if (sp < 0x20000000 || sp > 0x40000000 ||
        reset_vector < 0x08000000 || reset_vector > 0x20000000)
    {
        fclose(f);
        return info;
    }

    // Step 5: Determine valid size
    uint32_t valid_size;
    if (has_valid_meta)
    {
        // valid_size = (target_bank_offset == meta.app1.start_address) ? meta.app1.size : meta.app2.size;
        if (target_bank_offset == meta.bank1.start_address)
            valid_size = meta.bank1.size;
        else if (target_bank_offset == meta.bank2.start_address)
            valid_size = meta.bank2.size;
        else
        {
            fclose(f);
            return info;
        }
    }
    else
    {
        valid_size = (uint32_t)(file_size - target_bank_offset); // fallback: full to EOF
    }

    // Step 6: Calculate CRC from target offset to end of valid range
    if (fseek(f, target_bank_offset, SEEK_SET) != 0)
    {
        fclose(f);
        return info;
    }

    uint32_t remain = valid_size;
    uint8_t read_buf[256];
    while (remain > 0)
    {
        size_t chunk = (remain > sizeof(read_buf)) ? sizeof(read_buf) : remain;
        if (fread(read_buf, 1, chunk, f) != chunk)
        {
            fclose(f);
            return info;
        }
        info.file_crc_32 = calculate_crc_32_step(info.file_crc_32, read_buf, chunk);
        remain -= chunk;
    }

    fclose(f);

    // Step 7: Fill final info
    info.file_crc_32 ^= 0xFFFFFFFF;
    info.reset_handler = reset_vector;
    info.file_type = FILE_TYPE_BIN;
    info.file_size = valid_size;
    info.file_version = decode_version_from_filename(path_filename);
    LOG_LEVEL("bank1.crc: %08x bank2.crc: %08x re-crc:%08x\r\n ", meta.bank1.crc32, meta.bank2.crc32, info.file_crc_32);
    return info;
}

file_read_status_t read_next_bin_record(FILE *bin_file, long *file_offset, hex_record_t *hex_record)
{
    uint32_t file_total_size = 0;
    static uint8_t meta_valid = 0;
    static meta_info_t meta;

    if (!bin_file || !file_offset || !hex_record)
        return FILE_READ_ERROR;

    // Step 1: On first call, read META
    if (hex_record->f_count == 0)
    {
        // Get total file size
        if (fseek(bin_file, 0, SEEK_END) != 0)
            return FILE_READ_ERROR;

        file_total_size = ftell(bin_file);

        if (file_total_size >= sizeof(meta_info_t))
        {
            if (fseek(bin_file, file_total_size - sizeof(meta_info_t), SEEK_SET) == 0 &&
                fread(&meta, 1, sizeof(meta_info_t), bin_file) == sizeof(meta_info_t) &&
                meta.bank1.magic == APP_MATA_INFO_MAGIC &&
                meta.bank2.magic == APP_MATA_INFO_MAGIC)
            {
                meta_valid = 1;
            }
        }
    }

    // Step 2: Determine which app region current offset is in
    // long region_start = 0;
    uint32_t region_end = file_total_size;

    if (meta_valid)
    {
        if (*file_offset >= (long)meta.bank2.start_address)
        {
            // Reading bank2
            // region_start = meta.bank2.start_address;
            region_end = meta.bank2.start_address + meta.bank2.size;
        }
        else
        {
            // Reading bank1
            // region_start = meta.bank1.start_address;
            region_end = meta.bank1.start_address + meta.bank1.size;
        }
    }

    // Step 3: Check if we've reached the end of this app region
    if (*file_offset >= region_end)
        return FILE_READ_EOF;

    // Step 4: Seek and read up to DEFAULT_READ_BIN_MAX_SIZE bytes
    if (fseek(bin_file, *file_offset, SEEK_SET) != 0)
        return FILE_READ_ERROR;

    size_t remain = (size_t)(region_end - *file_offset);
    size_t to_read = remain > DEFAULT_OUPG_BINFILE_READ_MAX_SIZE ? DEFAULT_OUPG_BINFILE_READ_MAX_SIZE : remain;

    size_t bytes_read = fread(hex_record->data, 1, to_read, bin_file);
    if (bytes_read > 0)
    {
        hex_record->address = (uint32_t)(*file_offset);
        hex_record->length = (uint8_t)bytes_read;
        *file_offset += bytes_read;
        return FILE_READ_DATA_OK;
    }

    return feof(bin_file) ? FILE_READ_EOF : FILE_READ_ERROR;
}

// Main parsing function
file_info_t parse_firmware_file(uint32_t model_number, uint32_t target_bank_offset, const char *filename)
{
    file_info_t info = {.file_type = FILE_TYPE_UNKNOWN};
    LOG_LEVEL("parse firmware file:%s\r\n", filename);
    LOG_LEVEL("parse firmware file target_address:%08x\r\n", target_bank_offset);

    info = is_valid_hex_file(filename);
    if (info.file_type == FILE_TYPE_HEX)
        return info;

    info = is_valid_bin_file(model_number, target_bank_offset, filename);
    if (info.file_type == FILE_TYPE_BIN)
        return info;

    return info;
}

int copy_file_to_tmp(const char *src_path, const char *filename, char *dst_path, size_t dst_size)
{
    snprintf(dst_path, dst_size, "/tmp/%s", filename);
#ifdef PLATFORM_LINUX_RISC
    if (access(dst_path, F_OK) == 0)
    {
        return 1; // 表示已存在，未复制
    }

    FILE *src = fopen(src_path, "rb");
    if (!src)
        return -1;

    FILE *dst = fopen(dst_path, "wb");
    if (!dst)
    {
        fclose(src);
        return -1;
    }

    char buffer[1024];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), src)) > 0)
    {
        fwrite(buffer, 1, bytes, dst);
    }

    fclose(src);
    fclose(dst);
#endif
    return 0;
}

int search_and_copy_oupg_files(const char *dir_path, char *out_path, size_t out_path_size)
{
#ifdef PLATFORM_LINUX_RISC
    DIR *dir = opendir(dir_path);
    if (!dir)
    {
        LOG_LEVEL("dir not exitst:%s\r\n", dir_path);
        return 0;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type != DT_REG)
        {
            continue;
        }

        // if (strlen(entry->d_name) > out_path_size -10)
        //{
        //     continue;
        // }

        if (fnmatch("*.oupg", entry->d_name, 0) == 0)
        {
            char full_path[64];
            snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, entry->d_name);
            if (copy_file_to_tmp(full_path, entry->d_name, out_path, out_path_size) >= 0)
            {
                closedir(dir);
                return 1; // Found and copied
            }
        }
    }

    closedir(dir);
#endif
    return 0; // Not found
}

int file_exists(const char *file_path_name)
{
#ifdef PLATFORM_LINUX_RISC
    return access(file_path_name, F_OK) == 0;
#else
    return 0;
#endif
}
