/*******************************************************************************
 * @file     octopus_log.c
 * @brief    Implements UART logging functions to provide logging capabilities
 *           with various log levels (DEBUG, INFO, WARNING, ERROR). Supports
 *           formatted log messages and buffer outputs.
 *
 * This file contains functions that support logging in the system via UART.
 * It allows logging of messages with variable arguments, formatted output,
 * and buffer printing. The log levels are configurable to control the verbosity
 * of logs, making it useful for debugging and tracking system status.
 *
 * The `dbg_log_printf` functions allow formatted messages to be printed to
 * UART, while the `dbg_log_printf_level` function includes the log level and
 * function name in the log output.
 *
 * The logging level can be dynamically set using `dbg_log_set_level`, and
 * specific buffer data can be logged using `dbg_log_printf_buffer` or
 * `dbg_log_printf_buffer_level`.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team
 *******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_log.h"      // Include logging functions for debugging

#define OTSM_DEBUG_USART1 

#ifdef OTSM_RELEASE_VERSION
DBG_LOG_LEVEL current_log_level = LOG_LEVEL_NONE;
#else
DBG_LOG_LEVEL current_log_level = LOG_LEVEL_DEBUG;
#endif

#define LOG_DEFAULT_MAX_WIDTH 28
#define USE_MY_PRINTF

#ifdef USE_MY_PRINTF

#define FMT_FLAG_LEFT     (1 << 0)  // Left justify (for '-')
#define FMT_FLAG_PLUS     (1 << 1)  // Show positive sign '+' (for signed numbers)
#define FMT_FLAG_SPACE    (1 << 2)  // Show space for positive numbers (for signed numbers)
#define FMT_FLAG_ZERO     (1 << 3)  // Pad with zeros (for numbers)
#define FMT_FLAG_SPECIAL  (1 << 4)  // Prefix for octal (0) or hexadecimal (0x)
#define FMT_FLAG_SIGN     (1 << 5)  // Flag for signed numbers (e.g. positive or negative integers)
#define FMT_FLAG_LARGE    (1 << 6)  // Use uppercase letters for hexadecimal (e.g., 'A'-'F')

static const char *digits = "0123456789abcdefghijklmnopqrstuvwxyz";
static const char *upper_digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

// Check if a character is a digit
static inline int is_digital(char c) {
    return c >= '0' && c <= '9';
}

// Calculate string length with max limit
static size_t strnlen__(const char *s, size_t count) {
    const char *sc = s;
    while (*sc != '\0' && count--) ++sc;
    return sc - s;
}

// Convert string to int (used for width/precision parsing)
static int skip_atoi__(const char **s) {
    int i = 0;
    while (is_digital(**s))
        i = i * 10 + *((*s)++) - '0';
    return i;
}

// Print number in specified base with formatting
static void number__(std_putc putc, long num, int base, int size, int precision, int type)
{
    char sign, tmp[66];
    const char *dig = digits;
    int i;
    char tmpch;

    if (type & FMT_FLAG_LARGE)
        dig = upper_digits;
    if (type & FMT_FLAG_LEFT)
        type &= ~FMT_FLAG_ZERO;
    if (base < 2 || base > 36)
        return;

    //c = (type & FMT_FLAG_ZERO) ? '0' : ' ';
    sign = 0;
    if (type & FMT_FLAG_SIGN)
    {
        if (num < 0)
        {
            sign = '-';
            num = -num;
            size--;
        }
        else if (type & FMT_FLAG_PLUS)
        {
            sign = '+';
            size--;
        }
        else if (type & FMT_FLAG_SPACE)
        {
            sign = ' ';
            size--;
        }
    }
    if (type & FMT_FLAG_SPECIAL)
    {
        if (base == 16)
            size -= 2;
        else if (base == 8)
            size--;
    }

    i = 0;
    if (num == 0)
        tmp[i++] = '0';
    else
    {
        while (num != 0)
        {
            tmp[i++] = dig[((unsigned long)num) % (unsigned)base];
            num = ((unsigned long)num) / (unsigned)base;
        }
    }

    // Handle precision for zero-padded hexadecimal numbers
    if (base == 16 && precision < 2)
        precision = 2; // Ensure at least two digits for hexadecimal (e.g., "0x01" instead of "0x1")

    if (i > precision)
        precision = i;

    size -= precision;
    if (!(type & (FMT_FLAG_ZERO | FMT_FLAG_LEFT)))
    {
        while (size-- > 0)
        {
            tmpch = ' ';
            putc(&tmpch, 1);
        }
    }

    if (sign)
    {
        putc(&sign, 1);
    }

    if (type & FMT_FLAG_SPECIAL)
    {
        if (base == 8)
        {
            tmpch = '0';
            putc(&tmpch, 1);
        }
        else if (base == 16)
        {
            tmpch = '0';
            putc(&tmpch, 1);
            tmpch = 'x'; // '0x' prefix for hexadecimal numbers
            putc(&tmpch, 1);
        }
    }

    if (!(type & FMT_FLAG_LEFT))
    {
        while (size-- > 0)
        {
            tmpch = ' ';
            putc(&tmpch, 1);
        }
    }

    while (i < precision--)
    {
        tmpch = '0';  // Fill with zeroes if the number is shorter than precision
        putc(&tmpch, 1);
    }

    while (i-- > 0)
    {
        tmpch = tmp[i];
        putc(&tmpch, 1);
    }

    while (size-- > 0)
    {
        tmpch = ' ';
        putc(&tmpch, 1);
    }
}

static void vsprintf__(std_putc putc, const char *fmt, va_list args)
{
    int len;
    unsigned long num;
    int base;
    char *s;
    int flags = 0;       // Flags for number formatting
    int field_width = -1; // Width of output field
    int precision = -1;   // Min. # of digits for integers; max for strings
    int qualifier = -1;   // 'h', 'l', or 'L' for integer fields
    char *tmpstr = NULL;
    int tmpstr_size = 0;
    char tmpch;

    // Iterate over the format string
    for (; *fmt; fmt++)
    {
        if (*fmt != '%')
        {
            // Process non-format characters
            if (tmpstr == NULL)
            {
                tmpstr = (char *)fmt;
                tmpstr_size = 0;
            }
            tmpstr_size++;
            continue;
        }
        else if (tmpstr_size)
        {
            putc(tmpstr, tmpstr_size);
            tmpstr = NULL;
            tmpstr_size = 0;
        }

        // Parse flags like '-', '+', ' ', etc.
        flags = 0;
    repeat:
        fmt++; // Skip the first '%'
        switch (*fmt)
        {
        case '-':
            flags |= FMT_FLAG_LEFT;
            goto repeat;
        case '+':
            flags |= FMT_FLAG_PLUS;
            goto repeat;
        case ' ':
            flags |= FMT_FLAG_SPACE;
            goto repeat;
        case '#':
            flags |= FMT_FLAG_SPECIAL;
            goto repeat;
        case '0':
            flags |= FMT_FLAG_ZERO;
            goto repeat;
        }

        // Get field width (e.g., for padding)
        field_width = -1;
        if (is_digital(*fmt))
            field_width = skip_atoi__(&fmt);
        else if (*fmt == '*')
        {
            fmt++;
            field_width = va_arg(args, int);
            if (field_width < 0)
            {
                field_width = -field_width;
                flags |= FMT_FLAG_LEFT;
            }
        }

        // Get precision for integers or strings
        precision = -1;
        if (*fmt == '.')
        {
            ++fmt;
            if (is_digital(*fmt))
                precision = skip_atoi__(&fmt);
            else if (*fmt == '*')
            {
                ++fmt;
                precision = va_arg(args, int);
            }
            if (precision < 0)
                precision = 0;
        }

        // Get the conversion qualifier (e.g., 'h', 'l')
        qualifier = -1;
        if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L')
        {
            qualifier = *fmt;
            fmt++;
        }

        // Default base (decimal)
        base = 10;
        switch (*fmt)
        {
        case 'c':
            // Character
            if (!(flags & FMT_FLAG_LEFT))
            {
                while (--field_width > 0)
                {
                    tmpch = ' ';
                    putc(&tmpch, 1);
                }
            }
            tmpch = (unsigned char)va_arg(args, int);
            putc(&tmpch, 1);
            while (--field_width > 0)
            {
                tmpch = ' ';
                putc(&tmpch, 1);
            }
            continue;
        case 's':
            // String
            s = va_arg(args, char *);
            if (!s)
                s = "<NULL>";
            len = strnlen__(s, precision);
            if (!(flags & FMT_FLAG_LEFT))
            {
                while (len < field_width--)
                {
                    tmpch = ' ';
                    putc(&tmpch, 1);
                }
            }
            putc(s, len);
            while (len < field_width--)
            {
                tmpch = ' ';
                putc(&tmpch, 1);
            }
            continue;
        case 'p':
            // Pointer (address)
            if (field_width == -1)
            {
                field_width = 2 * sizeof(void *);
                flags |= FMT_FLAG_ZERO;
            }
            number__(putc, (unsigned long)va_arg(args, void *), 16, field_width, precision, flags);
            continue;
        case 'o':
            // Octal
            base = 8;
            break;
        case 'X':
            // Uppercase hexadecimal
            flags |= FMT_FLAG_LARGE;
        case 'x':
            // Hexadecimal
            base = 16;
            break;
        case 'd':
        case 'i':
            // Signed decimal
            flags |= FMT_FLAG_SIGN;
        case 'u':
            // Unsigned decimal
            break;
        default:
            if (*fmt != '%')
            {
                tmpch = '%';
                putc(&tmpch, 1);
            }
            if (*fmt)
            {
                tmpch = *fmt;
                putc(&tmpch, 1);
            }
            else
            {
                --fmt;
            }
            continue;
        }

        // Determine the number type (signed or unsigned)
        if (qualifier == 'l')
            num = va_arg(args, unsigned long);
        else if (qualifier == 'h')
        {
            if (flags & FMT_FLAG_SIGN)
                num = va_arg(args, int);
            else
                num = va_arg(args, unsigned int);
        }
        else if (flags & FMT_FLAG_SIGN)
            num = va_arg(args, int);
        else
            num = va_arg(args, unsigned int);

        // Output the formatted number
        number__(putc, num, base, field_width, precision, flags);
    }

    // Output any remaining characters
    if (tmpstr_size)
    {
        putc(tmpstr, tmpstr_size);
    }
}

static void native_uart_putc(char *data, uint16_t size)
{
	#ifdef OTSM_DEBUG_USART2
	UART2_Send_Buffer((uint8_t *)data, size);
	#else
	UART1_Send_Buffer((uint8_t *)data, size);
	#endif
}

#endif

/**
 * @brief Print a formatted log message.
 * @param format The format string.
 * @param ... The variable arguments.
 */
void dbg_log_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format); // Initialize the va_list to process the variable arguments
#ifdef USE_MY_PRINTF
    vsprintf__(native_uart_putc, format, args); // Call formatted print function
#else
    vprintf(format, args);
#endif
    va_end(args); // Clean up the va_list after use
}

/**
 * @brief Print a log message with level, function name, file, and line number.
 * @param function_name The name of the function calling the log.
 * @param format The format string.
 * @param ... The variable arguments.
 */
void dbg_log_printf_level(const char *function_name, const char *format, ...)
{
    // Skip logging if log level is set to NONE or format is NULL
    if (current_log_level == LOG_LEVEL_NONE || !format)
    {
        return;
    }

    // Map log level to string representation
    const char *level_str;
    switch (current_log_level)
    {
    case LOG_LEVEL_DEBUG:
        level_str = "DEBUG";
        break;
    case LOG_LEVEL_INFO:
        level_str = "INFO";
        break;
    case LOG_LEVEL_WARNING:
        level_str = "WARNING";
        break;
    case LOG_LEVEL_ERROR:
        level_str = "ERROR";
        break;
    default:
        level_str = "";
        break;
    }

// Print log header with timestamp, level, function name
#ifdef USE_MY_PRINTF
    dbg_log_printf("[%s][%28s] ", level_str, function_name);
#else
    printf("[%s][%28s] ", level_str, function_name);
#endif

    if (format == NULL || format[0] == '\0')
    {
        return;
    }

    va_list args;
    va_start(args, format);
// Print formatted log message
#ifdef USE_MY_PRINTF
    vsprintf__(native_uart_putc, format, args);
#else
    vprintf(format, args);
#endif
    va_end(args);
}

/**
 * @brief Print a buffer of bytes in hexadecimal format.
 * @param buff The buffer to print.
 * @param length The length of the buffer.
 */
void dbg_log_printf_buffer(uint8_t *buff, uint16_t length)
{
    for (int i = 0; i < length; i++)
    {
#ifdef USE_MY_PRINTF
        dbg_log_printf("%02x ", buff[i]);
#else
        printf("%02x ", buff[i]);
#endif
    }
#ifdef USE_MY_PRINTF
    dbg_log_printf("\r\n");
#else
    printf("\r\n");
#endif
}

/**
 * @brief Print a buffer of bytes in hexadecimal format with log level and function name.
 * @param function_name The name of the function calling the log.
 * @param buff The buffer to print.
 * @param length The length of the buffer.
 */
void dbg_log_printf_buffer_level(const char *function_name, const uint8_t *buff, uint16_t length)
{
    if (buff == NULL || length == 0)
    {
        return;
    }
    dbg_log_printf_level(function_name, "");
    for (int i = 0; i < length; i++)
    {
#ifdef USE_MY_PRINTF
        dbg_log_printf("%02x ", buff[i]);
#else
        printf("%02x ", buff[i]);
#endif
    }
#ifdef USE_MY_PRINTF
    dbg_log_printf("\r\n");
#else
    printf("\r\n");
#endif
}

/**
 * @brief Set the current log level.
 * @param level The log level to set (DEBUG, INFO, WARNING, ERROR, or NONE).
 */
void dbg_log_set_level(DBG_LOG_LEVEL level)
{
    current_log_level = level; // Update the log level
}
