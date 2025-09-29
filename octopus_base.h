/***********************************************************************************
 * @file    octopus_base.h
 * @brief   Base include file for the Octopus project.
 *          This file provides the fundamental standard library includes and
 *          common definitions required across the entire project.
 *
 * @version 1.0
 * @date    2025-09-01
 * @author  Macky
 *
 * @note    This file should be included at the very beginning of any project
 *          module to ensure consistent inclusion of base types, macros, and
 *          standard headers.
 *
 ***********************************************************************************/

#ifndef __OCTOPUS_BASE_H__
#define __OCTOPUS_BASE_H__

///////////////////////////////////////////////////////////////////////////////////
#define OTMS_VERSION_CODE (001)
#define OTMS_VERSION_NAME "0.0.1"

/***********************************************************************************
 * Standard Library Includes
 * These headers provide fundamental definitions, data types, and utilities
 * required throughout the project.
 ***********************************************************************************/
#include <stddef.h>  /**< Standard definitions for NULL and size_t */
#include <stdint.h>  /**< Fixed-width integer type definitions */
#include <stdbool.h> /**< Boolean type definitions (true/false) */
#include <stdio.h>   /**< Standard input/output functions */
#include <stdarg.h>  /**< Variable argument list handling (va_list, va_start, etc.) */
#include <stdlib.h>  /**< Standard library functions (malloc, free, rand, etc.) */
#include <string.h>  /**< String manipulation functions (memcpy, strlen, etc.) */
#include <assert.h>  /**< Debugging support for assertions */
#include <time.h>    /**< Time manipulation functions (time_t, clock, etc.) */
#include <ctype.h>   /**< Character classification and conversion functions */

#include "octopus_configuration.h"
#include "octopus_log.h" // Include logging functions for debugging
/***********************************************************************************
 * Project-wide Macros (Optional)
 * Define any global macros that may be required across multiple modules here.
 ***********************************************************************************/
// #define OCTOPUS_DEBUG    1   /**< Enable debug logging globally */

/*******************************************************************************
 * BIT MANIPULATION MACROS
 * Define macros for setting, clearing, toggling, and extracting bit values.
 ******************************************************************************/
/* --- Single bit masks --- */
#define BIT_0 (1UL << 0)
#define BIT_1 (1UL << 1)
#define BIT_2 (1UL << 2)
#define BIT_3 (1UL << 3)
#define BIT_4 (1UL << 4)
#define BIT_5 (1UL << 5)
#define BIT_6 (1UL << 6)
#define BIT_7 (1UL << 7)
#define BIT_8 (1UL << 8)

/* --- Single bit operations --- */
#define SETBIT(VAR, POS) ((VAR) |= (1UL << (POS)))  // Set bit at position POS
#define CLRBIT(VAR, POS) ((VAR) &= ~(1UL << (POS))) // Clear bit at position POS
#define TOGBIT(VAR, POS) ((VAR) ^= (1UL << (POS)))  // Toggle bit at position POS
#define GETBIT(VAR, POS) (((VAR) >> (POS)) & 0x1UL) // Get bit value (0 or 1) at position POS

/* --- Multi-bit operations --- */
#define SETBITS(VAR, MASK) ((VAR) |= (MASK))  // Set multiple bits defined by MASK
#define CLRBITS(VAR, MASK) ((VAR) &= ~(MASK)) // Clear multiple bits defined by MASK
#define TOGBITS(VAR, MASK) ((VAR) ^= (MASK))  // Toggle multiple bits defined by MASK
#define GETBITS(VAR, MASK) ((VAR) & (MASK))   // Get raw masked value of multiple bits

/* --- Multi-bit value extraction and assignment ---
 * Example:
 *   val = GETBITS_VALUE(REG, 4, 3); // Extract 4 bits starting from position 3
 */
#define GETBITS_VALUE(VAR, WIDTH, POS) (((VAR) >> (POS)) & ((1UL << (WIDTH)) - 1UL))

/* Assign a value to a specific bit field:
 *   Updates bits [POS + WIDTH - 1 : POS] of VAR with VALUE.
 */
#define SETBITS_VALUE(VAR, WIDTH, POS, VALUE)                    \
    do                                                           \
    {                                                            \
        (VAR) = ((VAR) & ~(((1UL << (WIDTH)) - 1UL) << (POS))) | \
                (((VALUE) & ((1UL << (WIDTH)) - 1UL)) << (POS)); \
    } while (0)

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
// Macros for byte and word manipulation
#define LSB_BIT(BYTE) ((BYTE) & 0x0F)        // Extract least significant nibble
#define MSB_BIT(BYTE) (((BYTE) >> 4) & 0x0F) // Extract most significant nibble

#define LSB_WORD(a) ((uint8_t)((a) & 0xFF))        // Extract least significant byte
#define MSB_WORD(a) ((uint8_t)(((a) >> 8) & 0xFF)) // Extract most significant byte

#define MK_LSB(a) (uint8_t)(a & 0xFF)                              // Extract the least significant byte (LSB) from a 16-bit word
#define MK_MSB(a) (uint8_t)((a >> 8) & 0xFF)                       // Extract the most significant byte (MSB) from a 16-bit word
#define MK_LSBWORD(a) (uint16_t)(a & 0xFFFF)                       // Extract the least significant word (LSB) from a 32-bit word
#define MK_MSBWORD(a) (uint16_t)((a >> 16) & 0xFFFF)               // Extract the most significant word (MSB) from a 32-bit word
#define MK_BYTE(MSB, LSB) ((uint8_t)(((MSB) << 4) | (LSB)))        // Combine two 4-bit values (MSB and LSB) into a single byte (alternative method)
#define MK_WORD(MSB, LSB) (uint16_t)(((uint16_t)MSB << 8) + LSB)   // Combine two 8-bit values (MSB and LSB) into a 16-bit word
#define MK_DWORD(MSB, LSB) (uint32_t)(((uint32_t)MSB << 16) + LSB) // Combine two 16-bit values (MSB and LSB) into a 32-bit double word
#define MK_SIG_WORD(a) (*(int16_t *)(&a))                          // Interpret a word as a signed 16-bit value

#define BYTES_TO_UINT32_LE(p)   \
    (((uint32_t)(p)[0]) |       \
     ((uint32_t)(p)[1] << 8) |  \
     ((uint32_t)(p)[2] << 16) | \
     ((uint32_t)(p)[3] << 24))

#define BYTES_TO_UINT32_BE(p)   \
    (((uint32_t)(p)[0] << 24) | \
     ((uint32_t)(p)[1] << 16) | \
     ((uint32_t)(p)[2] << 8) |  \
     ((uint32_t)(p)[3]))

#define UINT32_TO_BYTES_LE(val, p)                \
    do                                            \
    {                                             \
        (p)[0] = (uint8_t)((val) & 0xFF);         \
        (p)[1] = (uint8_t)(((val) >> 8) & 0xFF);  \
        (p)[2] = (uint8_t)(((val) >> 16) & 0xFF); \
        (p)[3] = (uint8_t)(((val) >> 24) & 0xFF); \
    } while (0)
/*******************************************************************************
 * CONSTANTS
 * Define mathematical constants and other useful values.
 ******************************************************************************/
#define PI_FLOAT (3.14159f) // Value of as a double/floating-point constant

/*******************************************************************************
 * FUNCTION DECLARATIONS
 * Declare any external functions used in this file.
 ******************************************************************************/
#define MY_ASSERT(expr)                                           \
    do                                                            \
    {                                                             \
        if (!(expr))                                              \
        {                                                         \
            LOG_LEVEL("ASSERT WARNING: %s, FILE: %s, LINE: %d\n", \
                      #expr, __FILE__, __LINE__);                 \
        }                                                         \
    } while (0)

/***********************************************************************************
 * End of File
 ***********************************************************************************/
#endif /* __OCTOPUS_BASE_H__ */
