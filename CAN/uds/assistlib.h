
#ifndef _COMMON_ASSIST_LIB_H_
#define _COMMON_ASSIST_LIB_H_
#include <stdint.h>

#define BIT_0           0x01
#define BIT_1           0x02
#define BIT_2           0x04
#define BIT_3           0x08
#define BIT_4           0x10
#define BIT_5           0x20
#define BIT_6           0x40
#define BIT_7           0x80

#define SetBit(VAR, Place)      ((VAR) |= (uint8_t)((uint8_t)1<<(uint8_t)(Place)))
#define ClrBit(VAR, Place)      ((VAR) &= (uint8_t)((uint8_t)((uint8_t)1<<(uint8_t)(Place))^(uint8_t)255))
#define ValBit(VAR, Place)      ((uint8_t)(VAR) & (uint8_t)((uint8_t)1<<(uint8_t)(Place)))
#define ChgBit(Var, Place)      ((Var) ^= (uint8_t)((uint8_t)1<<(uint8_t)(Place)))
#define GetBit(Var,Place)       ((Var& (1<<(Place&0x7)))==0?0:1)

#define LSB_BIT(BYTE)           (uint8_t)(BYTE & 0x0F)
#define MSB_BIT(BYTE)           (uint8_t)((BYTE >> 4) & 0x0F)

#define LSB(WORD)               (uint8_t)(WORD & 0xFF)
#define MSB(WORD)               (uint8_t)((WORD >> 8) & 0xFF)

#define LSBWORD(WORD)           (uint16_t)(WORD & 0xFFFF)
#define MSBWORD(WORD)           (uint16_t)((WORD >> 16) & 0xFFFF)

#define LSBDWORD(WORD)           (uint32_t)(WORD & 0xFFFFFFFF)
#define MSBDWORD(WORD)           (uint32_t)((WORD >> 32) & 0xFFFFFFFF)

#define BYTE(MSB, LSB)          (uint8_t)(((uint8_t)MSB << 4) + LSB)
#define WORD(MSB, LSB)          (uint16_t)(((uint16_t)MSB << 8) + LSB)
#define DWORD(MSB, LSB)         (uint32_t)(((uint32_t)MSB << 16) + LSB)
#define QWORD(MSB, LSB)         (uint64_t)(((uint64_t)MSB << 32) + LSB)

#endif
/*
    File End
*/
