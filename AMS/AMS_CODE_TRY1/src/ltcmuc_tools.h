/************************************

Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************/

#ifndef _LTCMUC_TOOLS_H
#define _LTCMUC_TOOLS_H

#ifdef ARDUINO
#include <Arduino.h>
#else

#include "../../tools/ad_lx_tools.h"

#endif

#define bitsMaskApplyToReg(REG, BITS, MASK) {BITS &= (MASK); REG &= ~(MASK); REG |= (BITS);}

#define bitMaskSetChk(value, bitMask) (((value) & (bitMask)) == (bitMask))
#define bitMaskClrChk(value, bitMask) (((value) & (bitMask)) == 0)

#define bitMaskSetClrChk(value, bitMask, setNotClr) ((setNotClr) ? bitMaskSetChk(value,bitMask) : bitMaskClrChk(value,bitMask))


#define bitMaskSet(value, bitMask) (value |=  (bitMask))
#define bitMaskClr(value, bitMask) (value &= ~(bitMask))

#define bitMaskCpy(des, src, bitMask) {des &= ~bitMask;des |= bitMask & src;}


#define isCharZero(charVal) ((charVal) == '0')


#define LOG_1(n) (((n) >= 2) ? 1 : 0)
#define LOG_2(n) (((n) >= 1<<2) ? (2 + LOG_1((n)>>2)) : LOG_1(n))
#define LOG_4(n) (((n) >= 1<<4) ? (4 + LOG_2((n)>>4)) : LOG_2(n))
#define LOG_8(n) (((n) >= 1<<8) ? (8 + LOG_4((n)>>8)) : LOG_4(n))
#define LOG(n)   (((n) >= 1<<16) ? (16 + LOG_8((n)>>16)) : LOG_8(n))


#define bitMaskSetClr(value, bitMask, setNotClr) ((setNotClr) ? bitMaskSet(value,bitMask) : bitMaskClr(value,bitMask))

// used to convert any bitmask like LTC2947_BM_TBCTL_DIV_0
// to its bit position index to be used for shift operations
// BM2BITPOS(LTC2947_BM_TBCTL_DIV_0) = BM2BITPOS(0x8) = BM2BITPOS(1<<3) = 3
#define BM2BITPOS(bm) ((bm)&1<<7 ? 7 : \
             (bm)&1<<6 ? 6 : \
             (bm)&1<<5 ? 5 : \
             (bm)&1<<4 ? 4 : \
             (bm)&1<<3 ? 3 : \
             (bm)&1<<2 ? 2 : \
             (bm)&1<<1 ? 1 : 0)

//#define LTC_INT16_MAX ((int16_t)0x7fffUL)
//#define LTC_INT16_MIN ((int16_t)0x8000UL)
#define LTC_ASSIGN_MAX(OUTVAL,INVAL) {if ((INVAL) > (OUTVAL)) (OUTVAL) = (INVAL);}
#define LTC_ASSIGN_MIN(OUTVAL,INVAL) {if ((INVAL) < (OUTVAL)) (OUTVAL) = (INVAL);}
// timeout checker: true if time has elapsed, false if not
#define LTC_TIMEOUT_CHECK(CURRENT_TIME,TIMEOUT) (((long)((CURRENT_TIME) - (TIMEOUT)) >= 0L))


#define LTC_SERIAL_PRINT_BOOL(A,ON,OFF) { Serial.print((A) ? (ON) : (OFF));}
#define LTC_SERIAL_PRINT_BOOLD(A,ON)    { Serial.print((A) ? (ON) : '_');}

int16_t LTC_2BytesToInt16(byte* bytes);
int32_t LTC_3BytesToInt32(byte* bytes);
int32_t LTC_4BytesToInt32(byte* bytes);
uint16_t LTC_2BytesToUInt16(byte* bytes);
uint32_t LTC_3BytesToUInt32(byte* bytes);
uint32_t LTC_4BytesToUInt32(byte* bytes);
int64_t LTC_6BytesToInt64(byte* bytes);

uint8_t SingleAscii2uint8(char c);


static inline int32_t LTC_4BytesToInt32(byte* bytes, uint8_t offset)
{
	return LTC_4BytesToInt32(bytes + offset);
}

static inline int32_t LTC_3BytesToInt32(byte* bytes, uint8_t offset)
{
	return LTC_3BytesToInt32(bytes + offset);
}

static inline int16_t LTC_2BytesToInt16(byte* bytes, uint8_t offset)
{
	return LTC_2BytesToInt16(bytes + offset);
}

static inline uint32_t LTC_4BytesToUInt32(byte* bytes, uint8_t offset)
{
	return LTC_4BytesToUInt32(bytes + offset);
}

static inline uint32_t LTC_3BytesToUInt32(byte* bytes, uint8_t offset)
{
	return LTC_3BytesToUInt32(bytes + offset);
}

static inline uint16_t LTC_2BytesToUInt16(byte* bytes, uint8_t offset)
{
	return LTC_2BytesToUInt16(bytes + offset);
}


void SerialPrintByteX2(uint8_t value);
void SerialPrintByteArrayHex(byte* data, uint8_t len, boolean addZeroX = false, boolean addComma = false);

boolean IsTermChar(char character);


void PrintComma();
void PrintZeroX();
void PrintOkErr(uint16_t error);


#ifdef ARDUINO

template<typename T>
boolean stringSplitter(String str, unsigned int* iStart, T* out);

unsigned int StringToBytes(String str, unsigned int len, byte* data);

long toIntAuto(String a);


boolean equals(String a, const __FlashStringHelper* ifsh);
boolean startsWith(String a, const __FlashStringHelper* ifsh);
String float2Scientific(float val, uint8_t digits);
#endif

/* ***************************************************
  BASIC PEC functions:
****************************************************** */
#ifndef ARDUINO
#define PEC15_USE_PRECOMPUTED_TABLE
#endif

#ifdef ARDUINO_SAM_DUE
//#define PEC15_USE_PRECOMPUTED_TABLE
#endif // ARDUINO_SAM_DUE

#define PEC10_POLY 0x8FU
#define PEC15_POLY 0x4599U

static inline void pec15_calc_init(uint16_t * pec15)
{
	(*pec15) = 16U; // initialize with PEC15 calculator seed value
}

static inline void pec15_calc_finalize(uint16_t* pec15)
{
	(*pec15) = (*pec15) << 1U;//The CRC15 has a 0 in the LSB so the pec15 must be multiplied by 2
}

uint16_t pec15_calc_byte_array(
	byte* data, // Array of data that will be used to calculate a PEC
	byte len     // Number of bytes that will be used to calculate a PEC
);

void pec15_calc(
	uint16_t* pec15,
	byte nextByte);

uint16_t pec10_calc(
	byte* data, // Array of data that will be used to calculate a PEC
	byte format // use any of PEC10_...
);


#define PEC10_NONE    0
#define PEC10_READ    1
#define PEC10_CMD     1
#define PEC10_WRITE   2
#define PEC10_READ256 3
#define PEC10_READ512 4

#ifndef PEC15_USE_PRECOMPUTED_TABLE
// called by pec15_calc
int16_t _pec15_calc_internal_(uint16_t remainder);
#endif

//#define pec15_calc_68xy pec15_calc_byte_array

#endif
