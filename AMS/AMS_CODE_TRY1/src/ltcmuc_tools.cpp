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

#include "ltcmuc_tools.h"

#ifdef ARDUINO
// convert floating point number to string using scientific format
String float2Scientific(float val, uint8_t digits)
{
	if (val == 0.0)
		return String(val, digits);

	int8_t e = 0;
	boolean isNeg = val < 0;
	float scaledVal = isNeg ? -val : val;
	if (scaledVal < 1.0)
	{
		do
		{
			scaledVal = scaledVal * 10;
			e--;
		} while (scaledVal < 1.0);
	}
	else
	{
		while (scaledVal >= 10.0)
		{
			scaledVal = scaledVal / 10;
			e++;
		}
	}
	if (e == 0)
		return String(val, digits);

	if (isNeg)
		scaledVal = -scaledVal;

	String ret = String(scaledVal, digits);
	ret += 'e';
	ret += e;
	return ret;
}
#endif

int32_t LTC_4BytesToInt32(byte* bytes)
{
	int32_t ret;

	ret = *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	return ret;
}

int32_t LTC_3BytesToInt32(byte* bytes)
{
	int32_t ret;
	// sign extension
	if (*bytes & 0x80)
		ret = 0xFF00;
	else
		ret = 0;

	ret |= *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	return ret;
}

int64_t LTC_6BytesToInt64(byte* bytes)
{
	int64_t ret = (bytes[0] & 0x80)
		? 0xFFFFL
		: 0L;
	for (byte i = 0; i < 6; i++)
	{
		ret = ret << 8;
		ret |= bytes[i];
	}
	return ret;
}

int16_t LTC_2BytesToInt16(byte* bytes)
{
	int16_t ret;
	ret = *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	return ret;
}

uint32_t LTC_4BytesToUInt32(byte* bytes)
{
	uint32_t ret;

	ret = *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	return ret;
}

uint32_t LTC_3BytesToUInt32(byte* bytes)
{
	uint32_t ret;

	ret = *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	return ret;
}

uint16_t LTC_2BytesToUInt16(byte* bytes)
{
	uint16_t ret;
	ret = *bytes;
	bytes++;
	ret = ret << 8;
	ret |= *bytes;
	return ret;
}

void SerialPrintByteArrayHex(byte* data, uint8_t len, boolean addZeroX, boolean addComma)
{
	if (addZeroX && !addComma)
		PrintZeroX();
	for (uint8_t i = 0; i < len; i++)
	{
		if (addZeroX && addComma)
			PrintZeroX();
		SerialPrintByteX2(data[i]);
		if (addComma)
			PrintComma();
	}
}

void SerialPrintByteX2(uint8_t value)
{
	uint8_t asciiHex;

	asciiHex = (value >> 4) + 0x30;
	if (asciiHex > 0x39)
		asciiHex += 7;//A...F

#ifdef ARDUINO
	Serial.write(asciiHex);
#else
	printf("%c", asciiHex);
#endif

	asciiHex = (value & 0x0F) + 0x30;
	if (asciiHex > 0x39)
		asciiHex += 7;//A...F

#ifdef ARDUINO
	Serial.write(asciiHex);
#else
	printf("%c", asciiHex);
#endif
}

#ifdef ARDUINO
static inline void ConvertType(int* /*   */ out, String in) { *out = toIntAuto(in); }
static inline void ConvertType(long* /*  */ out, String in) { *out = toIntAuto(in); }
static inline void ConvertType(float* /* */ out, String in) { *out = in.toFloat(); }
static inline void ConvertType(String* /**/ out, String in) { *out = in; }

template<typename T>
boolean stringSplitter(String str, unsigned int* iStart, T* out)
{
	if (*iStart >= str.length())
		return false;
	int iEnd = str.indexOf(',', *iStart);
	if (iEnd == -1)
	{
		ConvertType(out, str.substring(*iStart));
		return false;
	}
	ConvertType(out, str.substring(*iStart, iEnd));
	*iStart = iEnd + 1;
	return (*iStart < str.length());
}
template boolean stringSplitter<>(String, unsigned int*, int*);
template boolean stringSplitter<>(String, unsigned int*, long*);
template boolean stringSplitter<>(String, unsigned int*, float*);
template boolean stringSplitter<>(String, unsigned int*, String*);
#endif

void PrintComma()
{
#ifdef ARDUINO
	Serial.print(',');
#else
	printf(",");
#endif
}

void PrintZeroX()
{
#ifdef ARDUINO
	Serial.print(F("0x"));
#else
	printf(F("0x"));
#endif
}

#ifdef ARDUINO
boolean _equals(String a, const __FlashStringHelper* ifsh, boolean ignoreLength)
{
	PGM_P p = reinterpret_cast<PGM_P>(ifsh);
	size_t n = 0;
	while (1)
	{
		char c = pgm_read_byte(p++);
		if (c == '\0')
			break;
		if (n >= a.length())
			return false;
		if (c != a.charAt(n))
			return false;
		n++;
	}
	return ignoreLength || n == a.length();
}

long toIntAuto(String a)
{
	byte bytes[sizeof(long)]; // NOTE: on Arduino Uno long is 4 bytes!
	uint8_t n = StringToBytes(a, sizeof(long), bytes);
	long ret = 0;
	for (uint8_t i = 0; i < n; i++)
	{
		ret <<= 8;
		ret |= bytes[i];
	}
	return ret;
	//return strtol(a.c_str(), (char **)NULL, 0);
}

boolean startsWith(String a, const __FlashStringHelper* ifsh)
{
	return _equals(a, ifsh, true);
}

boolean equals(String a, const __FlashStringHelper* ifsh)
{
	return _equals(a, ifsh, false);
}
#endif

boolean IsTermChar(char character)
{
	return character == '\n' || character == '\r' || character == '\0';
}

void PrintOkErr(uint16_t error)
{
#ifdef ARDUINO
	if (error)
	{
		Serial.print(F("ERR:"));
		PrintZeroX();
		Serial.println(error, HEX);
	}
	else
	{
		Serial.println(F("OK"));
	}
#else
	if (error)
		printf("ERR:0x%02hhX\n", error);
	else
		printf(F("OK\n"));
#endif

	}

uint8_t SingleAscii2uint8(char c)
{
	if (c <= '9')
		return c - '0';
	else if (c <= 'F')
		return c - 'A' + (char)10;
	return c - 'a' + (char)10;
}


#ifdef ARDUINO
unsigned int StringToBytes(String str, unsigned int len, byte* data)
{
	unsigned int i;
	// check for HEX string
	boolean isHex = (str.length() > 2) && startsWith(str, F("0x"));
	if (isHex)
	{
		i = 0;
		// from character index
		unsigned int from = 2;
		// to character index Not Including
		unsigned int toNI = (str.length() % 2 == 0) ? 4 : 3; // 3 in case odd number of hex characters
		// from 2 to 3 means only one Character

		while (true)
		{
			if (toNI > str.length())
				break; // end of string

			if (i < len)
			{
				data[i] = SingleAscii2uint8(str.charAt(from));
				from++;
				if (from != toNI)
				{
					data[i] = data[i] << 4 | SingleAscii2uint8(str.charAt(from));
					from++;
				}
				i++;
				//data[i++] = strtol(str.substring(from, toNI).c_str(), (char **)NULL, 16);
			}
			else
				break;

			//from = toNI;
			toNI += 2; // next two hex characters
		}
		return i;
	}
	i = len;
	long n = str.toInt(); //trtol(str.c_str(), (char **)NULL, 0);
	while (true)
	{
		if (i == 0)
			break;
		i--;
		data[i] = n;
		n >>= 8;
	}
	return len;
}
#endif


/* ***************************************************
  BASIC PEC functions:
****************************************************** */

#ifndef PEC15_USE_PRECOMPUTED_TABLE
// called by pec15_calc
int16_t _pec15_calc_internal_(uint16_t remainder)
{
	remainder <<= 7;
	for (uint8_t i = 0; i < 8; i++) //BIT8 to BIT0
	{
		remainder <<= 1;
		if (remainder & 0x8000U)
			remainder ^= PEC15_POLY;
	}
	return remainder;
}
#else
// called by pec15_calc
#define _pec15_calc_internal_(ADDR) crc15Table[ADDR]
// precomputed CRC15 Table
const uint16_t crc15Table[256]{
	0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
	0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
	0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
	0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
	0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
	0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
	0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
	0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
	0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
	0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
	0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
	0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
	0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
	0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
	0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
	0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
	0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
	0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
	0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
	0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
	0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
	0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
	0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};
// Without PROGMEM on LINDUINO it will be stored to RAM
//(Note: On Linduino you would even have to do "pgm_read_word_near(crc15Table+addr)" as it is not possible to directly
// access program memory.)
#endif

// called for every byte
void pec15_calc(uint16_t* pec15, byte nextByte)
{
	uint8_t addr = (uint8_t)(((*pec15) >> 7) ^ nextByte);
	(*pec15) = ((*pec15) << 8) ^ _pec15_calc_internal_(addr);
}

/*
PEC calculator HOWTO:

1. Initialize PEC variable

	  uint16_t pec15;
	  pec15_calc_init(&pec15);

2. For every byte call pec15_calc

	  for (int i = 0; i < bytes; i++)
		pec15_calc(&pec15, data[i]);

3. Finalize PEC calculation
	  pec15_calc_finalize(&pec15);

4. Check PEC (salve to master) or write PEC (master to slave)
	  // salve to master:
	  boolean pecFromSlaveOK = (uint16_t)(PEC0<<8 | PEC1) == pec15;
	  // master to slave:
	  uint8_t PEC0 = (uint8_t)(pec15>>8);
	  uint8_t PEC1 = (uint8_t)pec15;
	  spi_write(PEC0);
	  spi_write(PEC1);

*/

// currently not used for Linduino as PEC calculation is done
// on the fly for every byte. Still this function is kept as an
// example on how to use the PEC calculation functions according to
// above description
uint16_t pec15_calc_byte_array(
	byte* data, // Array of data that will be used to calculate a PEC
	byte len     // Number of bytes that will be used to calculate a PEC
)
{
	uint16_t pec15;
	pec15_calc_init(&pec15);
	for (byte j = 0; j < len; j++)
		pec15_calc(&pec15, data[j]);
	pec15_calc_finalize(&pec15);
	return pec15;
}

uint16_t pec10_calc_int(uint16_t remainder, byte bit)
{
	//Perform modulo-2 division, a bit at a time.
	//Perform modulo-2 division, a bit at a time.
	for (; bit > 0; --bit)
	{
		//Try to divide the current data bit.
		if ((remainder & 0x200) > 0)//equivalent to remainder & 2^14 simply check for MSB
		{
			remainder = (uint16_t)((remainder << 1));
			remainder = (uint16_t)(remainder ^ PEC10_POLY);
		}
		else
		{
			remainder = (uint16_t)(remainder << 1);
		}
	}
	return remainder;
}

uint16_t pec10_calc(
	byte* data, // Array of data that will be used to calculate a PEC
	byte format // use any of PEC10_...
)
{
	uint16_t remainder = 16; // PEC_SEED;
	uint16_t len;

	switch (format)
	{
	case PEC10_WRITE:
		data[6] = 0; // for write commands the command counter is all zero
		// step through
	case PEC10_READ:
		len = 6;
		break;
	case PEC10_READ256:
		len = 256;
		break;
	case PEC10_READ512:
		len = 512;
		break;
	default:
		return 0xFFFF;
		break;
	}

	//Perform modulo-2 division, a byte at a time.
	for (byte pbyte = 0; pbyte < len; ++pbyte)
	{
		// Bring the next byte into the remainder.
		remainder ^= (uint16_t)(data[pbyte]) << 2;
		remainder = pec10_calc_int(remainder, 8);
	}
	// the last byte is different as it holds the 6-bit command counter
	// Note: for write commands, those bits are zero!
	remainder ^= (uint16_t)((data[len] & 0xFC) << 2);
	remainder = pec10_calc_int(remainder, 6);
	return ((uint16_t)(remainder & 0x3FF));
}


/*
int16_t _pec10_calc_internal_(int i)
{
	uint16_t remainder;

#define LTC2949_CRC10_POLY 0x8F

	remainder = i << 7;
	for (int bit = 8; bit > 0; --bit)
	{

		if (remainder & 0x200)
		{
			remainder = ((remainder << 1));
			remainder = (remainder ^ LTC2949_CRC10_POLY);
		}
		else
		{
			remainder = ((remainder << 1));
		}
	}
	return  remainder & 0xFFFF;
}

int16_t _pec15_calc_internal_(int i)
{
	uint16_t remainder;

#define PEC15_POLY 0x4599

	remainder = i << 7;
	for (int bit = 8; bit > 0; --bit)
	{

		if (remainder & 0x4000)
		{
			remainder = ((remainder << 1));
			remainder = (remainder ^ PEC15_POLY);
		}
		else
		{
			remainder = ((remainder << 1));
		}
	}
	return  remainder & 0xFFFF;
}

uint16_t pec10_calc(bool rx, unsigned int len, uint8_t data[])
{

	unsigned int  remainder = 16;//PEC_SEED;
	unsigned int polynom = 0x8F;//x10 + x7 + x3 + x2 + x + 1 <- the CRC15 polynomial         100 1000 1111

   //Perform modulo-2 division, a byte at a time.
	for (int pbyte = 0; pbyte < len; ++pbyte)
	{

		//Bring the next byte into the remainder.

		remainder ^= (data[pbyte] << 2);

		//Perform modulo-2 division, a bit at a time.
		for (int bit = 8; bit > 0; --bit)
		{
			//Try to divide the current data bit.
			if ((remainder & 0x200) > 0)//equivalent to remainder & 2^14 simply check for MSB
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ polynom);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}
	}

	if (rx)
	{
		remainder ^= (data[len] & 0xFC) << 2; // 6 command counter bits

		//Perform modulo-2 division, a bit at a time.
		for (int bit = 6; bit > 0; --bit)
		{
			//Try to divide the current data bit.
			if ((remainder & 0x200) > 0)//equivalent to remainder & 2^14 simply check for MSB
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ polynom);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}
	}
	return ((remainder & 0x3FF));

}
*/
