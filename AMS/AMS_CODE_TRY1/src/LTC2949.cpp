/*!
LTC2949 Battery Monitor Coulomb Counter
@verbatim
The LTC2949 is....
@endverbatim

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
//! @defgroup LTC2949 LTC2949: Battery Monitor Coulomb Counter

/*! @file
@ingroup LTC2949
Library for LTC2949 Battery Monitor Coulomb Counter
*/

#include "LTC2949.h"

/*!**********************************************************************
\brief  global variables
***********************************************************************/
// in indirect address mode we always write a burst of 4 bytes to RDCVIADDR which includes a write to REGSCTRL
byte LTC2949_iAddrRegsCtrl[LTC2949_IADDRREGSCTRL_LEN];
byte LTC2949_gpioCtrl[2];
byte LTC2949_adcConf; // PasV, NTC register

uint32_t LTC2949_lastTBx; // stores the last TBx value

//! enter the number of cell monitors in the daisychain (0 if there are no cell monitors)
//! the daisychain can be parallel to LTC2949 or LTC2949 is on top of the daisychain, see LTC2949_onTopOfDaisychain
uint8_t LTC2949_CellMonitorCount;
boolean LTC2949_onTopOfDaisychain;
boolean LTC2949_DebugEnable;
boolean LTC2949_forceWrRegsCtrl;
boolean LTC2949_autoForceWrRegsCtrl;

uint8_t LTC2949_CS = 10;// SS; SS is mapped differently on Arduino Zero

#ifdef ARDUINO

SPISettings LTC2949_SPISettings = SPISettings(LTC2949_DEFAULT_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);

#else

bool spiBufferBusy = true;
unsigned int spiBufferLen = 0;
uint8_t spiBuffer[SPIBUFFERMAXLEN];
AdiSpiAsync* asp = NULL;
void LTC2949_InitSPI(AdiSpiAsync* asa)
{
	asp = asa;
}

#endif

double LTC2949_TBFAC;

// holds the next timestamp when isoSPI timeout duration has elapsed
unsigned long LTC2949_isoSPI_wakeup_timeoutUs;

#ifdef LTC2949_CONFIG_MOSI_FOR_READ
uint8_t LTC2949_MOSI_FOR_READ = 0xFFU;
#endif

#ifdef LTC2949_STORETIME_LAST_PEC
unsigned long LTC2949_TimeLastPEC;
#endif

//void LTC2949_ADCConfigUpdate(byte data)
//{
//	LTC2949_adcConf = data;
//	LTC2949_p1asv = bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P1ASV);
//	LTC2949_p2asv = bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P2ASV);
//	LTC2949_ntc1 = bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC1);
//	LTC2949_ntc2 = bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC2);
//}

boolean LTC2949_AnyPasV()
{
	return (LTC2949_adcConf & (LTC2949_BM_ADCCONF_P1ASV | LTC2949_BM_ADCCONF_P2ASV)) != 0;//LTC2949_p1asv || LTC2949_p2asv;
}

/*!
  \brief This function will initialize all 2949 variables and the SPI port.

  This function will initialize the Linduino to communicate with the LTC2949 with a 0.5MHz SPI clock.
  */
void LTC2949_init_lib(
	byte cellMonitorCount,
	boolean ltc2949onTopOfDaisychain,
	boolean debugEnable)
{
	LTC2949_CellMonitorCount = cellMonitorCount;

	LTC2949_DebugEnable = debugEnable;
	LTC2949_onTopOfDaisychain = ltc2949onTopOfDaisychain;

	// make sure the default value adjusted to e.g. BCREN is written with the next transaction
	LTC2949_forceWrRegsCtrl = true;
	LTC2949_autoForceWrRegsCtrl = true;
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] = LTC2949_BM_REGSCTRL_RDCVCONF;
	//LTC2949_init_device_state(); // don't do this here, as it also resets local stored device configuration (e.g. LTC2949_TBFAC,ADCConfig,GpioCurrConfig)

#ifdef ARDUINO
	// select SPI for QuikEval connector
	LinduinoSelectSPI();

	// start the SPI library: (this also sets SS pin as output)
	SPI.begin();
#else
	spiBufferLen = 0;
	spiBufferBusy = true;
#endif
}

void LTC2949_sel_rd_mode(
	byte cellMonitorCount,
	boolean ltc2949onTopOfDaisychain)
{
	LTC2949_forceWrRegsCtrl = true;
	LTC2949_autoForceWrRegsCtrl = true;
	LTC2949_CellMonitorCount = cellMonitorCount;
	LTC2949_onTopOfDaisychain = ltc2949onTopOfDaisychain;
}

// initialize several variables that hold LTC2949's device state (e.g. this is called after LTC2949_reset)
void LTC2949_init_device_state()
{
	LTC2949_forceWrRegsCtrl = true;
	LTC2949_autoForceWrRegsCtrl = true;
	LTC2949_isoSPI_wakeup_timeoutUs = 0;
	LTC2949_TBFAC = LTC2949_INTC;
	//LTC2949_ADCConfigUpdate(0);
	LTC2949_adcConf = 0;
	LTC2949_GpioCurrConfigClr();
	// this is the default of REGSCTRL
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] = LTC2949_BM_REGSCTRL_RDCVCONF;
	LTC2949_lastTBx = 0; // stores the last TBx value
}


// Apply GPIO configuration to global variable LTC2949_gpioCtrl. 
// Call this function one ore more times to apply configuration for
// one or more GPIOs. Afterwards call function
// "LTC2949_GpioCurrConfigWrite" to write the actual 
// configuration to LTC2949
void LTC2949_GpioConfig(byte gpio, byte mode)
{
	// gpio  i  ADDR    REGNAME  	  BIT   BITNAME
	// 5     0  0xF1    FCURGPIOCTRL  1:0   GPO5CTRL
	// 1     1  0xF2    FGPIOCTRL	  1:0   GPO1CTRL
	// 2                              3:2   GPO2CTRL
	// 3                              5:4   GPO3CTRL
	// 4                              7:6   GPO4CTRL

	byte i;
	if (gpio == 0)
		return; // not allowed, ignore!
	if (gpio < 5)
	{
		i = 1;
		gpio--;
	}
	else if (gpio == 5)
	{
		i = 0;
		gpio = 0;
	}
	else
		return; // not allowed, ignore!

	gpio <<= 1U;

	LTC2949_gpioCtrl[i] &= ~(3U << gpio);
	LTC2949_gpioCtrl[i] |= mode << gpio;
}


byte LTC2949_GpioConfigGet(byte gpio)
{
	// gpio  i  ADDR    REGNAME  	  BIT   BITNAME
	// 5     0  0xF1    FCURGPIOCTRL  1:0   GPO5CTRL
	// 1     1  0xF2    FGPIOCTRL	  1:0   GPO1CTRL
	// 2                              3:2   GPO2CTRL
	// 3                              5:4   GPO3CTRL
	// 4                              7:6   GPO4CTRL

	byte i;
	if (gpio == 0)
		return 0; // not allowed, ignore!
	if (gpio < 5)
	{
		i = 1;
		gpio--;
	}
	else if (gpio == 5)
	{
		i = 0;
		gpio = 0;
	}
	else
	{
		return 0; // not allowed!
	}

	gpio <<= 1U;

	return (LTC2949_gpioCtrl[i] >> gpio) & 3U;
}


void LTC2949_CurrSrcConfig(byte currCtrl)
{
	// clear all bits related to current sources
	bitMaskClr(LTC2949_gpioCtrl[0],
		LTC2949_BM_FCURGPIOCTRL_MUXPCURPOL |
		LTC2949_BM_FCURGPIOCTRL_MUXPCUREN |
		LTC2949_BM_FCURGPIOCTRL_MUXNCURPOL |
		LTC2949_BM_FCURGPIOCTRL_MUXNCUREN
	);
	// set requested bits
	bitMaskSet(LTC2949_gpioCtrl[0], currCtrl & (
		LTC2949_BM_FCURGPIOCTRL_MUXPCURPOL |
		LTC2949_BM_FCURGPIOCTRL_MUXPCUREN |
		LTC2949_BM_FCURGPIOCTRL_MUXNCURPOL |
		LTC2949_BM_FCURGPIOCTRL_MUXNCUREN)
	);
}

// use with 
// LTC2949_BM_OCC_DACTHR_xxx for occ_Thr
// LTC2949_BM_OCC_DGLT_xxx   for occ_Dgl
// LTC2949_BM_OCC_POL_xxxx   for occ_Pol
// set occ_Thr to -1 to disable OCC
void LTC2949_OCCConfig(
	int8_t occ1Thr, uint8_t occ1Dgl, uint8_t occ1Pol,
	int8_t occ2Thr, uint8_t occ2Dgl, uint8_t occ2Pol)
{
	byte data[2] = { 0,0 };

	if (occ1Thr >= 0)
		data[0] = LTC2949_BM_OCC1CTRL_OCC1EN | occ1Thr | occ1Dgl | occ1Pol;
	if (occ2Thr >= 0)
		data[1] = LTC2949_BM_OCC2CTRL_OCC2EN | occ2Thr | occ2Dgl | occ2Pol;

	LTC2949_WRITE(LTC2949_REG_OCC1CTRL, 2, data);
}

void LTC2949_WriteTbctl(double extClk, boolean verbose)
{
	byte LTC2949_PRE;
	byte LTC2949_DIV;
	if (isnan(extClk) || (extClk > LTC2949_MAX_EXTCLK) || (extClk < LTC2949_MIN_EXTCLK))
	{
		LTC2949_PRE = 7;
		LTC2949_DIV = 0;
		LTC2949_TBFAC = LTC2949_INTC;
	}
	else
	{
		/**/ if (extClk <= LTC2949_PRE0_MAX_EXTCLK)
			LTC2949_PRE = 0;
		else if (extClk <= LTC2949_PRE1_MAX_EXTCLK)
			LTC2949_PRE = 1;
		else if (extClk <= LTC2949_PRE2_MAX_EXTCLK)
			LTC2949_PRE = 2;
		else if (extClk <= LTC2949_PRE3_MAX_EXTCLK)
			LTC2949_PRE = 3;
		else if (extClk <= LTC2949_PRE4_MAX_EXTCLK)
			LTC2949_PRE = 4;
		else // <= 25e6
			LTC2949_PRE = 5;

		LTC2949_DIV = extClk / LTC2949_TBC_NOM_FREQU / (double)(1U << LTC2949_PRE);
		LTC2949_TBFAC = (1 << LTC2949_PRE) * (LTC2949_DIV + 1) / extClk;
	}

#ifdef ARDUINO
	if (verbose)
	{
		Serial.print(F("TBC:"));
		Serial.print(extClk * 1e-6, 6);
		PrintComma();
		Serial.print(LTC2949_PRE);
		PrintComma();
		Serial.print(LTC2949_DIV);
		PrintComma();
		Serial.println(LTC2949_TBFAC * 1e6, 6);
	}
#endif

	LTC2949_WRITE(LTC2949_REG_TBCTRL, //! write PRE and DIV setting
		LTC2949_PRE << BM2BITPOS(LTC2949_BM_TBCTRL_PRE_0) |
		LTC2949_DIV << BM2BITPOS(LTC2949_BM_TBCTRL_DIV_0));
}

void LTC2949_MemLockRequest()
{
	// clear all MLK bits
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] &= ~(LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] |= LTC2949_BM_REGSCTRL_MLK0; // set lock request bit
	_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX]));
}

byte LTC2949_MemLockStatus(byte* mlk)
{
	byte error = LTC2949_READ(LTC2949_REG_REGSCTRL, 1, mlk);
	if (error)
		return error;
	mlk[0] &= LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0;
	return 0;
}

byte LTC2949_MemLockStatusInt()
{
	return LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] & (LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);
}

void LTC2949_MemLockRelease()
{
	// clear all MLK bits
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] &= ~(LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);
	_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX]));
}

/// returns 0 if no error
byte LTC2949_MemLock(boolean lock)
{
	// clear all MLK bits
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] &= ~(LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);

	if (lock)
		LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] |= LTC2949_BM_REGSCTRL_MLK0; // set lock request bit

	// request lock or unlock of memory (note: every time we write to special row, we can directly use _LTC2949_direct_write_ if we make sure to update LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] in case we write LTC2949_REG_REGSCTRL)
	_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX]));

	if (!lock)
		return 0; // in case of unlock we immediately return

	// calc timeout for lock request acknowledge
	unsigned long timeout = millis() + LTC2949_TIMING_MLOCK_ACK;

	byte error = 0;
	do
	{
		// read lock state
		error |= LTC2949_READ(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX]));

		// check any error except timeout
		if (error & ~LTC2949_ERRCODE_TIMEOUT) // here we ignore timeout
			break;

		// check valid lock acknowledge
		if (((LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] & (LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0)) == LTC2949_BM_REGSCTRL_MLK1)) // we expect this bit to be set after lock request
		{
			error &= ~LTC2949_ERRCODE_TIMEOUT;
			break;
		}

		// now we also break in case of timeout (as even the last read after timeout failed)
		if (error)
			break;

		if (LTC_TIMEOUT_CHECK(millis(), timeout))
			error |= LTC2949_ERRCODE_TIMEOUT;

	} while (1);

	// clear all lock bits
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] &= ~(LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);
	return error;
}

#ifdef LTCDEF_SPI_POLL_SUPPORT
byte LTC2949_68XX_CMD(
	uint8_t addr,
	uint16_t cmd,
	byte* data,
	uint16_t len,
	byte* data2949,
	uint16_t len2949,
	// timeout given in multiple of 16 us
	// 0:
	// 1:     16 us
	// 255: 4080 us
	uint8_t pollTimeout16Us
)
#else
byte LTC2949_68XX_CMD(
	uint8_t addr,
	uint16_t cmd,
	byte* data,
	uint16_t len,
	byte* data2949,
	uint16_t len2949
)
#endif
{
	if (LTC2949_DebugEnable)
#ifdef ARDUINO
		Serial.print(F("68:"));
#else
		printf(F("68:"));
#endif

	//This will guarantee that the LTC2949//LTC68xx isoSPI port is awake.
	LTC2949_wakeup_idle(LTC2949_forceWrRegsCtrl); // just to be on the safe side, in case we force write REGSCTRL we also force isoSPI wakeup pulses
	spiStart();

	uint16_t pec;
	pec15_calc_init(&pec);
	// send command with address
	spi_write_calc_pec(&pec, ((addr << 3) & 0xF8U) | ((cmd >> 8) & 0x7U));
	spi_write_calc_pec(&pec, cmd);
	// send PEC
	spi_write_pec(&pec);
#ifdef LTC2949_STORETIME_LAST_PEC
	LTC2949_TimeLastPEC = micros();
#endif

	// init for next PEC
	pec15_calc_init(&pec);

	byte error = 0;
	uint16_t length = LTC2949_CALC_MUL_6(len + len2949);

	switch (cmd & LTC2949_68XX_CMDTYPE_MASK)
	{
	case LTC2949_68XX_CMDTYPE_PL: // POLL COMMAND
		if (data2949 || len2949 > 0 || data || len > 0)
		{
			// 68xx poll command doesn't have any additional data!
			return LTC2949_ERRCODE_OTHER;
		}
		else
		{
#ifdef LTCDEF_SPI_POLL_SUPPORT
			unsigned long timeoutUs;
			timeoutUs = pollTimeout16Us + 1;
			timeoutUs <<= 4;
			timeoutUs += micros();
			while (true)
			{
				if (spi_read() == 0xFFU)
				{
					error &= ~LTC2949_ERRCODE_TIMEOUT;
					break;
				}
				if (error)
					break;
				if (LTC_TIMEOUT_CHECK(micros(), timeoutUs))
					error |= LTC2949_ERRCODE_TIMEOUT;
			}
#else
			// just read one additional byte in case of polling
			spi_read();
			break;
#endif
		}
		break;
	case LTC2949_68XX_CMDTYPE_WR: // WRITE COMMAND
		if (data2949 || len2949 > 0)
			// 68xx write command with data for LTC2949 does not exist!
			return LTC2949_ERRCODE_OTHER;
		if ((data == NULL) != (len == 0))
			// mismatch between data array and length (expect either data null and length 0 or other way around)
			return LTC2949_ERRCODE_OTHER;
		for (uint16_t i = 0; i < length;)
		{
			// write data
			spi_write_calc_pec(&pec, data[i++]);
			if (i % LTC2949_IR_RDCV_BYTES_PER_PEC != 0)
				continue;
			// send PEC bytes
			spi_write_pec(&pec);
			pec15_calc_init(&pec);
		}
		break;
	case LTC2949_68XX_CMDTYPE_RDPL: //  READ & POLL COMMAND
	case LTC2949_68XX_CMDTYPE_RD:   //  READ COMMAND
	default:
		uint16_t i = 0;
		for (; i < length;)
		{
#ifdef ARDUINO
			byte d = spi_read_calc_pec(&pec);
			if (i < len)
			{
				if (data) data[i] = d;
			}
			else if (data2949)
			{
				uint16_t j = i - len;
				if (j < len2949) data2949[j] = d;
			}
			i++;
			if (i % LTC2949_IR_RDCV_BYTES_PER_PEC != 0)
				continue;
			// read the PEC bytes and check for PEC error
			spi_read_chk_pec(&pec, &error);
			pec15_calc_init(&pec);
#else
			spi_read();
			i++;
			if (i % LTC2949_IR_RDCV_BYTES_PER_PEC != 0)
				continue;
			// read the PEC bytes
			spi_read();
			spi_read();
#endif
		}
		if (bitMaskSetChk(cmd, LTC2949_68XX_CMDTYPE_PL))
		{
#ifdef LTCDEF_SPI_POLL_SUPPORT
			unsigned long timeoutUs;
			timeoutUs = pollTimeout16Us + 1;
			timeoutUs <<= 4;
			timeoutUs += micros();

			while (true)
			{
				byte d = spi_read_calc_pec(&pec);
				// we always overwrite the last value, which is helpful for LTC2949's HS byte
				if (data2949 && len2949 > 0)
					data2949[len2949 - 1] = d;
				else if (data && len > 0)
					data[len - 1] = d;
				i++;
				if (i % LTC2949_IR_RDCV_BYTES_PER_PEC != 0)
					continue;
				// read the PEC bytes and check for PEC error
				spi_read_chk_pec(&pec, &error);
				pec15_calc_init(&pec);

				// we make greater equal, even though LTC2949's HS-OK is 0x0F
				// this way it is more general and will also trigger for not allowed values
				if (d >= 0x0FU)
				{
					error &= ~LTC2949_ERRCODE_TIMEOUT;
					break;
				}
				if (error)
					break;
				if (LTC_TIMEOUT_CHECK(micros(), timeoutUs))
					error |= LTC2949_ERRCODE_TIMEOUT;
			}
#else
			// just read one additional byte in case of polling
			spi_read();
			break;
#endif
		}
		break;
	}
	spiStop();
#ifdef ARDUINO
#else
	// process data in case of not ARDUINO
	pec15_calc_init(&pec);
	spiBufferLen = 4; // index of first data byte

	///////////////////////////////////////////////
	///////////////////////////////////////////////

	switch (cmd & LTC2949_68XX_CMDTYPE_MASK)
	{
	case LTC2949_68XX_CMDTYPE_PL: // POLL COMMAND
		// nothing to do
		// just read one additional byte in case of polling
		//spi_read(); // TODO: byte ignored
		break;
	case LTC2949_68XX_CMDTYPE_WR: // WRITE COMMAND
		// nothing to do
		break;
	case LTC2949_68XX_CMDTYPE_RDPL: //  READ & POLL COMMAND
	case LTC2949_68XX_CMDTYPE_RD:   //  READ COMMAND
	default:
		for (uint16_t i = 0; i < length;)
		{
			byte d = spi_read_calc_pec(&pec);
			if (i < len)
			{
				if (data) data[i] = d;
			}
			else if (data2949)
			{
				uint16_t j = i - len;
				if (j < len2949) data2949[j] = d;
			}
			i++;
			if (i % LTC2949_IR_RDCV_BYTES_PER_PEC != 0)
				continue;
			// read the PEC bytes and check for PEC error
			spi_read_chk_pec(&pec, &error);
			pec15_calc_init(&pec);
		}
		// TODO poll byte ignored
		break;
	}


	///////////////////////////////////////////////
	///////////////////////////////////////////////
#endif


	LTC2949_ProcessErrors(error);
	return error;
}


#if defined(LTC2949_DCRW_RAW_DATA_DEBUG) && defined(ARDUINO)
byte* _ltc2949_dcrw_raw_data = NULL;

uint16_t LTC2949_GetDcRwRawData(byte** dp)
{
	if (_ltc2949_dcrw_raw_data)
	{
		*dp = _ltc2949_dcrw_raw_data + 2;
		return ((uint16_t*)_ltc2949_dcrw_raw_data)[0];
	}
	return 0;
}
#endif

// in case of write the return value is always zero!
byte _LTC2949_DcRW_(byte addr, uint16_t len, byte* data, boolean readNotWrite)
{
#ifdef LTC2949_DEBUG_WR_REGSCTRL
	boolean LTC2949_DebugEnable_ = LTC2949_DebugEnable;
	{
		int16_t regsCtrlOffs = LTC2949_GetRegsCtrlDataOffset(addr, len);
		if (regsCtrlOffs >= 0)
		{
			LTC2949_DebugEnable = true;
			Serial.print(F("RC:"));
		}
	}
#endif
	if (LTC2949_DebugEnable)
#ifdef ARDUINO
		Serial.print(readNotWrite ? F("DR:") : F("DW:"));
#else
		printf(readNotWrite ? F("DR:") : F("DW:"));
#endif

	// calc optimum number of bytes per PEC
	byte bytesPerPec = LTC2949_CALC_OPT_NO_PECS(len);
	uint16_t pec;

#if defined(LTC2949_DCRW_RAW_DATA_DEBUG) && defined(ARDUINO)
	for (uint16_t rawbytes = len + len / bytesPerPec * 2;;)
	{
		if (_ltc2949_dcrw_raw_data)
			delete[] _ltc2949_dcrw_raw_data;
		_ltc2949_dcrw_raw_data = new byte[rawbytes + 2];
		*((uint16_t*)_ltc2949_dcrw_raw_data) = rawbytes;
		break;
	}
	byte* ltc2949_dcrw_raw_data = _ltc2949_dcrw_raw_data + 2;
#endif


	// init the pec calculator
	pec15_calc_init(&pec);

	//This will guarantee that the LTC2949 isoSPI port is awake.
	LTC2949_wakeup_idle(LTC2949_forceWrRegsCtrl); // just to be on the safe side, in case we force write REGSCTRL we also force isoSPI wakeup pulses
	spiStart();

	// direct read/write commands are always addressed!!!
	spi_write_calc_pec(&pec, LTC2949_ADDR_CMD);
	spi_write_calc_pec(&pec, addr);
	spi_write_pec(&pec);

	// write cmd_id (there is no PEC for the cmd_id byte as it has intrinsic checksum)
	spi_write(LTC2949_CALC_ID_BYTE(bytesPerPec, readNotWrite));

	byte error = 0;
	pec15_calc_init(&pec);
	if (readNotWrite)
	{
		// READ COMMAND
		for (uint16_t i = 0; i < len;)
		{
#ifdef ARDUINO
			data[i] = spi_read_calc_pec(&pec);
			error |= LTC2949_ERRCODE_COMMERR; // this is cleared after successful PEC check later
#ifdef LTC2949_DCRW_RAW_DATA_DEBUG
			* ltc2949_dcrw_raw_data = data[i];
			ltc2949_dcrw_raw_data++;
#endif
#else
			spi_write(LTC2949_MOSI_FOR_READ); // PEC check done later
#endif
			i++;

			if (i % bytesPerPec != 0)
				continue;

#ifdef ARDUINO
			// read the PEC bytes and check for PEC error
#ifdef LTC2949_DCRW_RAW_DATA_DEBUG
			pec15_calc_finalize(&pec);
			uint16_t pec_slave = spi_read();         //MSB 1st
			*ltc2949_dcrw_raw_data = pec_slave;
			ltc2949_dcrw_raw_data++;
			pec_slave = pec_slave << 8 | spi_read(); //LSB 2nd
			*ltc2949_dcrw_raw_data = pec_slave;
			ltc2949_dcrw_raw_data++;
			if (pec_slave != pec && ((error & LTC2949_ERRCODE_PECERR_MASK) < LTC2949_PEC_ERR_CNT_MAX))
				error++;
#else
			spi_read_chk_pec(&pec, &error);
#endif

			pec15_calc_init(&pec);
#else
			spi_write(LTC2949_MOSI_FOR_READ); // PEC check done later
			spi_write(LTC2949_MOSI_FOR_READ); // PEC check done later
#endif
			error &= ~LTC2949_ERRCODE_COMMERR; // PEC check done, so we clear it
			// Note: this ensures we always return with error LTC2949_ERRCODE_COMMERR
			//       in case we read data bytes without protection by PEC.
			// In other words: the two last bytes read must always be a PEC bytes.
		}
	}
	else
	{
		// WRITE COMMAND
		for (uint16_t i = 0; i < len;)
		{
			spi_write_calc_pec(&pec, data[i++]);

			if (i % bytesPerPec != 0)
				continue;

			// it's time to calc and send the PEC
			spi_write_pec(&pec);

			pec15_calc_init(&pec);
		}
	}
	spiStop();
#ifdef ARDUINO
#else
	// process data
	if (readNotWrite)
	{
		pec15_calc_init(&pec);
		spiBufferLen = 5; // index of first data byte
		// READ COMMAND
		for (uint16_t i = 0; i < len;)
		{
			data[i++] = spi_read_calc_pec(&pec);

			if (i % bytesPerPec != 0)
				continue;
			// read the PEC bytes and check for PEC error
			spi_read_chk_pec(&pec, &error);
			pec15_calc_init(&pec);
		}
	}
#endif

	LTC2949_ProcessErrors(error);

#ifdef LTC2949_DEBUG_WR_REGSCTRL
	LTC2949_DebugEnable = LTC2949_DebugEnable_;
#endif
	return error;
}

// used to simulate PEC errors (single byte with PEC error)
void LTC2949_DcWR_PECerror(byte addr, byte data, byte errorType)
{
	if (LTC2949_DebugEnable)
#ifdef ARDUINO
		Serial.print(F("dw:")); // instead of DW to distinguish from normal write
#else
		printf(F("dw:"));
#endif

	uint16_t pec;

	// init the pec calculator
	pec15_calc_init(&pec);

	// This will guarantee that the LTC2949 isoSPI port is awake.
	LTC2949_wakeup_idle(LTC2949_forceWrRegsCtrl); // just to be on the safe side, in case we force write REGSCTRL we also force isoSPI wakeup pulses

	spiStart();

	/////////////////////////////////////////////////////////////////////////////////////
	// CMD part
	/////////////////////////////////////////////////////////////////////////////////////
	spi_write_calc_pec(&pec, LTC2949_ADDR_CMD);
	pec15_calc(&pec, addr);
	spi_write(errorType == LTC2949_DCWR_ERRORTYPE_CMDADDR ? ~addr : addr);
	if (errorType == LTC2949_DCWR_ERRORTYPE_CMDPEC)
		pec = ~pec; // just invert all bits in case of CMDPEC error
	spi_write_pec(&pec);

	/////////////////////////////////////////////////////////////////////////////////////
	// CMDPEC part
	/////////////////////////////////////////////////////////////////////////////////////
	// write cmd_id (there is no PEC for the cmd_id byte as it has intrinsic checksum)
	byte id = LTC2949_CALC_ID_BYTE(1, false);
	if (errorType == LTC2949_DCWR_ERRORTYPE_ID2)
		id = id ^ (1 << 2); // just invert ID2 which is one of the two checksum bits
	else if (errorType == LTC2949_DCWR_ERRORTYPE_ID5)
		id = id ^ (1 << 5); // just invert ID5 which is one of the two checksum bits
	spi_write(id);

	/////////////////////////////////////////////////////////////////////////////////////
	// DATA part
	/////////////////////////////////////////////////////////////////////////////////////
	pec15_calc_init(&pec);
	pec15_calc(&pec, data);
	// WRITE data
	spi_write(errorType == LTC2949_DCWR_ERRORTYPE_DATA ? ~data : data); // just invert all bits in case of data error
	// WRITE PEC
	if (errorType == LTC2949_DCWR_ERRORTYPE_DATAPEC)
		pec = ~pec; // just invert all bits in case of data error
	spi_write_pec(&pec);

	spiStop();
}

// for indirect broadcast read only!
void _LTC2949_indirect_bcren_wr_pg_addr_(uint16_t ADDR)
{
	LTC2949_iAddrRegsCtrl[LTC2949_RDCVIADDR_IX] = ADDR;

	boolean wrRegsCtrl = false;

	// check if address is not on special ROW
	if (!LTC2949_IS_ADDR_SPECIAL_ROW(ADDR))
	{
		// extract page or address
		boolean page = bitMaskSetChk(ADDR, 0x100);
		// check if we are currently on a different page
		if (bitMaskSetChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_PAGE) != page)
		{
			// select the correct page
			bitMaskSetClr(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_PAGE, page);
			wrRegsCtrl = true;
		}
	}

	// check if RDVCCONF bit is set (for indirect register access this bit must be cleared!)...
	if (bitMaskSetChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_RDCVCONF))
	{
		// clear RDCVCONF bit
		bitMaskClr(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_RDCVCONF);
		wrRegsCtrl = true;
	}

	// check broadcast read enable
	if (bitMaskClrChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_BCREN))
	{
		bitMaskSet(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_BCREN);
		wrRegsCtrl = true;
	}

	if (wrRegsCtrl || LTC2949_forceWrRegsCtrl)
	{
		// write IADDR together with REGSCTRL
		_LTC2949_direct_write_(LTC2949_REG_RDCVIADDR, LTC2949_IADDRREGSCTRL_LEN, LTC2949_iAddrRegsCtrl);
		LTC2949_forceWrRegsCtrl = false;
		return;
	}

	// write only RDCVIADDR
	_LTC2949_direct_write_(LTC2949_REG_RDCVIADDR, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_RDCVIADDR_IX]));
}


// this makes sure, BCREN and RDCVCONF are cleared. This avoids altering LTC2949's HS byte in case some 
// RDCV command is used to read data from cell monitors only
void LTC2949_ClrRdcvConfBcren()
{
	// set indirect address to some special ROW address. Even if BCREN is cleared,
	// LTC2949 will internally behave like register(s) are read.
	// in the end its only important, that the address pointer is not set to some FIFO address, as this would alter the FIFO internally
	LTC2949_iAddrRegsCtrl[LTC2949_RDCVIADDR_IX] = LTC2949_REG_OPCTRL;
	// clear RDCVCONF, BCREN bits
	bitMaskClr(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_RDCVCONF | LTC2949_BM_REGSCTRL_BCREN);
	// write IADDR together with REGSCTRL
	_LTC2949_direct_write_(LTC2949_REG_RDCVIADDR, LTC2949_IADDRREGSCTRL_LEN, LTC2949_iAddrRegsCtrl);
	LTC2949_forceWrRegsCtrl = false;
}

void LTC2949_ReorderData(uint16_t* data, uint8_t len)
{
	if (data == NULL)
		return;
	// re-order data: (depending on endianness of the used MCU this might not be necessary. For Linduino, which is little endian we have to do it)
	for (byte i = 0; i < len; i++)
	{
		byte lsb = ((byte*)data)[i << 1];
		byte msb = ((byte*)data)[(i << 1) + 1];
		data[i] = lsb;
		data[i] |= msb << 8;
	}
	//             LSB                       MSB
	//data[0] = ((byte *)data)[0] | ((byte *)data)[1] << 8; // I1
	//data[1] = ((byte *)data)[2] | ((byte *)data)[3] << 8; // I2
	//data[2] = ((byte *)data)[4] | ((byte *)data)[5] << 8; // BAT
	//data[3] = ((byte *)data)[6] | ((byte *)data)[7] << 8; // AUX
	//data[4] = ((byte *)data)[8] | ((byte *)data)[9] << 8; // TAGs
}

#ifdef LTCDEF_SPI_POLL_SUPPORT
byte LTC2949_RdFastData(int16_t* data, uint16_t* cellMonDat, uint16_t rdcv, uint8_t pollTimeout16Us)
{
	if (pollTimeout16Us)
		// make it a LTC2949 read & poll command (to check for EOC)
		rdcv |= LTC2949_68XX_CMDTYPE_RDPL; // we could also do "rdcv |= LTC2949_68XX_CMDTYPE_PL;" which is equivalent, as LTC2949_68XX_CMDTYPE_RD bit must always be set when calling this function
#else
byte LTC2949_RdFastData(int16_t * data, uint16_t * cellMonDat, uint16_t rdcv)
{
#endif
	byte error;
	if (LTC2949_onTopOfDaisychain)
	{
		// make sure RDCVCONF is set
		// make sure BCREN is set
		if (LTC2949_forceWrRegsCtrl || !bitMaskSetChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_RDCVCONF | LTC2949_BM_REGSCTRL_BCREN))
		{
			// set BCREN,RDCVCONF bits
			bitMaskSet(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_BCREN | LTC2949_BM_REGSCTRL_RDCVCONF);
			_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX])); // write REGSCTRL
			LTC2949_forceWrRegsCtrl = false;
		}

		error = LTC2949_68XX_CMD(
			/*uint8_t addr:            */ 0, // must be broadcast address!!!
			/*uint16_t cmd:            */ rdcv,
			/*byte *data:              */ (byte*)cellMonDat,
			/*uint16_t len:            */ LTC2949_CellMonitorCount * 6,
			/*byte *data2949:          */ (byte*)data,
			/*uint16_t len2949:        */ LTC2949_RDFASTDATA_LENGTH << 1
#ifdef LTCDEF_SPI_POLL_SUPPORT
			,/*uint8_t pollTimeout16Us: */ pollTimeout16Us
#else
#endif
		);
		// if (cellMonDat) is checked inside LTC2949_ReorderData
		LTC2949_ReorderData(cellMonDat, LTC2949_CellMonitorCount * 3);
	}
	else
	{
		// make sure RDCVCONF is set
		// make sure BCREN is cleared
		if (LTC2949_forceWrRegsCtrl ||
			bitMaskClrChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_RDCVCONF) ||
			(bitMaskSetChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_BCREN)))
		{
			// set RDCVCONF bit
			bitMaskSet(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_RDCVCONF);
			// clr BCREN bit
			bitMaskClr(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_BCREN);
			_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX])); // write REGSCTRL
			LTC2949_forceWrRegsCtrl = false;
		}
		error = LTC2949_68XX_CMD(
			/*uint8_t addr:            */ LTC2949_ADDRESS, // must be addressed!!!
			/*uint16_t cmd:            */ rdcv,
			/*byte *data:              */ (byte*)data,
			/*uint16_t len:            */ LTC2949_RDFASTDATA_LENGTH << 1,
			/*byte *data2949:          */ NULL,
			/*uint16_t len2949:        */ 0
#ifdef LTCDEF_SPI_POLL_SUPPORT
			,/*uint8_t pollTimeout16Us: */ pollTimeout16Us
#else
#endif
		);
	}

	// re-order data: (depending on endianness of the used MCU this might not be necessary. For Linduino, which is little endian we have to do it)
	LTC2949_ReorderData((uint16_t*)data, LTC2949_RDFASTDATA_LENGTH);
	return error;
}

#ifdef ARDUINO
byte LTC2949_PollFastData(int16_t * data, boolean clrChkHS, uint16_t * cellMonDat, uint16_t rdcv)
{
	// allocate buffer for fast data if necessary
	boolean dataBufferAllocated = (data == NULL);
	if (dataBufferAllocated)
		data = new int16_t[LTC2949_RDFASTDATA_LENGTH];

	// poll LTC2949 for conversion done
	byte error = LTC2949_RdFastData(
		data,
		cellMonDat,
		rdcv,
		LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_FASTSSHT_RDY_TIME_US)
	);

	// check if we already read valid data from LTC2949
	if (LTC2949_FASTSSHT_HS_OK(data))
	{
		// all fine, nothing to do.
		// still we might need to ensure the HS byte is cleared after results were read successfully (see safety measures)
		if (clrChkHS)
		{
			if (!dataBufferAllocated)
			{
				// create new buffer. This avoid that the original buffer, that already holds good conversions results and good HS bytes, is overwritten.
				// still, as this only makes sense for fast single shot we can even read the data again, which must be the same data. 
				// For sure we need to read the HS byte to check it is cleared
				data = new int16_t[LTC2949_RDFASTDATA_LENGTH];
				dataBufferAllocated = true;
			}
			error |= LTC2949_RdFastData(data, cellMonDat, rdcv);
			if (!LTC2949_FASTSSHT_HS_CLR(data)) // for sure HS bytes must be cleared now (this only makes sense for fast single shot measurements! In CONT mode this may fail!)
				error |= LTC2949_ERRCODE_OTHER;
		}
	}
	else if (LTC2949_FASTSSHT_HS_CLR(data))
	{
		// we polled for HS==0x0F before, so it cannot be all HS are zero! something went wrong (e.g. timeout)
		error |= LTC2949_ERRCODE_OTHER;
	}
	else if (LTC2949_FASTSSHT_HS_LAST_OK(data)) // first HS != 0x0F, last HS == 0x0F
	{
		// normally here we would have to read fast data again, to get the actual conversion result.
		// still if dataBufferAllocated == true (data == NULL, e.g. caller is not interested in the actual data)  and we do not read again.
		// this is the case e.g. when LTC2949_PollFastData is called after enable of fast continuous, e.g. to poll for end of first conversion
		if (!dataBufferAllocated)
		{
#ifdef LTC2949_HS_LAST_DBG
			int16_t hsBytes = data[LTC2949_RDFASTDATA_HS];
#endif
			// we have to read data from LTC2949 again, as only the next RDCVx will report the final conversion results
			// also cell voltages will be read again
			// note: here we must not poll HS! (it must be zero now!)
			error |= LTC2949_RdFastData(data, cellMonDat, rdcv);

			if (!LTC2949_FASTSSHT_HS_CLR(data)) // HS must be cleared now
			{
				error |= LTC2949_ERRCODE_OTHER; // this must never happen in case of fast single shot events
#ifdef LTC2949_HS_LAST_DBG
				Serial.println(hsBytes, HEX);
#endif
			}
			else
				data[LTC2949_RDFASTDATA_HS] = LTC2949_RDFASTDATA_HS_OKOK; // set HS to the good case, meaning what we return is what would be if read OKOK already initially.
		}
	}
	else
	{
		// Unexpected HS bytes, something went wrong
		error |= LTC2949_ERRCODE_OTHER;
	}
	if (dataBufferAllocated)
		delete[] data;
	return error;
}
#else
byte LTC2949_PollFastData(int16_t * data, boolean clrChkHS, uint16_t * cellMonDat, uint16_t rdcv)
{
	// allocate buffer for fast data if necessary
	boolean dataBufferAllocated = (data == NULL);
	if (dataBufferAllocated)
		data = new int16_t[LTC2949_RDFASTDATA_LENGTH];

	byte error = 0;

	unsigned long pollTimeout;
	bool started = false;

	while (true)
	{
		// poll LTC2949 for conversion done
		error |= LTC2949_RdFastData(
			data,
			cellMonDat,
			rdcv
		);
		if (!started)
		{
			pollTimeout = micros() + 1200;
			started = true;
		}
		if (error & (byte)(~LTC2949_ERRCODE_TIMEOUT))
			break;

		// check if we already read valid data from LTC2949
		if (LTC2949_FASTSSHT_HS_OK(data))
		{
			// all fine, nothing to do.
			// still we might need to ensure the HS byte is cleared after results were read successfully (see safety measures)
			if (clrChkHS)
			{
				if (!dataBufferAllocated)
				{
					// create new buffer. This avoid that the original buffer, that already holds good conversions results and good HS bytes, is overwritten.
					// still, as this only makes sense for fast single shot we can even read the data again, which must be the same data. 
					// For sure we need to read the HS byte to check it is cleared
					data = new int16_t[LTC2949_RDFASTDATA_LENGTH];
					dataBufferAllocated = true;
				}
				error |= LTC2949_RdFastData(data, cellMonDat, rdcv);
				if (!LTC2949_FASTSSHT_HS_CLR(data)) // for sure HS bytes must be cleared now (this only makes sense for fast single shot measurements! In CONT mode this may fail!)
					error |= LTC2949_ERRCODE_OTHER;
			}
			bitMaskClr(error, LTC2949_ERRCODE_TIMEOUT);
			break;
		}
		else if (LTC2949_FASTSSHT_HS_LAST_OK(data)) // first HS != 0x0F, last HS == 0x0F
		{
			// normally here we would have to read fast data again, to get the actual conversion result.
			// still if dataBufferAllocated == true (data == NULL, e.g. caller is not interested in the actual data)  and we do not read again.
			// this is the case e.g. when LTC2949_PollFastData is called after enable of fast continuous, e.g. to poll for end of first conversion
			if (!dataBufferAllocated)
			{
				// we have to read data from LTC2949 again, as only the next RDCVx will report the final conversion results
				// also cell voltages will be read again
				// note: here we must not poll HS! (it must be zero now!)
				error |= LTC2949_RdFastData(data, cellMonDat, rdcv);

				if (!LTC2949_FASTSSHT_HS_CLR(data)) // HS must be cleared now
					error |= LTC2949_ERRCODE_OTHER; // this must never happen in case of fast single shot events
				else
					data[LTC2949_RDFASTDATA_HS] = LTC2949_RDFASTDATA_HS_OKOK; // set HS to the good case, meaning what we return is what would be if read OKOK already initially.
			}
			bitMaskClr(error, LTC2949_ERRCODE_TIMEOUT);
			break;
		}
		else if (LTC2949_FASTSSHT_HS_CLR(data))
		{
			if (LTC_TIMEOUT_CHECK(micros(), pollTimeout))
			{
				if (bitMaskSetChk(error, LTC2949_ERRCODE_TIMEOUT))
					break;
				error |= LTC2949_ERRCODE_TIMEOUT;
			}
			// read again
			continue;
		}
		else
		{
			// Unexpected HS bytes, something went wrong
			error |= LTC2949_ERRCODE_OTHER;
			break;
		}
	}
	if (dataBufferAllocated)
		delete[] data;
	return error;
}
#endif
// only switch page
//     if not special row
//     if current selected page is different to page of ADDR
void _LTC2949_ADJ_REGSCTRL_DC_(uint16_t ADDR, boolean readNotWrite)
{
	boolean wrRegsCtrl = false;
	// check page
	if ((!LTC2949_IS_ADDR_SPECIAL_ROW(ADDR)) &&
		(bitMaskSetChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_PAGE) != bitMaskSetChk(ADDR, 0x100)))
	{
		bitMaskSetClr(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_PAGE, bitMaskSetChk(ADDR, 0x100));
		wrRegsCtrl = true;
	}
	// check broadcast read enable
	if (readNotWrite && LTC2949_onTopOfDaisychain && bitMaskClrChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_BCREN))
	{
		bitMaskSet(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_BCREN);
		wrRegsCtrl = true;
	}
	if (wrRegsCtrl || LTC2949_forceWrRegsCtrl)
	{
		_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX]));
		LTC2949_forceWrRegsCtrl = false;
	}
}

void LTC2949_AdjustRegsCtrlBuffer(byte addr, uint16_t len, byte * data);

byte LTC2949_READ(uint16_t ADDR, uint16_t LEN, byte * DATAPOINTER)
{
	byte error;
	if (LTC2949_onTopOfDaisychain)
	{
		_LTC2949_indirect_bcren_wr_pg_addr_(ADDR);
		error = LTC2949_68XX_CMD(
			/*uint8_t addr:       */ 0, // daisychain is always broadcast
			/*uint16_t cmd:       */ LTC2949_68XX_CMD_RDCVA,
			/*byte *data:         */ NULL, // data from cell monitors is ignored here!
			/*uint16_t len:       */ LTC2949_CellMonitorCount * 6,
			/*byte *data2949:     */ DATAPOINTER,
			/*uint16_t len2949:   */ LEN
		);
	}
	else
	{
		_LTC2949_ADJ_REGSCTRL_DC_(ADDR, LTC2949_DcRead);
		error = _LTC2949_DcRW_(ADDR, LEN, DATAPOINTER, LTC2949_DcRead);
	}

	if (!error)
		LTC2949_AdjustRegsCtrlBuffer(ADDR, LEN, DATAPOINTER);
	return error;
}

void LTC2949_WRITE(uint16_t ADDR, uint16_t LEN, byte * DATAPOINTER)
{
	_LTC2949_ADJ_REGSCTRL_DC_(ADDR, LTC2949_DcWrite);
	_LTC2949_direct_write_(ADDR, LEN, DATAPOINTER);
	LTC2949_AdjustRegsCtrlBuffer(ADDR, LEN, DATAPOINTER);
}

byte LTC2949_WakeupAndAck()
{
	byte error;
	// write the default of REGSCTRL (Note BCREN will be automatically adjusted by following poll command if necessary!)
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] = LTC2949_BM_REGSCTRL_RDCVCONF;
	// this will force write of REGSCTRL with above value in any case
	LTC2949_forceWrRegsCtrl = true;
	// let's try to read
	LTC2949_wakeup_idle(true);
	// poll OPCTRL
	// Note: LTC2949_PollReg will always write REGSCTRL in case of indirect commands or in case of PEC error 
	// (thus BREN and RDCVCONF will always be adjusted according to communication mode)
	error = LTC2949_PollReg(
		/*uint16_t addr,          */ LTC2949_REG_OPCTRL,
		/*byte val,               */ 0, // check for SLEEP bit cleared
		/*byte mask,              */ LTC2949_BM_OPCTRL_SLEEP,  // only check SLEEP bit, ignore all other bits
		/*uint16_t timeout,       */ LTC2949_TIMING_BOOTUP,
		/*boolean stopOnPecErrors,*/ false
	);
	LTC2949_WRITE(LTC2949_REG_WKUPACK, 0x00); // write wake up acknowledge
	return error;
	/////// //not necessary, see below....
	/////// //        _LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, &(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX]));
	/////// 
	/////// // this will force write of REGSCTRL with above value
	/////// LTC2949_forceWrRegsCtrl = true;
	/////// 
	/////// // let's try to read
	/////// LTC2949_wakeup_idle(true);
	/////// byte opctrl;
	/////// error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &opctrl);
	/////// 
	/////// if ((error != 0) || bitMaskSetChk(opctrl, LTC2949_BM_OPCTRL_SLEEP))
	/////// {
	/////// 	LTC2949_forceWrRegsCtrl = true;
	/////// 	delay(LTC2949_TIMING_START_POLL_CORE_RDY); // give LTC2949 enough time to wake-up
	/////// 	LTC2949_wakeup_idle(true);	// again, as isoSPI port is idle again!
	/////// 
	/////// 	// poll REGSCTRL
	/////// 	// Note: LTC2949_PollReg will always write REGSCTRL in case of indirect commands or in case of PEC error (e.g. broadcast command, but BREN not set) or in case not reading expected value
	/////// 	// byte error = LTC2949_PollReg(
	/////// 	// 	/*uint16_t addr,          */ LTC2949_REG_REGSCTRL,
	/////// 	// 	/*byte val,               */ LTC2949_onTopOfDaisychain ? LTC2949_BM_REGSCTRL_BCREN : LTC2949_BM_REGSCTRL_RDCVCONF,
	/////// 	// 	/*byte mask,              */ 0xFF,
	/////// 	// 	/*uint16_t timeout,       */ LTC2949_TIMING_BOOTUP,
	/////// 	// 	/*boolean stopOnPecErrors,*/ false
	/////// 	// );
	/////// 
	/////// 	// poll OPCTRL
	/////// 	// Note: LTC2949_PollReg will always write REGSCTRL in case of indirect commands or in case of PEC error (e.g. broadcast command, but BREN not set)
	/////// 	error = LTC2949_PollReg(
	/////// 		/*uint16_t addr,          */ LTC2949_REG_OPCTRL,
	/////// 		/*byte val,               */ 0, // check for SLEEP bit cleared
	/////// 		/*byte mask,              */ LTC2949_BM_OPCTRL_SLEEP,  // only check SLEEP bit, ignore all other bits
	/////// 		/*uint16_t timeout,       */ LTC2949_TIMING_BOOTUP,
	/////// 		/*boolean stopOnPecErrors,*/ false
	/////// 	);
	/////// 	// Note: during power-up / LTC2949 still sleeping,
	/////// 	// read from any register will return 0x01, 
	/////// 	// which in case of register OPCTRL can be interpreted
	/////// 	// as SLEEP bit being set.
	/////// }
	/////// 
	/////// LTC2949_WRITE(LTC2949_REG_WKUPACK, 0x00); // write wake up acknowledge
	/////// return error;
}

byte LTC2949_OpctlAdjUpd(boolean clear)
{
	// Adjust update must always be done in IDLE mode!
	// Thus if we were in continuous mode the CONT bit will be cleared
	byte data = LTC2949_BM_OPCTRL_ADJUPD;
	if (clear)
	{
		data |= LTC2949_BM_OPCTRL_CLR;
		LTC2949_lastTBx = 0;
	}
	// note: every time we write to special row, we can directly use _LTC2949_direct_write_ if we make sure to update LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] in case we write LTC2949_REG_REGSCTRL
	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, 1, &data);
	delay(1);
	return LTC2949_PollReg(LTC2949_REG_OPCTRL, 0);
}

// true: TBx was updated
boolean LTC2949_ChkUpdate(byte * error, unsigned long* ms, byte tbAddr)
{
	uint32_t tbx;
	if (ms != NULL)
		*ms = millis(); // store the timestamp when the check was done

	// here we don't care about little or big endianness
	// as we just check for equal / unequal.
	*error = LTC2949_READ(tbAddr, 4, (byte*)&tbx);
	if (*error)
		return false;
	if (tbx == LTC2949_lastTBx)
		return false; // tbx still not changed
	// tbx changed
	LTC2949_lastTBx = tbx;
	return true;
}

double LTC2949_GetLastTBxDbl()
{
	return LTC2949_BytesToDouble((byte*)(&LTC2949_lastTBx), 4, false, LTC2949_LSB_TB1);
}

uint32_t LTC2949_GetLastTBxInt()
{
	return LTC_4BytesToUInt32((byte*)(&LTC2949_lastTBx));
}

byte LTC2949_PollUpdate(unsigned long* measStart, byte tbAddr)
{
	byte error = 0;
	boolean timeoutInitDone = false;
	unsigned long _measStart = 0;
	unsigned long _timeout;

	if (!measStart)
		measStart = &_measStart;

	while (true)
	{
		byte err;
		if (LTC2949_ChkUpdate(&err, measStart, tbAddr))
		{
			error |= err;
			error &= ~LTC2949_ERRCODE_TIMEOUT;
			break;
		}
		error |= err;
		if (error)
			break; // timeout or communication error

		if (!timeoutInitDone)
		{
			timeoutInitDone = true;
			_timeout = *measStart + LTC2949_TIMING_IDLE2CONT2UPDATE;
		}
		else if (LTC_TIMEOUT_CHECK(millis(), _timeout))
		{
			error |= LTC2949_ERRCODE_TIMEOUT;
		}
		if (LTC2949_DebugEnable)
			delay(20);
	}
	return error;
}

byte LTC2949_ReadFastCfg(boolean * ch1, boolean * ch2, boolean * pasv, boolean * aux, boolean * cont)
{
	byte data;
	byte error = LTC2949_READ(LTC2949_REG_FACTRL, 1, &data);

	*ch1 = bitMaskSetChk(data, LTC2949_BM_FACTRL_FACH1);
	*ch2 = bitMaskSetChk(data, LTC2949_BM_FACTRL_FACH2);
	*aux = bitMaskSetChk(data, LTC2949_BM_FACTRL_FACHA);
	*cont = bitMaskSetChk(data, LTC2949_BM_FACTRL_FACONV);

	// read PasV configuration
	error |= LTC2949_READ(LTC2949_REG_ADCCONF, 1, &data);

	if (!error)
		LTC2949_adcConf = data;

	//LTC2949_ADCConfigUpdate(data);

// check if any PasV is enabled
// NOTE: there are some settings that are not allowed. E.g. CH1 slow, CH2 fast and both PasV set: In this case P1asV would have priority for fast conv. but CH1 must be slow!
	if (!bitMaskClrChk(data, LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2))
		*pasv = !bitMaskClrChk(data, LTC2949_BM_ADCCONF_P1ASV | LTC2949_BM_ADCCONF_P2ASV);
	else
		*pasv = false;

	return error;
}

void LTC2949_WriteFastCfg(boolean ch1, boolean ch2, boolean aux, boolean cont)
{
	byte data = (cont ? LTC2949_BM_FACTRL_FACONV : 0) |
		(ch1 ? LTC2949_BM_FACTRL_FACH1 : 0) |
		(ch2 ? LTC2949_BM_FACTRL_FACH2 : 0) |
		(aux ? LTC2949_BM_FACTRL_FACHA : 0);
	// note: every time we write to special row, we can directly use _LTC2949_direct_write_ if we make sure to update LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] in case we write LTC2949_REG_REGSCTRL
	_LTC2949_direct_write_(LTC2949_REG_FACTRL, 1, &data);
}

void LTC2949_WriteFastCfg(byte cfgFast)
{
	cfgFast &= LTC2949_BM_FACTRL_FACONV | LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA;
	// note: every time we write to special row, we can directly use _LTC2949_direct_write_ if we make sure to update LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] in case we write LTC2949_REG_REGSCTRL
	_LTC2949_direct_write_(LTC2949_REG_FACTRL, 1, &cfgFast);
}

byte LTC2949_GoCont(byte cfgFast, byte adcCfg, unsigned long* startOfMeas, bool clrAccus)
{
	byte data;
	byte error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &data); // check current operation mode
	if (bitMaskSetChk(data, LTC2949_BM_OPCTRL_CONT))
	{
		// clear of FACTRL is not necessary (done automatically internally)
		// LTC2949_WriteFastCfg(0);     // clear FACTRL
		// we are in CONT mode, so go IDLE
		LTC2949_OpctlIdle();       // clear OPCTRL
	}

	// configure ADC (PasV, NTC...)
	LTC2949_ADCConfigAll(adcCfg);
	// write ADC config
	LTC2949_ADCConfigWrite();
	// clear prev. measurement and make adjust update
	error |= LTC2949_OpctlAdjUpd(clrAccus);

	// read & clear status
	// error |= LTC2949_ReadChkStatusFaults(true, reportFaults);

	if (!clrAccus)
	{
		LTC2949_READ(LTC2949_VAL_TB1, 4, (byte*)&LTC2949_lastTBx);
	}

	unsigned long _startOfMeas;
	// start cont. measurement
	LTC2949_OpctlCont(); // write CONT to OPCTRL
	if (error)
		return error;
	// poll for start of measurement (TB1/2/3/4 != 0)
	error = LTC2949_PollUpdate(&_startOfMeas);
	if (error)
		return error;

	// one measurement usually takes 100 ms. So the actual
	// start of measurement was 100 ms before the current millis value.
	_startOfMeas -= 100;

	if (startOfMeas != NULL)
		*startOfMeas = _startOfMeas;

	// enable fast continuous or single shot
	LTC2949_WriteFastCfg(cfgFast); // write FACTRL

	return error;
}

void LTC2949_ReportFaults(byte * dataBuffer, byte len)
{
#ifdef ARDUINO
	Serial.print(F("STAT:"));
#else
	printf(F("STAT:"));
#endif
	SerialPrintByteArrayHex(dataBuffer, len, true);
}

byte LTC2949_EEPROMWrite()
{
	LTC2949_OpctlIdle();

	if (byte error = LTC2949_EEPROMIsReady())
		return error;

	if (byte error = LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_INIT))
		return error;

	if (byte error = LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_CHECK))
		return error;

	if (byte error = LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_SAVE))
		return error;

	return 0;
}

byte LTC2949_EEPROMInitCheck()
{
	LTC2949_OpctlIdle();

	if (byte error = LTC2949_EEPROMIsReady())
		return error;

	if (byte error = LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_INIT))
		return error;

	if (byte error = LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_CHECK))
		return error;

	return 0;
}

byte LTC2949_EEPROMRead()
{
	LTC2949_OpctlIdle();

	if (byte error = LTC2949_EEPROMIsReady())
		return error;

	if (byte error = LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_CHECK))
		return error;

	if (byte error = LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_RESTORE))
		return error;

	return 0;
}

byte LTC2949_EEPROMIsReady()
{
	// check that all SO bits are reset
	byte data = 0;
	byte cnt = 80; // max time 400ms to read all SO bits being zero
	byte error;

	if ((error = LTC2949_READ(LTC2949_REG_EEPROM, 1, &data)))
		return error;

	if (data)
	{
		data = 0;
		LTC2949_WRITE(LTC2949_REG_EEPROM, 1, &data);
		delay(5);
	}

	do
	{
		error |= LTC2949_READ(LTC2949_REG_EEPROM, 1, &data);

		if (error)
			return error;

		data &= (LTC2949_BM_EEPROM_CHECK | LTC2949_BM_EEPROM_INIT | LTC2949_BM_EEPROM_RESTORE | LTC2949_BM_EEPROM_SAVE);

		if (data == 0)
			return 0;

		if (cnt == 0)
			return LTC2949_ERRCODE_TIMEOUT; // timeout

		cnt--;
		delay(5);
	} while (true);
}

byte LTC2949_EEPROMCommand(byte command)
{
	byte resultBit = command << 4; // the result bit is always 4 bits above the command bit
	byte data = command;
	uint16_t cnt = 2000; // max time 2000*0.001 = 2 seconds to complete command
	byte error = 0;

	LTC2949_WRITE(LTC2949_REG_EEPROM, 1, &data);

	do
	{
		error |= LTC2949_READ(LTC2949_REG_EEPROM, 1, &data);

		if (error)
			return error;

		data &= command | resultBit;

		if (data == resultBit)
			return 0;

		if (!cnt)
			return LTC2949_ERRCODE_TIMEOUT; // timeout

		cnt--;
		delay(1);
	} while (true);
}

// Note on LTC2949 reset:
//  as default the write to RESET bit in OPCTRL is locked
//  write of LTC2949_CODE_RSTUNLCK to OPCTRL unlocks the reset
//    the above write is protected by the PEC of the data of the write command
//    unlock command is processed by LTC2949's core within its cycle time
//    after unlock is processed the RESET bit in OPCTRL is unlocked
//    if unlocked, the RESET bit is NOT protected by the PEC of the data of the write command
// If possible it is recommended to use a power-cycle to reset LTC2949
// If not possible the reset may be unlocked at the very beginning of LTC2949's initialization 
// at the risk that communication errors may trigger an unintended reset
//   such a unintended reset is easily recognized by reading STATUS reg periodically (anyway recommended!)
// Unlocking reset only on-demand may have the risk that it is not possible recovering from a stuck FW
// (e.g. due to single event upset), as unlocking requires FW-core interaction
//
// make power cycle instead of reset if you can do it
// always unlock reset if you intend to use it (as unlock doesn't work if FW is stuck)
// RESET bit is not protected, but you can identify unintended reset
// timing profiling document
void LTC2949_reset_()
{
	byte data = LTC2949_BM_OPCTRL_CONT, error = 0;

	// check for operation mode (IDLE or CONT)
	// NOTE: This only works if any write to OPCTRL, that cleared CONT, did not happen in the last 100 ms. 
	//       Otherwise the cleared CONT is wrongly interpreted as already being in IDLE mode
	if (false) // DISABLED, see note above! As default we expect to be in CONT mode, see initialization of variable data above
		error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &data);

	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, 0); // force IDLE
	LTC2949_WRITE(LTC2949_REG_RSTUNLCK, LTC2949_CODE_RSTUNLCK); // unlock reset
	// Note: Both commands above are effective latest 100 (+10%) ms later, see following delay
	if ((error == 0) && bitMaskSetChk(data, LTC2949_BM_OPCTRL_CONT))
		delay(LTC2949_TIMING_CONT_CYCLE); // in case we were in CONT mode
	else
		delay(LTC2949_TIMING_IDLE_CYCLE); // in case we were in IDLE mode

	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_SLEEP); // go to sleep
	// wait for sleep
	delay(LTC2949_TIMING_IDLE_CYCLE); // we are in IDLE mode for sure!
	// now we are ready to make a reset
	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_RST);

	LTC2949_init_device_state();
}

void LTC2949_reset()
{
	LTC2949_WRITE(LTC2949_REG_OPCTRL, 0); // force STANDBY
	// write unlock reset on 2nd page.
	// note that LTC2949_WRITE(... also changes the page if necessary.
	LTC2949_WRITE(LTC2949_REG_RSTUNLCK, LTC2949_CODE_RSTUNLCK); // write 0x55 to RSTUNLCK register on 2nd page
	delay(LTC2949_TIMING_CONT_CYCLE); // delay 105 ms
	LTC2949_WRITE(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_SLEEP); // go to sleep
	delay(LTC2949_TIMING_IDLE_CYCLE); // wait 19 ms for sleep
	// now we are ready to make a reset
	LTC2949_WRITE(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_RST); // make the reset
	// DONE
	// 
	// now LTC2949 can be woken up again and STATUS register must report 0x0F
	// any other value means there was no reset (e.g. due to communication error)
	// and the reset has to be repeated
	LTC2949_init_device_state();
	delayMicroseconds(1);
}


//byte LTC2949_ADCConfigGetBuffer()
//{
//	return LTC2949_adcConf;
//}

void LTC2949_ADCConfigAll(byte adcCfg)
{
	bitsMaskApplyToReg(LTC2949_adcConf, adcCfg, LTC2949_BM_ADCCONF_P1ASV | LTC2949_BM_ADCCONF_P2ASV | LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_NTC2 | LTC2949_BM_ADCCONF_NTCSLOT1);
}

void LTC2949_ADCConfigPasV(byte pasvCfg)
{
	bitsMaskApplyToReg(LTC2949_adcConf, pasvCfg, LTC2949_BM_ADCCONF_P1ASV | LTC2949_BM_ADCCONF_P2ASV);
}

void LTC2949_ADCConfigNTC(byte ntcCfg)
{
	bitsMaskApplyToReg(LTC2949_adcConf, ntcCfg, LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_NTC2 /*| LTC2949_BM_ADCCONF_NTCAVCC*/);
}

void LTC2949_ADCConfigTC(byte tcCfg)
{
	bitsMaskApplyToReg(LTC2949_adcConf, tcCfg, LTC2949_BM_ADCCONF_NTCSLOT1);
}

void LTC2949_ADCConfigWrite()
{
	LTC2949_WRITE(LTC2949_REG_ADCCONF, LTC2949_adcConf); // LTC2949_OpctlAdjUpd() should be called by user afterwards
	//LTC2949_ADCConfigUpdate(LTC2949_adcConf);
}

byte LTC2949_ADCConfigRead(byte * data)
{
	byte error = LTC2949_READ(LTC2949_REG_ADCCONF, 1, data);
	if (!error)
		LTC2949_adcConf = *data;
	return error;
}

boolean LTC2949_Abs(byte * bytes, byte length)
{
	if (bitMaskClrChk(*bytes, 0x80))
		return false;// value is already positive

	// two's complement is generated by inverting all bits and add 1

	length--;
	bytes += length; // seek to LSB
	uint16_t cHelp = (~(*bytes)) & 0xFF; // invert LSB
	cHelp++; // add 1
	*bytes = cHelp; // store back to buffer

	while (length != 0)
	{
		// seek next byte (towards MSB)
		length--;
		bytes--;
		cHelp = cHelp >> 8; // restore carry from previous sum
		cHelp += (~(*bytes)) & 0xFF; // add inverted byte
		*bytes = cHelp; // store back
	}
	return true;// value inverted
}

double LTC2949_BytesToDouble(byte * bytes, byte length, boolean sig, double lsb)
{
	if (length == 0)
		return 0.0;
	else if (length == 1)
		return sig ? (int8_t)(bytes[0]) * lsb : bytes[0] * lsb;
	else if (length == 2)
		return sig ? LTC_2BytesToInt16(bytes) * lsb : LTC_2BytesToUInt16(bytes) * lsb;
	else if (length == 3)
		return sig ? LTC_3BytesToInt32(bytes) * lsb : LTC_3BytesToUInt32(bytes) * lsb;
	else if (length == 4)
		return sig ? LTC_4BytesToInt32(bytes) * lsb : LTC_4BytesToUInt32(bytes) * lsb;
	else
		return sig ? LTC2949_SignedBytesToDouble(bytes, length, lsb) : LTC2949_UnsignedBytesToDouble(bytes, length, lsb);
}

double LTC2949_UnsignedBytesToDouble(byte * unsignedBytes, byte length, double lsb)
{
	// NOTE: On Arduino double is 32-bit and NOT 64-bit, thus the returned value
	// will not reflect the full precission of e.g. C1, C2... which are 48-bit values
	double ret = (*unsignedBytes); // MSB!

	while (length > 1)
	{
		unsignedBytes++; // go to next byte
		length--;
		ret = ret * 256.0 + (*unsignedBytes);
	}
	return ret * lsb;
}

double LTC2949_SignedBytesToDouble(byte * signedBytes, byte length, double lsb)
{
	// reserve memory for unsigned bytes
	byte* unsignedBytes = new byte[length];
	// copy signed bytes to unsigned bytes
	memcpy(unsignedBytes, signedBytes, length);
	// calculate absolute value of the signed bytes and store sign
	// this function will change the unsigned bytes, for this reason
	// we copied the original unsigned bytes to a new array
	boolean sign = LTC2949_Abs(unsignedBytes, length);
	// convert the unsigned bytes to a double value
	double absDouble = LTC2949_UnsignedBytesToDouble(unsignedBytes, length, lsb);
	// free the allocated memory of the copied array
	delete[] unsignedBytes;
	// recover the previously stored sign to return a signed value
	return sign ? -absDouble : absDouble;
}

void LTC2949_FloatToF24Bytes(float f32, byte * bytes)
{
	// In this function the mantissa of the F24 format is determined
	// by cutting 16 bits out of the 23 bit mantissa of F32 format.
	// This is a truncation and not the exact correct rounding.
	// For correct rounding we should use bit 7 of F32's mantissa 
	// to check if we have to increase or decrease the mantissa 
	// of F24 by one.
	// The precision of F24 bit format is 1/2^17 = 0.00076 %. 
	// Truncation is equal to having one bit less precision
	// 1/2^16 = 0.0015 %. This error is still much smaller
	// than any error spec of LTC2949 and far away from typical
	// application precision requirements. Thus, there is no 
	// need to do the correct rounding here.

	if (f32 == 0)
	{
		bytes[0] = 0x00;
		bytes[1] = 0x00;
		bytes[2] = 0x00;
		return;
	}

	// get access to the float as a byte-array:
	byte* dataF32 = (byte*)&f32;

	// extract F32 exponent (8 bits)
	bytes[0] = (dataF32[MCU_FLOAT_23TO16] >> 7) & 0x1;
	bytes[0] |= dataF32[MCU_FLOAT_31TO24] << 1;

	// remove bias from F32 exponent
	// Note: this will fail for big/small exponents outside a 
	// certain range but there is nothing we can do about that,
	// its the user's responsibility that the input float is
	// in the valid range
	bytes[0] += LTC2949_F24TOF32EXPDIFF;

	// store sign to bit 23
	bytes[0] |= dataF32[MCU_FLOAT_31TO24] & 0x80;

	// store F32's mantissa bits 22..15 to F24's mantissa bits 15..8
	bytes[1] = (dataF32[MCU_FLOAT_15TO8] >> 7) & 0x1;
	bytes[1] |= dataF32[MCU_FLOAT_23TO16] << 1;

	// store F32's mantissa bits 14..7 to F24's mantissa bits 7..0
	bytes[2] = (dataF32[MCU_FLOAT_7TO0] >> 7) & 0x1;
	bytes[2] |= dataF32[MCU_FLOAT_15TO8] << 1;
}

void LTC2949_F24BytesToFloat(byte * bytes, float* f32)
{
	// get access to the float as a byte-array:
	byte* dataF32 = (byte*)f32;

	// extract F24's exponent (7bit)
	dataF32[MCU_FLOAT_31TO24] = bytes[0] & 0x7F;
	if (dataF32[MCU_FLOAT_31TO24])
		// convert exponent bias from F24 to F32 (only for non-zero exponents, to keep the special case of exp zero)
		dataF32[MCU_FLOAT_31TO24] += LTC2949_F32TOF24EXPDIFF;

	// store 8 bit exponent to bits 30..23
	dataF32[MCU_FLOAT_23TO16] = (dataF32[MCU_FLOAT_31TO24] << 7);
	dataF32[MCU_FLOAT_31TO24] = (dataF32[MCU_FLOAT_31TO24] >> 1) & 0x7f;

	// store sign
	dataF32[MCU_FLOAT_31TO24] |= bytes[0] & 0x80;

	// store F24's mantissa bits 15..9 to F32's mantissa bits 22..16
	dataF32[MCU_FLOAT_23TO16] |= (bytes[1] >> 1) & 0x7f;
	// store F24's mantissa bit 8 to F32's mantissa bit 15
	dataF32[MCU_FLOAT_15TO8] = bytes[1] << 7;

	// store F24's mantissa bit 7..1 to F32's mantissa bit 14..8
	dataF32[MCU_FLOAT_15TO8] |= (bytes[2] >> 1) & 0x7F;
	// store F24's mantissa bit 0 to F32's mantissa bit 7
	dataF32[MCU_FLOAT_7TO0] = bytes[2] << 7;
}

byte LTC2949_PollReg(
	uint16_t addr,
	byte val,
	byte mask,
	uint16_t timeout,
	boolean stopOnPecErrors,
	byte * finalRead
)
{
	byte ret;
	boolean stop = false;

	if (finalRead == NULL)
		finalRead = &ret;

	unsigned long endTime = millis() + timeout;

	do
	{
		if (byte error = LTC2949_READ(addr, 1, finalRead))
		{
			if (stopOnPecErrors)
				return error;
		}
		else if ((finalRead[0] & mask) == val)
			return 0;

		if (stop)
			return LTC2949_ERRCODE_TIMEOUT; // timeout

		if (!LTC2949_IS_ADDR_SPECIAL_ROW(addr))
			LTC2949_forceWrRegsCtrl = true; // if polling fails and we are not reading from special row, we force write of REGSCTRL. 

		if (LTC_TIMEOUT_CHECK(millis(), endTime))
			stop = true; // timeout, but we try one more time!

		if (LTC2949_DebugEnable)
			delay(20);

	} while (true);
}

/*!****************************************************
  \brief Wake isoSPI up from idle state
  Generic wakeup commannd to wake isoSPI up out of idle
  *****************************************************/
void LTC2949_wakeup_idle(boolean force)
{
#ifdef ARDUINO
	if (!force && !LTC_TIMEOUT_CHECK(micros(), LTC2949_isoSPI_wakeup_timeoutUs))
		return; // isoSPI is still awake, we don't have to do anything

	// timeout!

	// this will generate several isoSPI pulses: long-, long+, ....
	// the first part in the daisychain will enable its isoSPI
	// circuit and send a long+ pulse to the 2nd device
	// that will wake up isoSPI of the 2nd device and again
	// send a long+ to the next device and so on.
	// All those long+ pulses are equivalent to CS-high events.
	// only after all isoSPI ports are awake a CS low can
	// be generated

	// Note:
	// Before all devices of the chain are awake the LTC2949
	// will never see a long- pulse and thus it won't see a CS-low
	// event. Only after all isoSPI circuits are awake they can
	// actually pass through a long- pulse which triggers a CS-
	// low event on the 2949 that wakes up the LTC2949 core from sleep!
	// 
	if (LTC2949_CellMonitorCount)
	{
		for (uint8_t i = 0; i <= LTC2949_CellMonitorCount; i++)
		{
			spiStart(); //digitalWrite(LTC2949_CS, LOW);
			// Guarantees the isoSPI will be in ready mode
			// measured CS low pulse is 16 us
			delayMicroseconds(8);
			spiStop();  //digitalWrite(LTC2949_CS, HIGH);
			// add this delay for every part in the 
			// daisy-chain to make sure all parts are up and ready
			delayMicroseconds(400);
		}
	}
	else
	{
		spiStart();
		delayMicroseconds(8);
		spiStop();
		delayMicroseconds(10);
	}

	LTC2949_isoSPI_wakeup_timeoutUs_init();
#else
	// for raspberry pi we integrate it directly into libsoc_isospi_rw
#endif
}

// FIFO data states: MSB,LSB,TAG
#define LTC2949_RDFIFOLOOP_STATE_MSB 0x00U
#define LTC2949_RDFIFOLOOP_STATE_LSB 0x01U
#define LTC2949_RDFIFOLOOP_STATE_TAG 0x02U


#ifdef ARDUINO

byte _LTC2949_RdFifoLoop_(
	int8_t rdOvrCntDown,
	uint16_t * len,
	int16_t * adcWords,
	byte bytesPerPec
)
{
	// to store information if we got WROVR tag at the first reads
	boolean fifoFullAtBeginning = false;
	// set to true after the first RDOVR or OK tag was read
	boolean foundNonWrOvrTag = false;

	// check for reasonable number of samples
	if (len[0] > 1000U || len[0] < 1U)
	{
		len[0] = 0; // 0 samples read
		return LTC2949_ERRCODE_OTHER;
	}
#ifdef LTC2949_FIFO_DEBUG
	String str = "";
#endif
	// for pec error check
	uint16_t pec;
	pec15_calc_init(&pec);
	// byte counter used to intendify pec bytes
	byte byteCounter = 0;
	// FIFO register read state (MSB, LSB, TAG)
	byte rdFifoState = LTC2949_RDFIFOLOOP_STATE_MSB;
	// the current ADC word read (might be valid or not, for this reason we store it to adcWord first before it is copied to adcWords array)
	int16_t adcWord = 0xFFFF;
	// holds error code in case of errors
	byte error = 0;
	// counts the number of samples already read
	uint16_t samplesRead = 0;
	// for timeout check
	// worst case fast conversion time is 0.8us. +10ms margin for some overhead and Arduino uncertainty
	unsigned long timeout = (unsigned long)(0.8 * len[0] + 10.51);
	timeout += millis(); // set the timeout
	do
	{
		byte spiByte = spi_read_calc_pec(&pec);
		byteCounter++; // counts 1,2,3,....bytesPerPec,1,2.... (e.g. 1,2,3,4,5,6,1,2,.... in case of indirect read via RDCV)
		switch (rdFifoState)
		{
		case LTC2949_RDFIFOLOOP_STATE_MSB:
			rdFifoState = LTC2949_RDFIFOLOOP_STATE_LSB; // set next state
			adcWord = spiByte; // store the MSB
			break;

		case LTC2949_RDFIFOLOOP_STATE_LSB:
			rdFifoState = LTC2949_RDFIFOLOOP_STATE_TAG; // set next state
			adcWord = adcWord << 8 | spiByte; // add the LSB
			break;

		case LTC2949_RDFIFOLOOP_STATE_TAG:
		default:
			rdFifoState = LTC2949_RDFIFOLOOP_STATE_MSB; // set next state
			if (spiByte == LTC2949_FIFO_TAG_WROVR ||
				spiByte == LTC2949_FIFO_TAG_OK) // both TAGs are for valid samples
			{
				if (spiByte == LTC2949_FIFO_TAG_WROVR)
				{
					if (foundNonWrOvrTag)
					{
						// Now for sure we lost samples, as there were valid samples before
						error |= LTC2949_ERRCODE_FIFOWROVR;

					}
					else if (byteCounter == 3 || byteCounter == 6) // TODO: the first two can have the WROVR tag if FIFO was full, TODO: CHECK!
					{
						// that means this is the 1st or 2nd sample we read. As we do not know how much time elapsed
						// before the FIFO read starts we can not say if this is an error or not. E.g. we will
						// always start reading from FIFO full if more than 0.8*1000 ms have elapsed after
						// starting fast continuous.
						// Still we have to inform the user about this so he can take actions if necessary
						fifoFullAtBeginning = true;
					}
					else
					{
						// arriving here means we did not read any OK or RDOVR sample yet and this is not the first sample
						// which means more than one WROVR. This for sure is an error we flag here
						error |= LTC2949_ERRCODE_FIFOWROVR;
					}
					// if no valid sample was found yet, we can't say if we still read continuous samples!
				}
				else
				{
					foundNonWrOvrTag = true; // OK tag!
				}
				// store the sample
				adcWords[samplesRead] = adcWord;
				// increment number of read samples read
				samplesRead++;
			}
			else if (spiByte == LTC2949_FIFO_TAG_RDOVR)
			{
				// this sample was already read, nothing to do (FIFO is empty at this point)
				// just set flag for non-WROVR tag
				foundNonWrOvrTag = true;
				// RDOVR tag counter:
				if (rdOvrCntDown > 0)
				{
					if (samplesRead)
						rdOvrCntDown = 0; // stop asap if we already read good samples
					else
						// no valid samples read yet
						// decrement RDOVR tag counter
						// used to stop reading after RDOVR was read n times
						rdOvrCntDown--;
				}
#ifdef LTC2949_FIFO_DEBUG
				str += '\n';
				str += 'R';
				str += samplesRead;
#endif
			}
			else
			{
				// Unknown TAG, communication error!
				error |= LTC2949_ERRCODE_FIFOTAGERR;
			}
		} // end switch (stateChCntHits & LTC2949_RDFIFOLOOP_STATE_MASK)

		// check if we are going to read the PEC bytes
		if (byteCounter % bytesPerPec == 0)
		{
			byteCounter = 0;
			// read the PEC bytes and check for PEC error
			spi_read_chk_pec(&pec, &error);
			pec15_calc_init(&pec);
		}

		// check for timeout error
		if (LTC_TIMEOUT_CHECK(millis(), timeout))
		{
			error |= LTC2949_ERRCODE_TIMEOUT;
			break;
		}

		// check if we should stop after FIFO empty
		if (rdOvrCntDown == 0)
			break;

		// TODO stop on error?
		// if (error) break;

	} while (samplesRead < len[0]); // continue until all requested samples were read

	// return number of samples read, but preserve LTC2949_FLAG_RDFIFOLOOP_FIFOFULL flag
	len[0] = samplesRead;
	if (fifoFullAtBeginning)
		len[0] |= LTC2949_FLAG_RDFIFOLOOP_FIFOFULL;

#ifdef LTC2949_FIFO_DEBUG
	Serial.println(str);
	str = "";
#endif

	return error;
}

byte _LTC2949_IndirectRdFifo_(
	byte fifoAddr,
	int8_t rdOvrCntDown,
	uint16_t * len,
	int16_t * adcWords,
	byte * cellMonDat)
{
	if (LTC2949_DebugEnable)
#ifdef ARDUINO
		Serial.print(F("fr:"));
#else
		printf(F("fr:"));
#endif

	_LTC2949_indirect_bcren_wr_pg_addr_(fifoAddr);

	uint16_t pec;
	pec15_calc_init(&pec);

	LTC2949_wakeup_idle(LTC2949_forceWrRegsCtrl); // just to be on the safe side, in case we force write REGSCTRL we also force isoSPI wakeup pulses
	spiStart();

	spi_write_calc_pec(&pec, LTC2949_IR_RDCV_CMD_HIGH);
	spi_write_calc_pec(&pec, LTC2949_IR_RDCV_CMD_LOW);
	spi_write_pec(&pec);

	byte error = 0;
	// Note: _LTC2949_IndirectRdFifo_ is only called if LTC2949_onTopOfDaisychain is true!!!
	if (uint16_t totalBytes = LTC2949_CellMonitorCount * 6)
	{
		pec15_calc_init(&pec);
		for (uint16_t i = 0; i < totalBytes;)
		{
			byte d = spi_read_calc_pec(&pec);
			if (cellMonDat)
				cellMonDat[i] = d;
			i++;
			if (i % LTC2949_IR_RDCV_BYTES_PER_PEC != 0)
				continue;
			// read the PEC bytes and check for PEC error
			spi_read_chk_pec(&pec, &error);
			pec15_calc_init(&pec);
		}
	}

	error |= _LTC2949_RdFifoLoop_(
		rdOvrCntDown,
		len,
		adcWords,
		LTC2949_IR_RDCV_BYTES_PER_PEC);

	spiStop();

	LTC2949_ProcessErrors(error);

	return error;
}

byte _LTC2949_DCRdFifo(
	byte fifoAddr,
	int8_t rdOvrCntDown,
	uint16_t * len,
	int16_t * adcWords)
{
	if (LTC2949_DebugEnable)
#ifdef ARDUINO
		Serial.print(F("FR:"));
#else
		printf(F("FR:"));
#endif

	uint16_t pec;
	pec15_calc_init(&pec);

	LTC2949_wakeup_idle(LTC2949_forceWrRegsCtrl); // just to be on the safe side, in case we force write REGSCTRL we also force isoSPI wakeup pulses
	spiStart();

	// direct read is always addressed!
	spi_write_calc_pec(&pec, LTC2949_ADDR_CMD);
	spi_write_calc_pec(&pec, fifoAddr);
	spi_write_pec(&pec);

#ifdef LTC2949_FIFO_RD_MUL16
	const byte bytesPerPec = 16U;
#else
	byte bytesPerPec = LTC2949_CALC_OPT_NO_PECS(len[0]);
#endif

	// write cmd_id (there is no PEC for the cmd_id byte as it has intrinsic checksum)
	spi_write(LTC2949_CALC_ID_BYTE(bytesPerPec, LTC2949_DcRead));
	byte error;

	error = _LTC2949_RdFifoLoop_(
		rdOvrCntDown,
		len,
		adcWords,
		bytesPerPec);

	spiStop();

	LTC2949_ProcessErrors(error);

	return error;
}

#else
#endif
// stop read once FIFO is empty
// NRDOVR:0..7

#define LTC2949_FIFO_STATE_1STSAMPLE_WROVR  0x01
#define LTC2949_FIFO_STATE_LASTSAMPLE_RDOVR 0x02

#ifdef ARDUINO
// Read fifo function: Note this function always reads either 16 bytes or 6 bytes per PEC, depending on 
// direct or indirect read. Still it never reads more samples than requested, thus it stops either when
// FIFO is empty (LTC2949_RDFIFO_STOP_EMPTY) or when number of samples requested were read. Thus
// it can happen, that for the last samples read no PEC is read!
byte LTC2949_ReadFifo(
	byte addr,
	uint16_t * len,
	int16_t * samples,
	boolean * fifoFull1stSample,
	int8_t rdOvrCntDown,
	byte * cellMonData)
{
	if (len == 0)
	{
		fifoFull1stSample[0] = false;
		return 0;
	}
	byte error = LTC2949_onTopOfDaisychain
		? _LTC2949_IndirectRdFifo_(addr, rdOvrCntDown, len, samples, cellMonData)
		: _LTC2949_DCRdFifo(addr, rdOvrCntDown, len, samples);

	if (fifoFull1stSample)
		fifoFull1stSample[0] = bitMaskSetChk(len[0], LTC2949_FLAG_RDFIFOLOOP_FIFOFULL);

	// clear all bits apart those reporting number of samples read
	len[0] &= LTC2949_FIFO_SAMPLES_CNT_MASK;
	return error;
}
#else
#endif

byte LTC2949_ReadChkStatusFaults(
	boolean lockMemAndClr,
	boolean printResult,
	byte len,
	byte * statFaultsExpAndRd,
	boolean * expChkFailed,
	byte expDefaultSet)
{
	len = min(len, 10); // STATUS, 7xALTERTS, FAULTS, EXTFAULTS = 10 is maximum

	// lock memory?
	byte error = lockMemAndClr ? LTC2949_MemLock(true) : 0;
	boolean  _expChkFailed;
	byte expStatFaultsDefault[10];
	if (expChkFailed == NULL)
		expChkFailed = &_expChkFailed;
	*expChkFailed = false;

	if (statFaultsExpAndRd == NULL)
		statFaultsExpAndRd = expStatFaultsDefault;
	else
	{
		// default expectations
		if (expDefaultSet & _LTC2949_STATFAULTSCHK_ALL_DEFAULT_MASK)
		{
			// set all to 0
			memset(expStatFaultsDefault, 0, len);

			// after sleep?
			if (bitMaskSetChk(expDefaultSet, LTC2949_STATFAULTSCHK_DFLT_AFTER_SLEEP))
				expStatFaultsDefault[0] =
				LTC2949_BM_STATUS_UVLOA |
				LTC2949_BM_STATUS_PORA |
				LTC2949_BM_STATUS_UVLOD;//0x0B

			// after reset?
			if (bitMaskSetChk(expDefaultSet, LTC2949_STATFAULTSCHK_DFLT_AFTER_RESET))
			{
				expStatFaultsDefault[0] =
					LTC2949_BM_STATUS_UVLOA |
					LTC2949_BM_STATUS_PORA |
					LTC2949_BM_STATUS_UVLOSTBY |
					LTC2949_BM_STATUS_UVLOD;//0x0F
				if (len > 9)
					expStatFaultsDefault[8] = LTC2949_BM_EXTFAULTS_HWMBISTEXEC;
			}

			// when in cont?
			if (bitMaskSetChk(expDefaultSet, LTC2949_STATFAULTSCHK_DFLT_AFTER_CONT))
				expStatFaultsDefault[0] |= LTC2949_BM_STATUS_UPDATE;

			// or finally all cleared (overwrites all above)
			if (bitMaskSetChk(expDefaultSet, LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR))
			{
				expStatFaultsDefault[0] = 0;
				if (len > 9)
					expStatFaultsDefault[8] = 0;
			}
		}
		else
		{
			// use given values
			memcpy(expStatFaultsDefault, statFaultsExpAndRd, len);
		}
	}

	// read all STATUS / ALTERS regs
	error |= LTC2949_READ(LTC2949_REG_STATUS, min(8, len), statFaultsExpAndRd);
	if (len > 9)
		// read EXTFAULTS & FAULTS reg
		error |= LTC2949_READ(LTC2949_REG_EXTFAULTS, 2, &(statFaultsExpAndRd[8]));
	else if (len > 8)
		// read FAULTS only
		error |= LTC2949_READ(LTC2949_REG_FAULTS, 1, &(statFaultsExpAndRd[8]));

	// clear should only be done if memory is locked!
	if (lockMemAndClr)
	{
		// zeros for clearing of status regs
		byte zeros[8];
		memset(zeros, 0, 8);
		// clear STATUS / ALTERS and FAULTS
		LTC2949_WRITE(LTC2949_REG_STATUS, min(8, len), zeros);
		if (len > 9)
			LTC2949_WRITE(LTC2949_REG_EXTFAULTS, 2, zeros);
		else if (len > 8)
			LTC2949_WRITE(LTC2949_REG_FAULTS, 1, zeros);
	}

	// print before modifying read results!
	if (printResult)
		LTC2949_ReportFaults(statFaultsExpAndRd, len);

	if (statFaultsExpAndRd != expStatFaultsDefault) // only if expected values were given, see above
	{
		// check for bits that should be ignored
		if (bitMaskSetChk(expDefaultSet, LTC2949_STATFAULTSCHK_IGNORE_STATUPD))
			bitMaskCpy(expStatFaultsDefault[0], statFaultsExpAndRd[0], LTC2949_BM_STATUS_UPDATE);
		if (bitMaskSetChk(expDefaultSet, LTC2949_STATFAULTSCHK_IGNORE_STAT0F))
			bitMaskCpy(expStatFaultsDefault[0], statFaultsExpAndRd[0], 0x0F);
		if (bitMaskSetChk(expDefaultSet, LTC2949_STATFAULTSCHK_IGNORE_MBISTEXEC) && len > 9)
			bitMaskCpy(expStatFaultsDefault[8], statFaultsExpAndRd[8], LTC2949_BM_EXTFAULTS_HWMBISTEXEC);

		// check regs versus expectations
		for (byte i = 0; i < len; i++)
			*expChkFailed = *expChkFailed || (statFaultsExpAndRd[i] != expStatFaultsDefault[i]);
	}

	if (lockMemAndClr)
	{
		error |= LTC2949_MemLock(false);
		// Note: Function LTC2949_ReadChkStatusFaults should not be called faster than one system tick
		//       if clearing should be effective
		//delay(LTC2949_TIMING_IDLE2CONT2UPDATE);
	}

	return error;
}

void LTC2949_OpctlSleep()
{
	// we always start from page 0 after wakeup so already make this adjustment here!
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] &= ~(LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0 | LTC2949_BM_REGSCTRL_PAGE);
	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_SLEEP);
}

void spiStart()
{
#ifdef ARDUINO
	SPI.beginTransaction(LTC2949_SPISettings);
	digitalWrite(LTC2949_CS, LOW);
	if (LTC2949_DebugEnable)
#ifdef LTC2949_DEBUG_BYTE_ARRAY_CSTYLE
		Serial.print('{');
#else
		Serial.print('\\');
#endif
	else
		delayMicroseconds(1);
#else
	spiBufferLen = 0;
	spiBufferBusy = true;
#endif

}

void spiStop()
{
#ifdef ARDUINO
	if (LTC2949_DebugEnable)
#ifdef LTC2949_DEBUG_BYTE_ARRAY_CSTYLE
		Serial.print('}');
#else
		Serial.print('/');
#endif
	else
		delayMicroseconds(1);
	digitalWrite(LTC2949_CS, HIGH);
	LTC2949_isoSPI_wakeup_timeoutUs_init();
	SPI.endTransaction();
#else
	if (spiBufferLen == 0)
		spiBufferLen = 1;// write at least one dummy byte

	asp->SpiWriteRead(spiBuffer, spiBufferLen);

	if (LTC2949_DebugEnable)
	{
#ifdef LTC2949_DEBUG_BYTE_ARRAY_CSTYLE
		printf("{");
		SerialPrintByteArrayHex(spiBuffer, spiBufferLen, true, true);
		printf("}");
#else
		printf("\\");
		SerialPrintByteArrayHex(spiBuffer, spiBufferLen);
		printf("/");
#endif
	}
	spiBufferBusy = false;
#endif

}

void spi_write(byte data)
{
#ifdef ARDUINO
	SPI.transfer(data);
	if (LTC2949_DebugEnable)
	{
#ifdef LTC2949_DEBUG_BYTE_ARRAY_CSTYLE
		PrintZeroX();
		SerialPrintByteX2(data);
		PrintComma();
#else
		SerialPrintByteX2(data);
#endif
	}
#else
	spiBuffer[spiBufferLen++] = data;
#endif
}

byte spi_read(/*byte data*/)
{
#ifdef ARDUINO
	byte data = SPI.transfer(LTC2949_MOSI_FOR_READ);
	if (LTC2949_DebugEnable)
	{
#ifdef LTC2949_DEBUG_BYTE_ARRAY_CSTYLE
		PrintZeroX();
		SerialPrintByteX2(data);
		PrintComma();
#else
		SerialPrintByteX2(data);
#endif
	}
	return data;
#else
	if (spiBufferBusy)
	{
		spi_write(LTC2949_MOSI_FOR_READ);
		return 0;
	}
	return spiBuffer[spiBufferLen++];
#endif
}

static inline uint16_t spi_read_pec()
{
	uint16_t pec_slave = spi_read();         //MSB 1st
	pec_slave = pec_slave << 8 | spi_read(); //LSB 2nd
	return pec_slave;
}

boolean spi_read_chk_pec(uint16_t * pec15, byte * errCnt)
{
#ifdef ARDUINO
#else
	if (spiBufferBusy)
	{
		spi_write(LTC2949_MOSI_FOR_READ);
		spi_write(LTC2949_MOSI_FOR_READ);
		return true;
	}
#endif
	pec15_calc_finalize(pec15);
	boolean ok = spi_read_pec() == *pec15;
	// check for != NULL not necessary, as this functions must never be called with NULL
	if (!ok /*&& (errCnt != NULL)*/ && ((errCnt[0] & LTC2949_ERRCODE_PECERR_MASK) < LTC2949_PEC_ERR_CNT_MAX))
		errCnt[0]++;
	return ok;
}

void spi_write_calc_pec(uint16_t * pec15, byte data)
{
	pec15_calc(pec15, data);
	spi_write(data);
}

void spi_write_pec(uint16_t * pec15)
{
	pec15_calc_finalize(pec15);
	spi_write(*pec15 >> 8); // MSB 1st
	spi_write(*pec15);
}

byte spi_read_calc_pec(uint16_t * pec15)
{
	byte data = spi_read();
	pec15_calc(pec15, data);
	return data;
}

void LTC2949_AdjustRegsCtrlBuffer(byte addr, uint16_t len, byte * data)
{
	int16_t regsCtrlOffs = LTC2949_GetRegsCtrlDataOffset(addr, len);
	if (regsCtrlOffs >= 0)
		LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] = data[regsCtrlOffs];
}

byte LTC2949_CALC_ID_BYTE(byte bytesPerPec, boolean readNotWrite)
{
	byte cmd_id = bytesPerPec - 1;
	// 1st shift cmd_id[3..2] to position ID[4..3] and leave cmd_id[1..0] at ID[1..0]
	cmd_id = ((cmd_id & 0xc) << 1) | (cmd_id & 0x3);
	// calc PEC1 ^ PEC0, place to ID2 and PEC3 ^ PEC2, place to ID5
	cmd_id = (((cmd_id << 1) ^ (cmd_id << 2)) & 0x24) | cmd_id;
	// put all together
	cmd_id |= (readNotWrite ? LTC2949_DC_READ_BIT : LTC2949_DC_WRITE_BIT);
	return cmd_id;
}

byte LTC2949_CALC_OPT_NO_PECS(uint16_t len)
{
	// calc optimum number of bytes per PEC
	if (len <= 16)
		return len;
	else
	{
		byte bytesPerPec = 16;
		for (; bytesPerPec > 1; bytesPerPec--)
		{
			if ((len % bytesPerPec) == 0)
				break;
		}
		return bytesPerPec;
	}
}

uint16_t LTC2949_CALC_MUL_6(uint16_t len)
{
	byte rem = len % 6;

	return (rem != 0) ? len + 6 - rem : len;
	// e.g. 1: rem = 1, len = 1 + 6 - 1
}

float LTC2949_PLsb(boolean p1Notp2)
{
	return p1Notp2
		? (bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P1ASV)
			? LTC2949_LSB_P1ASV //LTC2949_LSB_SLOT1 / 8.0
			: LTC2949_LSB_P1)
		: (bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P2ASV)
			? LTC2949_LSB_P2ASV //LTC2949_LSB_SLOT2 / 8.0
			: LTC2949_LSB_P2);
}

float LTC2949_PLsb(boolean p1Notp2, float rsns)
{
	return p1Notp2 ?
		(bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P1ASV) ? LTC2949_LSB_SLOT1 / 8.0 : LTC2949_LSB_P1 / rsns) :
		(bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P2ASV) ? LTC2949_LSB_SLOT2 / 8.0 : LTC2949_LSB_P2 / rsns);
}

float LTC2949_SlotLsb(boolean slot1NotSlot2)
{
	return slot1NotSlot2 ?
		(bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC1) ? LTC2949_LSB_TEMP : LTC2949_LSB_SLOT1) :
		(bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC2) ? LTC2949_LSB_TEMP : LTC2949_LSB_SLOT2);
}

float LTC2949_Slot1DataToReal(byte * data)
{
	return LTC_2BytesToInt16(data) * (bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC1) ? LTC2949_LSB_TEMP : LTC2949_LSB_SLOT1);
}

float LTC2949_Slot2DataToReal(byte * data)
{
	return LTC_2BytesToInt16(data) * (bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC2) ? LTC2949_LSB_TEMP : LTC2949_LSB_SLOT2);
}

float LTC2949_P1DataToReal(byte * data)
{
	return LTC_3BytesToInt32(data) * (bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P1ASV) ? (LTC2949_LSB_SLOT1 / 8.0) : LTC2949_LSB_P1);
}

float LTC2949_P2DataToReal(byte * data)
{
	return LTC_3BytesToInt32(data) * (bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P2ASV) ? (LTC2949_LSB_SLOT2 / 8.0) : LTC2949_LSB_P2);
}

void LTC2949_SlotsCfg(byte slot1P, byte slot1N, byte slot2P, byte slot2N)
{
	byte data[4] = { slot1N, slot1P, slot2N, slot2P };
	LTC2949_WRITE(LTC2949_REG_SLOT1MUXN, 4, data);
}

byte LTC2949_GetSlotsCfg(byte * slot1P, byte * slot1N, byte * slot2P, byte * slot2N)
{
	byte data[4];
	byte error = LTC2949_READ(LTC2949_REG_SLOT1MUXN, 4, data);
	*slot1N = data[0];
	*slot1P = data[1];
	*slot2N = data[2];
	*slot2P = data[3];
	return error;
}

void LTC2949_SlotFastCfg(byte slotP, byte slotN)
{
	byte data[2] = { slotN, slotP };
	_LTC2949_direct_write_(LTC2949_REG_FAMUXN, 2, data);
}

byte LTC2949_GetSlotFastCfg(byte * slotP, byte * slotN)
{
	byte data[2];
	byte error = LTC2949_READ(LTC2949_REG_FAMUXN, 2, data);
	*slotN = data[0];
	*slotP = data[1];
	return error;
}

void LTC2949_ProcessErrors(byte error)
{
	if (error)
		LTC2949_forceWrRegsCtrl = LTC2949_autoForceWrRegsCtrl;
	if (!LTC2949_DebugEnable)
		return;
	//PrintComma();
	PrintOkErr(error);
}

// read FIFO address pointers
byte LTC2949_RdFifoCounters(byte fifoAddr, uint16_t * wrCnt, uint16_t * rdCnt)
{
	// convert FIFO address to fifo counter info base address (all page 0!)
	if (fifoAddr == LTC2949_REG_FIFOI1)
		fifoAddr = 0x68;
	else if (fifoAddr == LTC2949_REG_FIFOI2)
		fifoAddr = 0x6C;
	else if (fifoAddr == LTC2949_REG_FIFOBAT)
		fifoAddr = 0x71;
	else
		fifoAddr = 0x75; // LTC2949_REG_FIFOAUX

	byte data[4];
	byte error = LTC2949_READ(fifoAddr, 4, data);
	// convert data
	if (wrCnt)
		wrCnt[0] = data[0] | (0x03U & data[1]) << 8U;
	if (rdCnt)
		rdCnt[0] = (0xfcU & data[1]) >> 2U | (0x0fU & data[2]) << 6U;
	return error;

	/* not used:
	uint16_t rdCnt_ = (0xf0U & data[2]) >> 4U | (0x3fU & data[3]) << 4U;
	boolean wrExt = ((data[3] & (1U << 6U)) >> 6U) != 0;
	*/
}

// read number of samples available within FIFO
byte LTC2949_RdFifoSampleCount(byte fifoAddr, uint16_t * samples)
{
	uint16_t rdCnt;
	byte error = LTC2949_RdFifoCounters(
		/*fifoAddr:*/ fifoAddr,
		/*wrCnt:   */ samples,
		/*rdCnt:   */ &rdCnt);

	if (samples[0]/*(wrCnt)*/ <= rdCnt)
		samples[0]/*(wrCnt)*/ = samples[0]/*(wrCnt)*/ - rdCnt + 1000;
	else
		samples[0]/*(wrCnt)*/ = samples[0]/*(wrCnt)*/ - rdCnt;
	return error;
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// special routines to communicate via softByp to analog die:
// debug register TST
#define LTC2949_REG_DBG_TST 0xFE
// debug register FW
#define LTC2949_REG_DBG_FW  0xD0
// debug register DBG_0 on page 1
#define LTC2949_REG_DBG_0   0x140
// debug register DBG_1 on page 1
#define LTC2949_REG_DBG_1   0x141
// debug register DBG_2 on page 1
#define LTC2949_REG_DBG_2   0x142
// debug register DBG_3 on page 1
#define LTC2949_REG_DBG_3   0x143

// Poll for SoftBypass operation done
static inline uint8_t LTC2949_SwBypPollDone()
{
	// operation is done once 0xD0 is read as 0x00
	return LTC2949_PollReg(LTC2949_REG_DBG_FW, 0x00);
}

/*
>> > "{:x}".format(ord('d'))
'64'
>> > "{:x}".format(ord('e'))
'65'
>> > "{:x}".format(ord('B'))
'42'
*/

// Enter SoftBypass
void LTC2949_EnterDebug()
{
	LTC2949_WRITE(LTC2949_REG_DBG_TST, (byte)'d'); // write 0x64 to 0xFE
	LTC2949_WRITE(LTC2949_REG_DBG_TST, (byte)'e'); // write 0x65 to 0xFE
	LTC2949_WRITE(LTC2949_REG_DBG_TST, (byte)'B'); // write 0x42 to 0xFE
}

// Leave SoftBypass
void LTC2949_ExitDebug()
{
	LTC2949_WRITE(LTC2949_REG_DBG_TST, (byte)'B'); // write 0x42 to 0xFE
	LTC2949_WRITE(LTC2949_REG_DBG_TST, (byte)'e'); // write 0x65 to 0xFE
	LTC2949_WRITE(LTC2949_REG_DBG_TST, (byte)'d'); // write 0x64 to 0xFE
}

// write to analog die register
uint8_t LTC2949_SwBypWrite(uint8_t address, uint32_t ui24)
{
	uint8_t data[4];
	// data to be written (24 bit)
	data[0] = ui24 >> 16;
	data[1] = ui24 >> 8;
	data[2] = ui24;
	// analog die register address
	data[3] = address;

#ifdef ARDUINO
	if (LTC2949_DebugEnable)
	{
		Serial.println('W');
		PrintZeroX();
		Serial.println(ui24, HEX);
		PrintZeroX();
		Serial.println(address, HEX);
		SerialPrintByteArrayHex(data, 4, true);
		Serial.println('w');
	}
#else
#endif
	LTC2949_WRITE(LTC2949_REG_DBG_0, 4, &data[0]); // write data and address via 4-byte write burst
	LTC2949_WRITE(LTC2949_REG_DBG_FW, 3); // write operation code to DBG_FW: 3=WRITE
	return LTC2949_SwBypPollDone(); // poll for end of operation
}

// read register from analog die
uint8_t LTC2949_SwBypRead(uint8_t address, uint32_t * ui24)
{
	uint8_t data[3];
	uint8_t error;
	LTC2949_WRITE(LTC2949_REG_DBG_3, address); // write analog die register address
	LTC2949_WRITE(LTC2949_REG_DBG_FW, 4); // write operation code to DBG_FW: 4=READ
	error = LTC2949_SwBypPollDone(); // poll for end of operation
	error |= LTC2949_READ(LTC2949_REG_DBG_0, 3, data); // read result of operation (analog die's register content)
	// put all bytes into single 24 (32) bit word
	*ui24 = ((((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | ((uint32_t)data[2]));

#ifdef ARDUINO
	if (LTC2949_DebugEnable)
	{
		Serial.println('R');
		PrintZeroX();
		Serial.println(*ui24, HEX);
		PrintZeroX();
		Serial.println(address, HEX);
		SerialPrintByteArrayHex(data, 3, true);
		Serial.println('r');
	}
#else
#endif
	return error;
}

// enable access to PROM
static inline uint8_t LTC2949_SwBypPROMReadEnable()
{
	// set READ ENABLE of the PROM
	return LTC2949_SwBypWrite(
		0x60, // address of PROM CONTROL REG (96=0x60)
		0x01  // bit[0] is READ ENABLE BIT
	);
}

// disable access to PROM
static inline uint8_t LTC2949_SwBypPROMReadDisable()
{
	// clear READ ENABLE of the PROM
	return LTC2949_SwBypWrite(
		0x60, // address of PROM CONTROL REG (96=0x60)
		0x00  // bit[0] is READ ENABLE BIT
	);
}

#ifdef LTC2949_SWBYPGETPROM
uint8_t LTC2949_SwBypGetPROM(uint32_t * prom, uint8_t len)
{
	uint8_t error = LTC2949_SwBypPROMReadEnable();

	len = min(8, len);

	for (uint8_t i = 0; i < len; i++)
		error |= LTC2949_SwBypRead(0x50 + i, prom + i);

	error |= LTC2949_SwBypPROMReadDisable();
	return error;
}
#endif

// read factory trims (gain correction factors) for AUX, I1, I2 ADCs
uint8_t LTC2949_SwBypGetTrimms(int8_t * gcaux, int8_t * gci1, int8_t * gci2)
{
	uint32_t ui24; uint8_t error;
	error = LTC2949_SwBypPROMReadEnable(); // enable access to PROM
	error |= LTC2949_SwBypRead(0x50, &ui24); // read AUX GC
	*gcaux = (int8_t)(((ui24 & 0x0ff00U) >> 8) & 0xffU);
	error |= LTC2949_SwBypRead(0x51, &ui24); // read I1 GC
	*gci1 = (int8_t)(((ui24 & 0x0ff00U) >> 8) & 0xffU);
	error |= LTC2949_SwBypRead(0x52, &ui24); // read I2 GC
	*gci2 = (int8_t)(((ui24 & 0x0ff00U) >> 8) & 0xffU);
	error |= LTC2949_SwBypPROMReadDisable(); // disable access to PROM
	return error;
}

// set bits within analog die register
uint8_t LTC2949_SwBypSetBits(uint8_t address, uint32_t bits)
{
	uint32_t ui24; uint8_t error;
	error = LTC2949_SwBypRead(address, &ui24);
	ui24 |= bits;
	error |= LTC2949_SwBypWrite(address, ui24);
	return error;
}

// clear bits within analog die register
uint8_t LTC2949_SwBypClrBits(uint8_t address, uint32_t bits)
{
	uint32_t ui24; uint8_t error;
	error = LTC2949_SwBypRead(address, &ui24);
	ui24 &= ~bits;
	error |= LTC2949_SwBypWrite(address, ui24);
	return error;
}

uint8_t LTC2949_GetSiliconRevision(uint8_t * revCode, bool verbose)
{
	byte data[3];
	uint8_t error;
	if (revCode == NULL)
		revCode = &(data[2]);
	error = LTC2949_READ(LTC2949_REG_DIGINFO, 2, data);

	//LTC2949_REG_DIGINFO
	revCode[0] = (data[0] & 0xFU) + 1U; // data[0]bits[3:0]: digital die revision; data[0]bits[7:4]: digital die device ID (0b1000)
	//LTC2949_REG_ANAINFO
	revCode[0] |= (data[1] & 0x7U) << 4;// data[1]bits[2:0]: analog die revision;  data[1]bits[6:3]: analog die device ID (0b100)

	// sanity check on device info
	if (((data[0] & 0xF0U) != 0x80U) || ((data[1] & 0x38U) != (0x04U << 3)))
		error |= LTC2949_ERRCODE_OTHER;

	if (!verbose)
		return error;

	SerialPrintByteArrayHex(data, 2, true);
	PrintComma();
#ifdef ARDUINO
	Serial.print(revCode[0], HEX);
#else
	printf("%02hhX", revCode[0]);
#endif
	PrintComma();
	PrintOkErr(error);
	/*
	0x8326,64,OK
	*/
	return error;
}

// calculate temperature of NTC via Steinhart–Hart equation
// see LTC2949 on how to calculate Steinhart–Hart parameters A, B, C
float LTC2949_CalcNTCTemp(
	float vref, //!< VREF pin voltage (can also be set constant to the nominal value of 3.0V)
	float vntc, //!< NTC voltage versus GND (measured via Vx)
	float rRef, //!< Reference resistor connected between VREF pin and NTC.
	float ntcA, //!< NTC's Steinhart–Hart parameter A
	float ntcB, //!< NTC's Steinhart–Hart parameter B
	float ntcC  //!< NTC's Steinhart–Hart parameter C
)
{
	// Note:
	// I_NTC = (vref - vntc) / rref
	// R_NTC = vntc / I_NTC
	float rNTC = vntc / (vref - vntc) * rRef;
	float logRntc = log(rNTC);
	//
	// Steinhart–Hart equation
	float Tinv = ntcA + ntcB * logRntc + ntcC * logRntc * logRntc * logRntc;
	float tempdC = 1.0 / Tinv - 273.15;
#ifdef LTC2949_NTCCALC_VERBOSE
	Serial.print(vref);
	PrintComma();
	Serial.print(vntc);
	PrintComma();
	Serial.print(rNTC);
	PrintComma();
	Serial.print(logRntc);
	PrintComma();
	Serial.print(Tinv);
	PrintComma();
#endif
	return tempdC;
}

/*
uint8_t LTC2949_SwBypRead(uint8_t address, uint8_t *data)
{
uint8_t p_to_bytes[1];
uint8_t error;
p_to_bytes[0] = address;
LTC2949_WRITE(LTC2949_REG_DBG_3, 1, &p_to_bytes[0]);
p_to_bytes[0] = 4;
LTC2949_WRITE(LTC2949_REG_DBG_FW, 1, &p_to_bytes[0]);
error |= LTC2949_SwBypPollDone();
error |= LTC2949_READ(LTC2949_REG_DBG_0, 3, data);
return error;
}
*/

/*
uint8_t sw_byp_prom_rden(void)
{
// activate READ ENABLE of the PROM
return LTC2949_SwBypWrite(
96, // address of PROM CONTROL REG
1   // bit[0] is READ ENABLE BIT
);
}
uint8_t sw_byp_prom_rddis(void)
{
uint8_t data[4];
// deactivate READ ENABLE of the PROM
data[0] = 96;   // address of PROM CONTROL REG
data[1] = 0x00;
data[2] = 0x00;
data[3] = 0x00; // bit[0] is READ ENABLE BIT
return LTC2949_SwBypWrite(data[0], &data[1]);
}
*/

/*
uint8_t LTC2949_SwBypWrite(uint8_t address, uint8_t *data)
{
uint8_t p_to_bytes[4];
uint8_t error;
p_to_bytes[0] = *(data + 0);
p_to_bytes[1] = *(data + 1);
p_to_bytes[2] = *(data + 2);
p_to_bytes[3] = address;
LTC2949_WRITE(LTC2949_REG_DBG_0, 4, &p_to_bytes[0]);
p_to_bytes[0] = 3;
LTC2949_WRITE(LTC2949_REG_DBG_FW, 1, &p_to_bytes[0]);
error |= LTC2949_SwBypPollDone();
return error;
}
*/
