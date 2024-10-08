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

/*! @file
	@ingroup LTC2949
	Header for LTC2949 Battery Monitor Coulomb Counter
	*/

#ifndef _LTC2949_H
#define _LTC2949_H

#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#include "ltcmuc_tools.h"
#define LTCDEF_SPI_POLL_SUPPORT
#else
#undef LTCDEF_SPI_POLL_SUPPORT
#include "../ltcmuc_tools/ltcmuc_tools.h"
#include "../../AdiSpiAsync/AdiSpiAsync.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
typedef uint8_t byte;
typedef bool boolean;
void LTC2949_InitSPI(AdiSpiAsync* asa);
#endif

#undef LTC2949_DEBUG_WR_REGSCTRL

#undef LTC2949_DCRW_RAW_DATA_DEBUG

#undef LTC2949_FIFO_DEBUG

#ifdef LTC2949_FIFO_DEBUG
extern String fifoDebugStr;
#endif

#define LTC2949_FIFO_RD_MUL16
#define LTC2949_DEBUG_BYTE_ARRAY_CSTYLE


// #ifndef LTC2949_CS
// 
// #if defined (__AVR__) || (__avr__)
// //for Arduino UNO/Mega based on AVR IC
// #define LTC2949_CS SS
// #else
// //for Arduino DUE based on ARM IC
// #define LTC2949_CS 10
// #endif
// #endif

#define LTC2949_STORETIME_LAST_PEC

#define SPI_USE_ARDUINO_LIB

#define LTC2949_DEFAULT_SPIMODE      SPI_MODE3

#define LTC2949_DEFAULT_SPIFREQU     500000
#define LTC2949_MAX_SPIFREQU        1000000

// for debugging only, to control MOSI line during read actions.
#undef LTC2949_CONFIG_MOSI_FOR_READ
//#define LTC2949_CONFIG_MOSI_FOR_READ

#ifdef LTC2949_CONFIG_MOSI_FOR_READ
extern uint8_t LTC2949_MOSI_FOR_READ;
#else
#define LTC2949_MOSI_FOR_READ 0xFFU
#endif


// max. core boot-up time after power-up or reset (ms)
// typical boot-up time is 73ms. 10% margin is more than enough leading to 80ms
#define LTC2949_TIMING_BOOTUP              100U
// Time after power-up or reset to start polling if core is read (boot-up done, ms)
// the max. wakeup time spec is above. But we start polling for device to be ready earlier which is ok.
#define LTC2949_TIMING_START_POLL_CORE_RDY 10U
// max. CONT mode cycle time (ms)
#define LTC2949_TIMING_CONT_CYCLE          105U
// max. time it takes to lock the memory, worst case is twice CONT cycle time plus some margin (ms)
#define LTC2949_TIMING_MLOCK_ACK           (2*LTC2949_TIMING_CONT_CYCLE+15U)
// max. IDLE cycle time (ms)
// typical IDLE cycle time is 17 ms. 10% margin is more than enough leading to 19ms
#define LTC2949_TIMING_IDLE_CYCLE          19U
// max. power-up to auto sleep time (ms)
#define LTC2949_TIMING_AUTO_SLEEP_MAX      1500U
// min. power-up to auto sleep time (ms)
#define LTC2949_TIMING_AUTO_SLEEP          1000U
// max. time from CONT enable to 1st slow channel values UPDATE done
#define LTC2949_TIMING_IDLE2CONT2UPDATE    160U
// max. time from TB4 update to STATUS/ALERTS update
#define LTC2949_TIMING_TB4_TO_STATS        20U

// Reset Unlock code
// this value written to LTC2949_REG_RSTUNLCK unlocks the reset bit in LTC2949_REG_OPCTRL
#define LTC2949_CODE_RSTUNLCK 0x55


/*!

 |MD| Dec  | ADC Conversion Model|
 |--|------|---------------------|
 |01| 1    | Fast            |
 |10| 2    | Normal        |
 |11| 3    | Filtered          |
 */
#define MD_422HZ  0
#define MD_FAST 1
#define MD_NORMAL 2
#define MD_FILTERED 3


 /*!
 |CH | Dec  | Channels to convert |
 |---|------|---------------------|
 |000| 0    | All Cells       |
 |001| 1    | Cell 1 and Cell 7   |
 |010| 2    | Cell 2 and Cell 8   |
 |011| 3    | Cell 3 and Cell 9   |
 |100| 4    | Cell 4 and Cell 10  |
 |101| 5    | Cell 5 and Cell 11  |
 |110| 6    | Cell 6 and Cell 12  |
 */

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6

 /*!

   |CHG | Dec  |Channels to convert   |
   |----|------|----------------------|
   |000 | 0    | All GPIOS and 2nd Ref|
   |001 | 1    | GPIO 1           |
   |010 | 2    | GPIO 2               |
   |011 | 3    | GPIO 3           |
   |100 | 4    | GPIO 4           |
   |101 | 5    | GPIO 5         |
   |110 | 6    | Vref2            |
   */

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

   /*!****************************************************
   \brief Controls if Discharging transitors are enabled
	or disabled during Cell conversions.

   | DCP | Discharge Permitted During conversion |
   |----|----------------------------------------|
   |0   | No - discharge is not permitted        |
   |1   | Yes - discharge is permitted           |

   ********************************************************/
#define DCP_DISABLED 0
#define DCP_ENABLED 1


#define LTC2949_68XX_T6M_27KHZ_US ((unsigned int)( 290*1.1))
#define LTC2949_68XX_T6C_27KHZ_US ((unsigned int)(1113*1.1))

   //
   // WRITE command
#define LTC2949_68XX_CMDTYPE_WR   (0U<<11U)
// READ command
#define LTC2949_68XX_CMDTYPE_RD   (1U<<11)
// POLL command
#define LTC2949_68XX_CMDTYPE_PL   (2U<<11U)
// RD&POLL command (to be used with LTC2949 only to poll EOC after fast single shot)
#define LTC2949_68XX_CMDTYPE_RDPL (LTC2949_68XX_CMDTYPE_RD|LTC2949_68XX_CMDTYPE_PL)
// command type mask
#define LTC2949_68XX_CMDTYPE_MASK (3U<<11)

// Write Configuration 
// Register Group A
//0 0 0 0 0 0 0 0 0 0 1
#define LTC2949_68XX_CMD_WRCFGA (0x001U | LTC2949_68XX_CMDTYPE_WR)
//0 0 0 0 0 0 0 0 0 0 1
#define LTC2949_68XX_CMD_WRCFG LTC2949_68XX_CMD_WRCFGA
// Write Configuration 
// Register Group B*
//0 0 0 0 0 1 0 0 1 0 0
#define LTC2949_68XX_CMD_WRCFGB (0x024U | LTC2949_68XX_CMDTYPE_WR)
// Read Configuration 
// Register Group A
//0 0 0 0 0 0 0 0 0 1 0
#define LTC2949_68XX_CMD_RDCFGA (0x002U | LTC2949_68XX_CMDTYPE_RD)
//0 0 0 0 0 0 0 0 0 1 0
#define LTC2949_68XX_CMD_RDCFG LTC2949_68XX_CMD_RDCFGA
// Read Configuration 
// Register Group B*
//0 0 0 0 0 1 0 0 1 1 0
#define LTC2949_68XX_CMD_RDCFGB (0x026U | LTC2949_68XX_CMDTYPE_RD)
// Read Cell Voltage 
// Register Group A
//0 0 0 0 0 0 0 0 1 0 0
#define LTC2949_68XX_CMD_RDCVA (0x004U | LTC2949_68XX_CMDTYPE_RD)
// Read Cell Voltage 
// Register Group B
//0 0 0 0 0 0 0 0 1 1 0
#define LTC2949_68XX_CMD_RDCVB (0x006U | LTC2949_68XX_CMDTYPE_RD)
// Read Cell Voltage 
// Register Group C
//0 0 0 0 0 0 0 1 0 0 0
#define LTC2949_68XX_CMD_RDCVC (0x008U | LTC2949_68XX_CMDTYPE_RD)
// Read Cell Voltage 
// Register Group D
//0 0 0 0 0 0 0 1 0 1 0
#define LTC2949_68XX_CMD_RDCVD (0x00aU | LTC2949_68XX_CMDTYPE_RD)
// Read Cell Voltage 
// Register Group E*
//0 0 0 0 0 0 0 1 0 0 1
#define LTC2949_68XX_CMD_RDCVE (0x009U | LTC2949_68XX_CMDTYPE_RD)
// Read Cell Voltage 
// Register Group F*
//0 0 0 0 0 0 0 1 0 1 1
#define LTC2949_68XX_CMD_RDCVF (0x00bU | LTC2949_68XX_CMDTYPE_RD)
// Read Auxiliary 
// Register Group A
//0 0 0 0 0 0 0 1 1 0 0
#define LTC2949_68XX_CMD_RDAUXA (0x00cU | LTC2949_68XX_CMDTYPE_RD)
// Read Auxiliary 
// Register Group B
//0 0 0 0 0 0 0 1 1 1 0
#define LTC2949_68XX_CMD_RDAUXB (0x00eU | LTC2949_68XX_CMDTYPE_RD)
// Read Auxiliary 
// Register Group C*
//0 0 0 0 0 0 0 1 1 0 1
#define LTC2949_68XX_CMD_RDAUXC (0x00dU | LTC2949_68XX_CMDTYPE_RD)
// Read Auxiliary 
// Register Group D*
//0 0 0 0 0 0 0 1 1 1 1
#define LTC2949_68XX_CMD_RDAUXD (0x00fU | LTC2949_68XX_CMDTYPE_RD)
// Read Status 
// Register Group A
//0 0 0 0 0 0 1 0 0 0 0
#define LTC2949_68XX_CMD_RDSTATA (0x010U | LTC2949_68XX_CMDTYPE_RD)
// Read Status 
// Register Group B
//0 0 0 0 0 0 1 0 0 1 0
#define LTC2949_68XX_CMD_RDSTATB (0x012U | LTC2949_68XX_CMDTYPE_RD)
// Write S Control Register 
// Group
//0 0 0 0 0 0 1 0 1 0 0
#define LTC2949_68XX_CMD_WRSCTRL (0x014U | LTC2949_68XX_CMDTYPE_WR)
// Write PWM Register Group 
//0 0 0 0 0 1 0 0 0 0 0
#define LTC2949_68XX_CMD_WRPWM (0x020U | LTC2949_68XX_CMDTYPE_WR)
// Write PWM/S Control 
// Register Group B*
//0 0 0 0 0 0 1 1 1 0 0
#define LTC2949_68XX_CMD_WRPSB (0x01cU | LTC2949_68XX_CMDTYPE_WR)
// Read S Control Register 
// Group
//0 0 0 0 0 0 1 0 1 1 0
#define LTC2949_68XX_CMD_RDSCTRL (0x016U | LTC2949_68XX_CMDTYPE_RD)
// Read PWM Register Group 
//0 0 0 0 0 1 0 0 0 1 0
#define LTC2949_68XX_CMD_RDPWM (0x022U | LTC2949_68XX_CMDTYPE_RD)
// Read PWM/S Control 
// Register Group B*
//0 0 0 0 0 0 1 1 1 1 0
#define LTC2949_68XX_CMD_RDPSB (0x01eU | LTC2949_68XX_CMDTYPE_RD)
// Start S Control Pulsing and 
// Poll Status
//0 0 0 0 0 0 1 1 0 0 1
#define LTC2949_68XX_CMD_STSCTRL (0x019U | LTC2949_68XX_CMDTYPE_WR)
// Clear S Control Register 
// Group
//0 0 0 0 0 0 1 1 0 0 0
#define LTC2949_68XX_CMD_CLRSCTRL (0x018U | LTC2949_68XX_CMDTYPE_WR)
// Start Cell Voltage ADC 
// Conversion and Poll Status
//0 1 MD[1] MD[0] 1 1 DCP 0 CH[2] CH[1] CH[0]
#define LTC2949_68XX_CMD_ADCV (0x260U | LTC2949_68XX_CMDTYPE_WR)
// Start Open Wire ADC 
// Conversion and Poll Status
//0 1 MD[1] MD[0] PUP 1 DCP 1 CH[2] CH[1] CH[0]
#define LTC2949_68XX_CMD_ADOW (0x228U | LTC2949_68XX_CMDTYPE_WR)
// Start Self Test Cell Voltage 
// Conversion and Poll Status
//0 1 MD[1] MD[0] ST[1] ST[0] 0 0 1 1 1
#define LTC2949_68XX_CMD_CVST (0x207U | LTC2949_68XX_CMDTYPE_WR)
// Start Overlap Measurement of 
// Cell 7 Voltage
//0 1 MD[1] MD[0] 0 0 DCP 0 0 0 1
#define LTC2949_68XX_CMD_ADOL (0x201U | LTC2949_68XX_CMDTYPE_WR)
// Start GPIOs ADC Conversion 
// and Poll Status
//1 0 MD[1] MD[0] 1 1 0 0 CHG [2] CHG [1] CHG[0]
#define LTC2949_68XX_CMD_ADAX (0x460U | LTC2949_68XX_CMDTYPE_WR)
// Start GPIOs ADC Conversion 
// With Digital Redundancy and 
// Poll Status
//1 0 MD[1] MD[0] 0 0 0 0 CHG [2] CHG [1] CHG[0]
#define LTC2949_68XX_CMD_ADAXD (0x400U | LTC2949_68XX_CMDTYPE_WR)
// Start Self Test GPIOs 
// Conversion and Poll Status
//1 0 MD[1] MD[0] ST[1] ST[0] 0 0 1 1 1
#define LTC2949_68XX_CMD_AXST (0x407U | LTC2949_68XX_CMDTYPE_WR)
// Start Status Group ADC 
// Conversion and Poll Status
//1 0 MD[1] MD[0] 1 1 0 1 CHST[2] CHST[1] CHST[0]
#define LTC2949_68XX_CMD_ADSTAT (0x468U | LTC2949_68XX_CMDTYPE_WR)
// Start Status Group ADC 
// Conversion With Digital 
// Redundancy and Poll Status
//1 0 MD[1] MD[0] 0 0 0 1 CHST [2]CHST [1]CHST [0]
#define LTC2949_68XX_CMD_ADSTATD (0x408U | LTC2949_68XX_CMDTYPE_WR)
// Start Self Test Status Group 
// Conversion and Poll Status
//1 0 MD[1] MD[0] ST[1] ST[0] 0 1 1 1 1
#define LTC2949_68XX_CMD_STATST (0x40fU | LTC2949_68XX_CMDTYPE_WR)
// Start Combined Cell 
// Voltage and GPIO1  GPIO2 
// Conversion and Poll Status
//1 0 MD[1] MD[0] 1 1 DCP 1 1 1 1
#define LTC2949_68XX_CMD_ADCVAX (0x46fU | LTC2949_68XX_CMDTYPE_WR)
// Start Combined Cell Voltage 
// and SC Conversion and Poll 
// Status
//1 0 MD[1] MD[0] 1 1 DCP 0 1 1 1
#define LTC2949_68XX_CMD_ADCVSC (0x467U | LTC2949_68XX_CMDTYPE_WR)
// Clear Cell Voltage 
// Register Groups
//1 1 1 0 0 0 1 0 0 0 1
#define LTC2949_68XX_CMD_CLRCELL (0x711U | LTC2949_68XX_CMDTYPE_WR)
// Clear Auxiliary 
// Register Groups
//1 1 1 0 0 0 1 0 0 1 0
#define LTC2949_68XX_CMD_CLRAUX (0x712U | LTC2949_68XX_CMDTYPE_WR)
// Clear Status Register Groups  
//1 1 1 0 0 0 1 0 0 1 1
#define LTC2949_68XX_CMD_CLRSTAT (0x713U | LTC2949_68XX_CMDTYPE_WR)
// Poll ADC Conversion Status 
//1 1 1 0 0 0 1 0 1 0 0
#define LTC2949_68XX_CMD_PLADC (0x714U | LTC2949_68XX_CMDTYPE_PL)
// Diagnose MUX  
// and Poll Status
//1 1 1 0 0 0 1 0 1 0 1
#define LTC2949_68XX_CMD_DIAGN (0x715U | LTC2949_68XX_CMDTYPE_WR)
// Write COMM  
// Register Group
//1 1 1 0 0 1 0 0 0 0 1
#define LTC2949_68XX_CMD_WRCOMM (0x721U | LTC2949_68XX_CMDTYPE_WR)
// Read COMM Register Group 
//1 1 1 0 0 1 0 0 0 1 0
#define LTC2949_68XX_CMD_RDCOMM (0x722U | LTC2949_68XX_CMDTYPE_RD)
// Start I2C /SPI 
// Communication 
//1 1 1 0 0 1 0 0 0 1 1
#define LTC2949_68XX_CMD_STCOMM (0x722U | LTC2949_68XX_CMDTYPE_WR)
/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////



#define LTC2949_ERRCODE_OK           0x00
#define LTC2949_ERRCODE_COMMERR      0x10
#define LTC2949_ERRCODE_FIFOWROVR    0x20
#define LTC2949_ERRCODE_FIFOTAGERR   0x40
#define LTC2949_ERRCODE_OTHER        0x80
#define LTC2949_ERRCODE_TIMEOUT      0x08
#define LTC2949_ERRCODE_FAULTS       0x04
#define LTC2949_ERRCODE_PECERR_MASK  0x03
#define LTC2949_ERRCODE_PECERR_0     0x01
#define LTC2949_ERRCODE_PECERR_1     0x02
#define LTC2949_PEC_ERR_CNT_MAX      LTC2949_ERRCODE_PECERR_MASK

#define LTC2949_GPIO_OUTPUT_HIGH 3
#define LTC2949_GPIO_OUTPUT_LOW	 1
#define LTC2949_GPIO_INPUT		 0
#define LTC2949_GPIO_OUTPUT_TGL  2
   //GPIO control
   //00 : GPIO is input only
   //11 : GPIO is output driven high
   //01 : GPIO is output driven low
   //10 : GPIO is heartbeat output

/* Test for a little-endian machine */
// this is the way to check when using GCC compiler.
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define MCU_FLOAT_LITTLE_ENDIAN true
#else
#define MCU_FLOAT_LITTLE_ENDIAN false
#endif

// For Arduino the compiler (GCC) is little endian
#define MCU_FLOAT_31TO24 (MCU_FLOAT_LITTLE_ENDIAN ? 3 : 0)
#define MCU_FLOAT_23TO16 (MCU_FLOAT_LITTLE_ENDIAN ? 2 : 1)
#define MCU_FLOAT_15TO8  (MCU_FLOAT_LITTLE_ENDIAN ? 1 : 2)
#define MCU_FLOAT_7TO0   (MCU_FLOAT_LITTLE_ENDIAN ? 0 : 3)

// this is the difference in the bias of the exponents between F24/F32
#define LTC2949_F24TOF32EXPDIFF (64-128)
#define LTC2949_F32TOF24EXPDIFF (128-64)

#define LTC2949_RDFASTDATA_LENGTH 5
// indices within RDFASTDATA for the measurement results and the HS bytes
#define LTC2949_RDFASTDATA_I1  0
#define LTC2949_RDFASTDATA_I2  1
#define LTC2949_RDFASTDATA_BAT 2
#define LTC2949_RDFASTDATA_AUX 3
#define LTC2949_RDFASTDATA_HS  4

// Values for the FCTRL register, for setting to Fast On or Fast Off.
#define LTC2949_FACTRL_FAST_ALLCONTEN  0x0F
#define LTC2949_FACTRL_FAST_ALLEN      0x0E
#define LTC2949_FACTRL_FAST_OFF        0x00
#define LTC2949_FIFO_TAG_RDOVR  0x55U
#define LTC2949_FIFO_TAG_WROVR  0xAAU
#define LTC2949_FIFO_TAG_OK     0x00U

#define LTC2949_STATE_MSB 0U
#define LTC2949_STATE_LSB 1U
#define LTC2949_STATE_TAG 2U



// make a direct command read (to be used with _LTC2949_DcRW_ function)
#define LTC2949_DcRead  true
// make a direct command write (to be used with _LTC2949_DcRW_ function)
#define LTC2949_DcWrite false

#define LTC2949_DC_READ_BIT  0x80
#define LTC2949_DC_WRITE_BIT 0x40

// define the default RDCV command to be used by LTC2949 indirect read
// here we set RDCVA broadcast

#define LTC2949_IR_RDCV_CMD_HIGH 0x00
#define LTC2949_IR_RDCV_CMD_LOW  0x04

// LTC2949 has address 0xF, bit7 is addressed command bit
#define LTC2949_RDCV_ADDRESSED_CMD_HIGH (1<<7 | 0xF<<3 | LTC2949_IR_RDCV_CMD_HIGH)
#define LTC2949_RDCV_ADDRESSED_CMD_LOW  0x04
#define LTC2949_RDCV_BYTES_PER_PEC 6

#define LTC2949_SLOTMUX_GND        0
#define LTC2949_SLOTMUX_V1         1
#define LTC2949_SLOTMUX_V2         2
#define LTC2949_SLOTMUX_V3         3
#define LTC2949_SLOTMUX_V4         4
#define LTC2949_SLOTMUX_V5         5
#define LTC2949_SLOTMUX_V6         6
#define LTC2949_SLOTMUX_V7         7
#define LTC2949_SLOTMUX_V8         8
#define LTC2949_SLOTMUX_V9         9
#define LTC2949_SLOTMUX_V10        10
#define LTC2949_SLOTMUX_V11        11
#define LTC2949_SLOTMUX_V12        12
#define LTC2949_SLOTMUX_IMT        13
#define LTC2949_SLOTMUX_IPT        14
#define LTC2949_SLOTMUX_VBATM      15
#define LTC2949_SLOTMUX_VBATP      16
#define LTC2949_SLOTMUX_CFI2M      17
#define LTC2949_SLOTMUX_CFI2P      18
#define LTC2949_SLOTMUX_CFI1M      19
#define LTC2949_SLOTMUX_CFI1P      20
#define LTC2949_SLOTMUX_VREF2      22
#define LTC2949_SLOTMUX_VREF2_250k 23

//#define LTC2949_AUXMUX_TA1   21

// default register poll timeout
#define LTC2949_POLL_TIMEOUT 2000

// addressed: 0b1111_1110 = 0xFE
#define LTC2949_ADDR_CMD   0xFE

#define LTC2949_ADDRESS  0x1F

// define index of REGSCTRL and RDCVIADDR within LTC2949_iAddrRegsCtrl byte array
#define LTC2949_REGSCTRL_IX (LTC2949_REG_REGSCTRL - LTC2949_REG_RDCVIADDR)
#define LTC2949_RDCVIADDR_IX 0
#define LTC2949_IADDRREGSCTRL_LEN (LTC2949_REGSCTRL_IX + 1)
#define LTC2949_IR_RDCV_BYTES_PER_PEC 6
#define LTC2949_IS_ADDR_SPECIAL_ROW(ADDR) bitMaskSetChk(ADDR, 0xF0)

// worst case time after ADCV at which conversion results can be read via RDCV
#define LTC2949_FASTSSHT_RDY_TIME_US 1260U


// 0x0F00

// all HS ok means valid data
#define LTC2949_RDFASTDATA_HS_OKOK   0x0F0F
// last HS ok means next RDCV will report valid data
// note: data is re-ordered, meaning 0f is the last read and 00 is the first read!!!
//#define LTC2949_RDFASTDATA_HS_CLROK  0x0F00
#define LTC2949_RDFASTDATA_HS_OK     0x0F00
// all HS cleared means no new data
#define LTC2949_RDFASTDATA_HS_CLRCLR 0x0000
// check for valid HS bytes of fast SSHT result read acquired via RDCV
#define LTC2949_FASTSSHT_HS_OK(FASTDATABUFFER) (/*1st & 2nd HS:*/FASTDATABUFFER[LTC2949_RDFASTDATA_HS] == LTC2949_RDFASTDATA_HS_OKOK)
// check HS bytes if next RDCV would provide valid result (this is the case if the 1st HS was 0x00, but the last HS was 0x0F)
// note: data is re-ordered, meaning MSB is the last read and LSB is the first read!!!
#define LTC2949_FASTSSHT_HS_LAST_OK(FASTDATABUFFER) ((FASTDATABUFFER[LTC2949_RDFASTDATA_HS]&0xFF00) == LTC2949_RDFASTDATA_HS_OK)


// /*last HS:*/(FASTDATABUFFER[LTC2949_RDFASTDATA_HS]>>8 == LTC2949_RDFASTDATA_HS_OK) && /*first HS:*/(FASTDATABUFFER[LTC2949_RDFASTDATA_HS]&0xFF != LTC2949_RDFASTDATA_HS_OK))
// check all HS bytes are cleared
#define LTC2949_FASTSSHT_HS_CLR(FASTDATABUFFER) (/*1st & 2nd HS:*/FASTDATABUFFER[LTC2949_RDFASTDATA_HS] == LTC2949_RDFASTDATA_HS_CLRCLR)
// Note: other combinations than those listed above are not allowed


// maximum is 1000 samples, so we use 10 bits to count samples
#define LTC2949_FIFO_SAMPLES_CNT_MASK  ((1U<<10U) - 1U)
// flag used to identify if FIFO was full when the first sample was read
#define LTC2949_FLAG_RDFIFOLOOP_FIFOFULL (1U<<12U)

// stop reading from fifo after 2 RDOVR samples were read
#define LTC2949_RDFIFO_STOP_EMPTY    2
#define LTC2949_RDFIFO_REQU_SAMPLES -1


// for status faults checks
// default status reg after sleep recovery
#define LTC2949_STATFAULTSCHK_DFLT_AFTER_SLEEP      1
// default status/extfaults reg after (power  on) reset
#define LTC2949_STATFAULTSCHK_DFLT_AFTER_RESET     (1+2)
// default status/extfaults reg after cleared
#define LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR        4
// default status reg when in cont mode (UPDATE bit expected, but only if at least 100 ms have elapsed!)
#define LTC2949_STATFAULTSCHK_DFLT_AFTER_CONT       8
// used internal only to check if default values should be used as expected values
#define _LTC2949_STATFAULTSCHK_ALL_DEFAULT_MASK      0x0F

// ignore value of UPDATE bit in STATUS register
#define LTC2949_STATFAULTSCHK_IGNORE_STATUPD        16
// ignore value of low nibble bits in STATUS register
#define LTC2949_STATFAULTSCHK_IGNORE_STAT0F         32
// ignore value of MBISTEXEC in EXTFAULTS register
#define LTC2949_STATFAULTSCHK_IGNORE_MBISTEXEC      64

// Default setting used for LTC2949_ReadChkStatusFaults if parameter is not given
#define LTC2949_STATFAULTSCHK_DEFAULT_SETTING (LTC2949_STATFAULTSCHK_IGNORE_STATUPD|LTC2949_STATFAULTSCHK_IGNORE_STAT0F|LTC2949_STATFAULTSCHK_IGNORE_MBISTEXEC|LTC2949_STATFAULTSCHK_DFLT_AFTER_RESET)

// floating point gain correction factor with internal representation of 0xAAA... in LTC2949's internal fixed point format
#define LTC2949_SM_AA_FP_FAC (1.0 + 0.666015625)
// floating point gain correction factor with internal representation of 0x555... in LTC2949's internal fixed point format
#define LTC2949_SM_55_FP_FAC (1.0 + 0.3330078125)
#define LTC2949_SM_GC_LSB    0.0009765625
#define LTC2949_SM_GC_OFFS   1.0

#define LTC2949_VREF         3.0
//#define LTC2949_VREF_TOL     0.01

#define LTC2949_IADC_MIDSCALE  (LTC2949_LSB_I1*(2UL<<15))
// 124.5 mV
#define LTC2949_IADC_FULLSCALE (LTC2949_LSB_I1*(2UL<<16))

// nominal VREF2 value
#define LTC2949_VREF2         2.39
// tolerance for VREF2 measurement used for safety diagnostics
#define LTC2949_VREF2_TOL     1.16e-2

#define LTC2949_VREF2_INT_NOMINAL (int16_t)(LTC2949_VREF2/LTC2949_LSB_FAUX + 0.5)
#define LTC2949_VREF2_INT_ABS_TOL (int16_t)(LTC2949_VREF2/LTC2949_LSB_FAUX*LTC2949_VREF2_TOL + 0.5)

#define LTC2949_LEAKTEST_ABSTOL ((8e-9 * 250e3) / LTC2949_LSB_FAUX)


#define LTC2949_I1I2_TOL      0.001
// absolute tolerance in LSBs for the check I1 versus I2. 26*7.6uV=197.6uV
#define LTC2949_I1I2_FAST_ABS_TOL  ((int16_t)(200e-6/LTC2949_LSB_FIFOI1))

// there is a intrinsic gain variation between ADCs running fast or slow.
// this only needs to be taken into account when doing safety measure SM14
#define LTC2949_FAST2SLOW_MAGIC_GC 1.0004730224609375

// regexp all special regs: LTC2949_REG_(OPCTRL|FCURGPIOCTRL|FGPIOCTRL|FAMUX[NP]|FACTRL|FIFO(I[12]|BAT|AUX)|RDCVIADDR|REGSCTRL)

// <------- AUTO-GENERATED-HEADER STARTS HERE!!!! ------>

/////////////////////////////////////////////////////////
// Definitions of available registers without those    //
// that are part of multiple byte values               //
// (e.g. accumulators).                                //
//                                                     //
// Note: The address constants are 9 bit values where  //
// the lower 8 bit define the address and the 9th bit  //
// defines the page!                                   //
/////////////////////////////////////////////////////////

// Wake-up acknowledge: 
// Write 0x00 to confirm wake-up from sleep. 
// Without acknowledge device will go to sleep after 1 second. (register on page 0)
#define LTC2949_REG_WKUPACK 0x070

// Status register (active high) (register on page 0)
#define LTC2949_REG_STATUS 0x080

// Voltage, Temperature Threshold Alerts (register on page 0)
#define LTC2949_REG_STATVT 0x081

// Current Power Threshold Alerts (register on page 0)
#define LTC2949_REG_STATIP 0x082

// Charge Threshold Alerts (register on page 0)
#define LTC2949_REG_STATC 0x083

// Energy Threshold Alerts (register on page 0)
#define LTC2949_REG_STATE 0x084

// Charge, Energy Overflow Alerts (register on page 0)
#define LTC2949_REG_STATCEOF 0x085

// Time Base Alerts (register on page 0)
#define LTC2949_REG_STATTB 0x086

// VCC threshold alerts (register on page 0)
#define LTC2949_REG_STATVCC 0x087

// STATUS mask register (1: alert from STATUS is disabled) (register on page 0)
#define LTC2949_REG_STATUSM 0x088

// STATVT mask register (1: alert from STATVT is disabled) (register on page 0)
#define LTC2949_REG_STATVTM 0x089

// STATIP mask register (1: alert from STATIP is disabled) (register on page 0)
#define LTC2949_REG_STATIPM 0x08A

// STATC mask register (1: alert from STATC is disabled) (register on page 0)
#define LTC2949_REG_STATCM 0x08B

// STATE mask register (1: alert from STATE is disabled) (register on page 0)
#define LTC2949_REG_STATEM 0x08C

// STATCEOF mask register (1: alert from STATCEOF is disabled) (register on page 0)
#define LTC2949_REG_STATCEOFM 0x08D

// STATTB mask register (1: alert from STATTB is disabled) (register on page 0)
#define LTC2949_REG_STATTBM 0x08E

// STATVCC mask register (1: alert from STATVCC is disabled) (register on page 0)
#define LTC2949_REG_STATVCCM 0x08F

// Gain correction factory trim for AUX ADC (register on page 0)
#define LTC2949_REG_GCV 0x09F

// Gain correction factory trim for I1 ADC (register on page 0)
#define LTC2949_REG_GCI1 0x0AF

// Debug counter, counts milliseconds. (register on page 0)
#define LTC2949_REG_DBGCNT 0x0D5

// Shadow register of OCC1 control. (register on page 0)
#define LTC2949_REG_OCC1CTRLSHDW 0x0D9

// Shadow register of OCC2 control. (register on page 0)
#define LTC2949_REG_OCC2CTRLSHDW 0x0DA

// Extended faults (register on page 0)
#define LTC2949_REG_EXTFAULTS 0x0DC

// Miscellaneous faults (register on page 0)
#define LTC2949_REG_FAULTS 0x0DD

// Overcurrent  Control  Register allow to set the
// overcurrent comparator thresholds and deglitch times (register on page 0)
#define LTC2949_REG_OCC1CTRL 0x0DE

// Overcurrent  Control  Register allow to set the
// overcurrent comparator thresholds and deglitch times (register on page 0)
#define LTC2949_REG_OCC2CTRL 0x0DF

// Gain correction factory trim for I2 ADC (register on page 0)
#define LTC2949_REG_GCI2 0x0E0

// Accumulation control of Charge1/2, Energy1/2 by Current1/2 polarity.
// 00: Accumulation takes place always, 
// 01: only if the current is positive, 
// 10: only if the current is negative, 
// 11: No accumulation takes place. (register on page 0)
#define LTC2949_REG_ACCCTRL1 0x0E1

// Accumulation control of Charge3 and Energy4
// 00: Accumulation takes place always
// 01: only if the current is positive
// 10: only if the current is negative
// 11: No accumulation takes place (register on page 0)
#define LTC2949_REG_ACCCTRL2 0x0E2

// Deadband of Current1 for accumulation. 
// Current1 is not accumulated if its absolute value is below the deadband. (register on page 0)
#define LTC2949_REG_ACCI1DB 0x0E4

// Deadband of Current2 for accumulation. 
// Current2 is not accumulated if its absolute value is below the deadband. (register on page 0)
#define LTC2949_REG_ACCI2DB 0x0E5

// Gain correction factory trim for AUX ADC, MSB of F24 representation. (register on page 0)
#define LTC2949_REG_F24GCVH 0x0E6

// Gain correction factory trim for AUX ADC, center byte of F24 representation. (register on page 0)
#define LTC2949_REG_F24GCVM 0x0E7

// GPIO4 slow heartbeat master enable control (register on page 0)
#define LTC2949_REG_GPIO4HBCTRL 0x0E8

// Time Base Control (register on page 0)
#define LTC2949_REG_TBCTRL 0x0E9

// Gain correction factory trim for AUX ADC, LSB of F24 representation. (register on page 0)
#define LTC2949_REG_F24GCVL 0x0EA

// SLOT1 multiplexer setting of negative ADC input
// 0: AGND, 1-12: V1-V12
// 13: IMT, 14: IPT
// 15: BATM, 16: BATP
// 17: CFI2M, 18: CFI2P
// 19: CFI1M, 20: CFI1P
// 21,23: Reserved, 22: 2nd VREF (register on page 0)
#define LTC2949_REG_SLOT1MUXN 0x0EB

// SLOT1 multiplexer setting of positive ADC input
// 0: AGND, 1-12: V1-V12
// 13: IMT, 14: IPT
// 15: BATM, 16: BATP
// 17: CFI2M, 18: CFI2P
// 19: CFI1M, 20: CFI1P
// 21,23: Reserved, 22: 2nd VREF (register on page 0)
#define LTC2949_REG_SLOT1MUXP 0x0EC

// SLOT2 multiplexer setting of negative ADC input
// 0: AGND, 1-12: V1-V12
// 13: IMT, 14: IPT
// 15: BATM, 16: BATP
// 17: CFI2M, 18: CFI2P
// 19: CFI1M, 20: CFI1P
// 21,23: Reserved, 22: 2nd VREF (register on page 0)
#define LTC2949_REG_SLOT2MUXN 0x0ED

// SLOT2 multiplexer setting of positive ADC input
// 0: AGND, 1-12: V1-V12
// 13: IMT, 14: IPT
// 15: BATM, 16: BATP
// 17: CFI2M, 18: CFI2P
// 19: CFI1M, 20: CFI1P
// 21,23: Reserved, 22: 2nd VREF (register on page 0)
#define LTC2949_REG_SLOT2MUXP 0x0EE

// Operation Control (register on page 0)
#define LTC2949_REG_OPCTRL 0x0F0

// GPIO and current source control
// 00: GPIO is tristate
// 11: GPIO is high (DVCC)
// 01: GPIO is low (DGND)
// 10: GPIO toggles at 400kHz between DVCC, DGND (register on page 0)
#define LTC2949_REG_FCURGPIOCTRL 0x0F1

// GPIO control
// 00: GPIO is tristate
// 11: GPIO is high (DVCC)
// 01: GPIO is low (DGND)
// 10: GPIO toggles at 400kHz between DVCC, DGND (register on page 0)
#define LTC2949_REG_FGPIOCTRL 0x0F2

// Fast auxilliary multiplexer setting of negative ADC input
// 0: AGND, 1-12: V1-V12
// 13: IMT, 14: IPT
// 15: BATM, 16: BATP
// 17: CFI2M, 18: CFI2P
// 19: CFI1M, 20: CFI1P
// 21,23: Reserved, 22: 2nd VREF (register on page 0)
#define LTC2949_REG_FAMUXN 0x0F3

// Fast auxilliary multiplexer setting of positive ADC input
// 0: AGND, 1-12: V1-V12
// 13: IMT, 14: IPT
// 15: BATM, 16: BATP
// 17: CFI2M, 18: CFI2P
// 19: CFI1M, 20: CFI1P
// 21,23: Reserved, 22: 2nd VREF (register on page 0)
#define LTC2949_REG_FAMUXP 0x0F4

// Fast channel access control (register on page 0)
#define LTC2949_REG_FACTRL 0x0F5

// I1 fast channel conversion results fifo (register on page 0)
#define LTC2949_REG_FIFOI1 0x0F7

// I2 fast channel conversion results fifo (register on page 0)
#define LTC2949_REG_FIFOI2 0x0F8

// BAT (Px as voltage) fast channel conversion results fifo (register on page 0)
#define LTC2949_REG_FIFOBAT 0x0F9

// Auxilliary fast channel conversion results fifo (register on page 0)
#define LTC2949_REG_FIFOAUX 0x0FA

// Address pointer for indirect memory access via RDCV command (for daisy chain operation only) (register on page 0)
#define LTC2949_REG_RDCVIADDR 0x0FC

// Register Control (register on page 0)
#define LTC2949_REG_REGSCTRL 0x0FF

// External EEPROM Control Register (register on page 1)
#define LTC2949_REG_EEPROM 0x150

// error injectors: excite MISC fault, push buttons, start the test when set, poll for 0 == completion (register on page 1)
#define LTC2949_REG_ISO0 0x170

// excite AFSR1-2, sanity check of the voltages (register on page 1)
#define LTC2949_REG_ISO1 0x171

// excite data pump test (register on page 1)
#define LTC2949_REG_ISO2 0x172

// intermediate test results; AFSR2: 1st occ_ctrl to fail (register on page 1)
//#define LTC2949_REG_ISORPT 0x173

// if 0x55 it unlocks the SPI reset feature (register on page 1)
#define LTC2949_REG_RSTUNLCK 0x1A9

// MUXN gain correction setting 1 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXNSET1 0x1BC

// MUXP gain correction setting 1 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXPSET1 0x1BD

// MUXN gain correction setting 2 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXNSET2 0x1BE

// MUXP gain correction setting 2 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXPSET2 0x1BF

// MUXN gain correction setting 3 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXNSET3 0x1CC

// MUXP gain correction setting 3 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXPSET3 0x1CD

// MUXN gain correction setting 4 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXNSET4 0x1CE

// MUXP gain correction setting 4 (see SLOT1MUX, SLOT2MUX and FAMUX) (register on page 1)
#define LTC2949_REG_MUXPSET4 0x1CF

// Reserved, write 0 (register on page 1)
#define LTC2949_REG_RESERVED 0x1DE

// ADC Configuration Register allows to disable power multiplication on P1ADC and P2ADC, to turn on NTC temperature measurement of SLOT1 / SLOT2 measurements. (register on page 1)
#define LTC2949_REG_ADCCONF 0x1DF

// Stores information about digital die revision (register on page 1)
#define LTC2949_REG_DIGINFO 0x1EE

// Stores information about analog die revision (register on page 1)
#define LTC2949_REG_ANAINFO 0x1EF

/////////////////////////////////////////////////////////
// Definitions of all byte addresses of all multiple   //
// byte values (e.g. accumulators).                    //
//                                                     //
// Note: The address constants are 9 bit values where  //
// the lower 8 bit define the address and the 9th bit  //
// defines the page!                                   //
/////////////////////////////////////////////////////////

// Charge1 (register on page 0)
#define LTC2949_REG_C1_47_40 0x000

// (register on page 0)
#define LTC2949_REG_C1_39_32 0x001

// (register on page 0)
#define LTC2949_REG_C1_31_24 0x002

// (register on page 0)
#define LTC2949_REG_C1_23_16 0x003

// (register on page 0)
#define LTC2949_REG_C1_15_8 0x004

// (register on page 0)
#define LTC2949_REG_C1_7_0 0x005

// Energy1 (register on page 0)
#define LTC2949_REG_E1_47_40 0x006

// (register on page 0)
#define LTC2949_REG_E1_39_32 0x007

// (register on page 0)
#define LTC2949_REG_E1_31_24 0x008

// (register on page 0)
#define LTC2949_REG_E1_23_16 0x009

// (register on page 0)
#define LTC2949_REG_E1_15_8 0x00A

// (register on page 0)
#define LTC2949_REG_E1_7_0 0x00B

// Time1 (register on page 0)
#define LTC2949_REG_TB1_31_24 0x00C

// (register on page 0)
#define LTC2949_REG_TB1_23_16 0x00D

// (register on page 0)
#define LTC2949_REG_TB1_15_8 0x00E

// (register on page 0)
#define LTC2949_REG_TB1_7_0 0x00F

// Charge2 (register on page 0)
#define LTC2949_REG_C2_47_40 0x010

// (register on page 0)
#define LTC2949_REG_C2_39_32 0x011

// (register on page 0)
#define LTC2949_REG_C2_31_24 0x012

// (register on page 0)
#define LTC2949_REG_C2_23_16 0x013

// (register on page 0)
#define LTC2949_REG_C2_15_8 0x014

// (register on page 0)
#define LTC2949_REG_C2_7_0 0x015

// Energy2 (register on page 0)
#define LTC2949_REG_E2_47_40 0x016

// (register on page 0)
#define LTC2949_REG_E2_39_32 0x017

// (register on page 0)
#define LTC2949_REG_E2_31_24 0x018

// (register on page 0)
#define LTC2949_REG_E2_23_16 0x019

// (register on page 0)
#define LTC2949_REG_E2_15_8 0x01A

// (register on page 0)
#define LTC2949_REG_E2_7_0 0x01B

// Time2 (register on page 0)
#define LTC2949_REG_TB2_31_24 0x01C

// (register on page 0)
#define LTC2949_REG_TB2_23_16 0x01D

// (register on page 0)
#define LTC2949_REG_TB2_15_8 0x01E

// (register on page 0)
#define LTC2949_REG_TB2_7_0 0x01F

// Charge3 (register on page 0)
#define LTC2949_REG_C3_63_56 0x024

// (register on page 0)
#define LTC2949_REG_C3_55_48 0x025

// (register on page 0)
#define LTC2949_REG_C3_47_40 0x026

// (register on page 0)
#define LTC2949_REG_C3_39_32 0x027

// (register on page 0)
#define LTC2949_REG_C3_31_24 0x028

// (register on page 0)
#define LTC2949_REG_C3_23_16 0x029

// (register on page 0)
#define LTC2949_REG_C3_15_8 0x02A

// (register on page 0)
#define LTC2949_REG_C3_7_0 0x02B

// Time3 (register on page 0)
#define LTC2949_REG_TB3_31_24 0x02C

// (register on page 0)
#define LTC2949_REG_TB3_23_16 0x02D

// (register on page 0)
#define LTC2949_REG_TB3_15_8 0x02E

// (register on page 0)
#define LTC2949_REG_TB3_7_0 0x02F

// Energy4 (register on page 0)
#define LTC2949_REG_E4_63_56 0x034

// (register on page 0)
#define LTC2949_REG_E4_55_48 0x035

// (register on page 0)
#define LTC2949_REG_E4_47_40 0x036

// (register on page 0)
#define LTC2949_REG_E4_39_32 0x037

// (register on page 0)
#define LTC2949_REG_E4_31_24 0x038

// (register on page 0)
#define LTC2949_REG_E4_23_16 0x039

// (register on page 0)
#define LTC2949_REG_E4_15_8 0x03A

// (register on page 0)
#define LTC2949_REG_E4_7_0 0x03B

// Time4 (register on page 0)
#define LTC2949_REG_TB4_31_24 0x03C

// (register on page 0)
#define LTC2949_REG_TB4_23_16 0x03D

// (register on page 0)
#define LTC2949_REG_TB4_15_8 0x03E

// (register on page 0)
#define LTC2949_REG_TB4_7_0 0x03F

// Maximum Current1 tracking (register on page 0)
#define LTC2949_REG_I1MAX_15_8 0x040

// (register on page 0)
#define LTC2949_REG_I1MAX_7_0 0x041

// Minimum Current1 tracking (register on page 0)
#define LTC2949_REG_I1MIN_15_8 0x042

// (register on page 0)
#define LTC2949_REG_I1MIN_7_0 0x043

// Maximum Power1 tracking (register on page 0)
#define LTC2949_REG_P1MAX_15_8 0x044

// (register on page 0)
#define LTC2949_REG_P1MAX_7_0 0x045

// Minimum Power1 tracking (register on page 0)
#define LTC2949_REG_P1MIN_15_8 0x046

// (register on page 0)
#define LTC2949_REG_P1MIN_7_0 0x047

// Maximum Current2 tracking (register on page 0)
#define LTC2949_REG_I2MAX_15_8 0x048

// (register on page 0)
#define LTC2949_REG_I2MAX_7_0 0x049

// Minimum Current2 tracking (register on page 0)
#define LTC2949_REG_I2MIN_15_8 0x04A

// (register on page 0)
#define LTC2949_REG_I2MIN_7_0 0x04B

// Maximum Power2 tracking (register on page 0)
#define LTC2949_REG_P2MAX_15_8 0x04C

// (register on page 0)
#define LTC2949_REG_P2MAX_7_0 0x04D

// Minimum Power2 tracking (register on page 0)
#define LTC2949_REG_P2MIN_15_8 0x04E

// (register on page 0)
#define LTC2949_REG_P2MIN_7_0 0x04F

// Maximum Voltage at BAT (BATP - BATM) tracking (register on page 0)
#define LTC2949_REG_BATMAX_15_8 0x050

// (register on page 0)
#define LTC2949_REG_BATMAX_7_0 0x051

// Minimum Voltage at BAT (BATP - BATM) tracking (register on page 0)
#define LTC2949_REG_BATMIN_15_8 0x052

// (register on page 0)
#define LTC2949_REG_BATMIN_7_0 0x053

// Maximum Temperature tracking (register on page 0)
#define LTC2949_REG_TEMPMAX_15_8 0x054

// (register on page 0)
#define LTC2949_REG_TEMPMAX_7_0 0x055

// Minimum Temperature tracking (register on page 0)
#define LTC2949_REG_TEMPMIN_15_8 0x056

// (register on page 0)
#define LTC2949_REG_TEMPMIN_7_0 0x057

// Maximum VCC voltage tracking (register on page 0)
#define LTC2949_REG_VCCMAX_15_8 0x058

// (register on page 0)
#define LTC2949_REG_VCCMAX_7_0 0x059

// Minimum VCC voltage tracking (register on page 0)
#define LTC2949_REG_VCCMIN_15_8 0x05A

// (register on page 0)
#define LTC2949_REG_VCCMIN_7_0 0x05B

// Maximum Voltage of Slot1 (register on page 0)
#define LTC2949_REG_SLOT1MAX_15_8 0x05C

// (register on page 0)
#define LTC2949_REG_SLOT1MAX_7_0 0x05D

// Minimum Voltage at Slot1 (register on page 0)
#define LTC2949_REG_SLOT1MIN_15_8 0x05E

// (register on page 0)
#define LTC2949_REG_SLOT1MIN_7_0 0x05F

// Maximum Voltage of Slot2 (register on page 0)
#define LTC2949_REG_SLOT2MAX_15_8 0x060

// (register on page 0)
#define LTC2949_REG_SLOT2MAX_7_0 0x061

// Minimum Voltage at Slot2 (register on page 0)
#define LTC2949_REG_SLOT2MIN_15_8 0x062

// (register on page 0)
#define LTC2949_REG_SLOT2MIN_7_0 0x063

// RAW Current1 (register on page 0)
#define LTC2949_REG_I1RAW_23_16 0x079

// (register on page 0)
#define LTC2949_REG_I1RAW_15_8 0x07A

// (register on page 0)
#define LTC2949_REG_I1RAW_7_0 0x07B

// RAW Current2 (register on page 0)
#define LTC2949_REG_I2RAW_23_26 0x07C

// (register on page 0)
#define LTC2949_REG_I2RAW_25_8 0x07D

// (register on page 0)
#define LTC2949_REG_I2RAW_7_0 0x07E

// Current1 (register on page 0)
#define LTC2949_REG_I1_23_16 0x090

// (register on page 0)
#define LTC2949_REG_I1_15_8 0x091

// (register on page 0)
#define LTC2949_REG_I1_7_0 0x092

// Power1 (register on page 0)
#define LTC2949_REG_P1_23_16 0x093

// (register on page 0)
#define LTC2949_REG_P1_15_8 0x094

// (register on page 0)
#define LTC2949_REG_P1_7_0 0x095

// Current2 (register on page 0)
#define LTC2949_REG_I2_23_16 0x096

// (register on page 0)
#define LTC2949_REG_I2_15_8 0x097

// (register on page 0)
#define LTC2949_REG_I2_7_0 0x098

// Power2 (register on page 0)
#define LTC2949_REG_P2_23_16 0x099

// (register on page 0)
#define LTC2949_REG_P2_15_8 0x09A

// (register on page 0)
#define LTC2949_REG_P2_7_0 0x09B

// Average of Current1: (n-1)+(n-2)+(n-3)+(n-4) (register on page 0)
#define LTC2949_REG_IAVG1_23_16 0x09C

// (register on page 0)
#define LTC2949_REG_IAVG1_15_8 0x09D

// (register on page 0)
#define LTC2949_REG_IAVG1_7_0 0x09E

// Voltage at BAT (BATP - BATM) (register on page 0)
#define LTC2949_REG_BAT_15_8 0x0A0

// (register on page 0)
#define LTC2949_REG_BAT_7_0 0x0A1

// Temperature (register on page 0)
#define LTC2949_REG_TEMP_15_8 0x0A2

// (register on page 0)
#define LTC2949_REG_TEMP_7_0 0x0A3

// Voltage at AVCC (register on page 0)
#define LTC2949_REG_VCC_15_8 0x0A4

// (register on page 0)
#define LTC2949_REG_VCC_7_0 0x0A5

// Voltage of SLOT1 (register on page 0)
#define LTC2949_REG_SLOT1_15_8 0x0A6

// (register on page 0)
#define LTC2949_REG_SLOT1_7_0 0x0A7

// Voltage of SLOT2 (register on page 0)
#define LTC2949_REG_SLOT2_15_8 0x0A8

// (register on page 0)
#define LTC2949_REG_SLOT2_7_0 0x0A9

// Vref (register on page 0)
#define LTC2949_REG_VREF_15_8 0x0AA

// (register on page 0)
#define LTC2949_REG_VREF_7_0 0x0AB

// Average of Current2: (n-1)+(n-2)+(n-3)+(n-4) (register on page 0)
#define LTC2949_REG_IAVG2_23_16 0x0AC

// (register on page 0)
#define LTC2949_REG_IAVG2_15_8 0x0AD

// (register on page 0)
#define LTC2949_REG_IAVG2_7_0 0x0AE

// Current1 moving avergae 4 (register on page 0)
#define LTC2949_REG_I1AVG_23_16 0x0B0

// (register on page 0)
#define LTC2949_REG_I1AVG_15_8 0x0B1

// (register on page 0)
#define LTC2949_REG_I1AVG_7_0 0x0B2

// Current1 History 1 (prev. result '-1') (register on page 0)
#define LTC2949_REG_I1H1_23_16 0x0B3

// (register on page 0)
#define LTC2949_REG_I1H1_15_8 0x0B4

// (register on page 0)
#define LTC2949_REG_I1H1_7_0 0x0B5

// Current1 History 2 (prev. result '-1') (register on page 0)
#define LTC2949_REG_I1H2_23_16 0x0B6

// (register on page 0)
#define LTC2949_REG_I1H2_15_8 0x0B7

// (register on page 0)
#define LTC2949_REG_I1H2_7_0 0x0B8

// Current1 History 3 (prev. result '-2') (register on page 0)
#define LTC2949_REG_I1H3_23_16 0x0B9

// (register on page 0)
#define LTC2949_REG_I1H3_15_8 0x0BA

// (register on page 0)
#define LTC2949_REG_I1H3_7_0 0x0BB

// Current1 History 4 (prev. result '-3') (register on page 0)
#define LTC2949_REG_I1H4_23_16 0x0BC

// (register on page 0)
#define LTC2949_REG_I1H4_15_8 0x0BD

// (register on page 0)
#define LTC2949_REG_I1H4_7_0 0x0BE

// Current2 moving avergae 4 (register on page 0)
#define LTC2949_REG_I2AVG_23_16 0x0C0

// (register on page 0)
#define LTC2949_REG_I2AVG_15_8 0x0C1

// (register on page 0)
#define LTC2949_REG_I2AVG_7_0 0x0C2

// Current2 History 1 (prev. result '-1') (register on page 0)
#define LTC2949_REG_I2H1_23_16 0x0C3

// (register on page 0)
#define LTC2949_REG_I2H1_15_8 0x0C4

// (register on page 0)
#define LTC2949_REG_I2H1_7_0 0x0C5

// Current2 History 2 (prev. result '-1') (register on page 0)
#define LTC2949_REG_I2H2_23_16 0x0C6

// (register on page 0)
#define LTC2949_REG_I2H2_15_8 0x0C7

// (register on page 0)
#define LTC2949_REG_I2H2_7_0 0x0C8

// Current2 History 3 (prev. result '-2') (register on page 0)
#define LTC2949_REG_I2H3_23_16 0x0C9

// (register on page 0)
#define LTC2949_REG_I2H3_15_8 0x0CA

// (register on page 0)
#define LTC2949_REG_I2H3_7_0 0x0CB

// Current2 History 4 (prev. result '-3') (register on page 0)
#define LTC2949_REG_I2H4_23_16 0x0CC

// (register on page 0)
#define LTC2949_REG_I2H4_15_8 0x0CD

// (register on page 0)
#define LTC2949_REG_I2H4_7_0 0x0CE

// Charge 1 threshold high (register on page 1)
#define LTC2949_REG_C1TH_47_40 0x100

// (register on page 1)
#define LTC2949_REG_C1TH_39_32 0x101

// (register on page 1)
#define LTC2949_REG_C1TH_31_24 0x102

// (register on page 1)
#define LTC2949_REG_C1TH_23_16 0x103

// (register on page 1)
#define LTC2949_REG_C1TH_15_8 0x104

// (register on page 1)
#define LTC2949_REG_C1TH_7_0 0x105

// Charge 1 threshold low (register on page 1)
#define LTC2949_REG_C1TL_47_40 0x106

// (register on page 1)
#define LTC2949_REG_C1TL_39_32 0x107

// (register on page 1)
#define LTC2949_REG_C1TL_31_24 0x108

// (register on page 1)
#define LTC2949_REG_C1TL_23_16 0x109

// (register on page 1)
#define LTC2949_REG_C1TL_15_8 0x10A

// (register on page 1)
#define LTC2949_REG_C1TL_7_0 0x10B

// Time Base 1 threshold high (register on page 1)
#define LTC2949_REG_TB1TH_31_24 0x10C

// (register on page 1)
#define LTC2949_REG_TB1TH_23_16 0x10D

// (register on page 1)
#define LTC2949_REG_TB1TH_15_8 0x10E

// (register on page 1)
#define LTC2949_REG_TB1TH_7_0 0x10F

// Energy 1 threshold high (register on page 1)
#define LTC2949_REG_E1TH_47_40 0x110

// (register on page 1)
#define LTC2949_REG_E1TH_39_32 0x111

// (register on page 1)
#define LTC2949_REG_E1TH_31_24 0x112

// (register on page 1)
#define LTC2949_REG_E1TH_23_16 0x113

// (register on page 1)
#define LTC2949_REG_E1TH_15_8 0x114

// (register on page 1)
#define LTC2949_REG_E1TH_7_0 0x115

// Energy 1 threshold low (register on page 1)
#define LTC2949_REG_E1TL_47_40 0x116

// (register on page 1)
#define LTC2949_REG_E1TL_39_32 0x117

// (register on page 1)
#define LTC2949_REG_E1TL_31_24 0x118

// (register on page 1)
#define LTC2949_REG_E1TL_23_16 0x119

// (register on page 1)
#define LTC2949_REG_E1TL_15_8 0x11A

// (register on page 1)
#define LTC2949_REG_E1TL_7_0 0x11B

// Charge 2 threshold high (register on page 1)
#define LTC2949_REG_C2TH_47_40 0x120

// (register on page 1)
#define LTC2949_REG_C2TH_39_32 0x121

// (register on page 1)
#define LTC2949_REG_C2TH_31_24 0x122

// (register on page 1)
#define LTC2949_REG_C2TH_23_16 0x123

// (register on page 1)
#define LTC2949_REG_C2TH_15_8 0x124

// (register on page 1)
#define LTC2949_REG_C2TH_7_0 0x125

// Charge 2 threshold low (register on page 1)
#define LTC2949_REG_C2TL_47_40 0x126

// (register on page 1)
#define LTC2949_REG_C2TL_39_32 0x127

// (register on page 1)
#define LTC2949_REG_C2TL_31_24 0x128

// (register on page 1)
#define LTC2949_REG_C2TL_23_16 0x129

// (register on page 1)
#define LTC2949_REG_C2TL_15_8 0x12A

// (register on page 1)
#define LTC2949_REG_C2TL_7_0 0x12B

// Time Base 2 threshold high (register on page 1)
#define LTC2949_REG_TB2TH_31_24 0x12C

// (register on page 1)
#define LTC2949_REG_TB2TH_23_16 0x12D

// (register on page 1)
#define LTC2949_REG_TB2TH_15_8 0x12E

// (register on page 1)
#define LTC2949_REG_TB2TH_7_0 0x12F

// Energy 2 threshold high (register on page 1)
#define LTC2949_REG_E2TH_47_40 0x130

// (register on page 1)
#define LTC2949_REG_E2TH_39_32 0x131

// (register on page 1)
#define LTC2949_REG_E2TH_31_24 0x132

// (register on page 1)
#define LTC2949_REG_E2TH_23_16 0x133

// (register on page 1)
#define LTC2949_REG_E2TH_15_8 0x134

// (register on page 1)
#define LTC2949_REG_E2TH_7_0 0x135

// Energy 2 threshold low (register on page 1)
#define LTC2949_REG_E2TL_47_40 0x136

// (register on page 1)
#define LTC2949_REG_E2TL_39_32 0x137

// (register on page 1)
#define LTC2949_REG_E2TL_31_24 0x138

// (register on page 1)
#define LTC2949_REG_E2TL_23_16 0x139

// (register on page 1)
#define LTC2949_REG_E2TL_15_8 0x13A

// (register on page 1)
#define LTC2949_REG_E2TL_7_0 0x13B

// Charge3 threshold high (register on page 1)
#define LTC2949_REG_C3TH_63_56 0x144

// (register on page 1)
#define LTC2949_REG_C3TH_55_48 0x145

// (register on page 1)
#define LTC2949_REG_C3TH_47_40 0x146

// (register on page 1)
#define LTC2949_REG_C3TH_39_32 0x147

// (register on page 1)
#define LTC2949_REG_C3TH_31_24 0x148

// (register on page 1)
#define LTC2949_REG_C3TH_23_16 0x149

// (register on page 1)
#define LTC2949_REG_C3TH_15_8 0x14A

// (register on page 1)
#define LTC2949_REG_C3TH_7_0 0x14B

// Time Base 3 threshold high (register on page 1)
#define LTC2949_REG_TB3TH_31_24 0x14C

// (register on page 1)
#define LTC2949_REG_TB3TH_23_16 0x14D

// (register on page 1)
#define LTC2949_REG_TB3TH_15_8 0x14E

// (register on page 1)
#define LTC2949_REG_TB3TH_7_0 0x14F

// Charge3 threshold low (register on page 1)
#define LTC2949_REG_C3TL_63_56 0x154

// (register on page 1)
#define LTC2949_REG_C3TL_55_48 0x155

// (register on page 1)
#define LTC2949_REG_C3TL_47_40 0x156

// (register on page 1)
#define LTC2949_REG_C3TL_39_32 0x157

// (register on page 1)
#define LTC2949_REG_C3TL_31_24 0x158

// (register on page 1)
#define LTC2949_REG_C3TL_23_16 0x159

// (register on page 1)
#define LTC2949_REG_C3TL_15_8 0x15A

// (register on page 1)
#define LTC2949_REG_C3TL_7_0 0x15B

// Temperature coefficient of Current1 sense resistor 2nd order (register on page 1)
#define LTC2949_REG_RS1TC2_23_16 0x15C

// (register on page 1)
#define LTC2949_REG_RS1TC2_15_8 0x15D

// (register on page 1)
#define LTC2949_REG_RS1TC2_7_0 0x15E

// Energy 4 threshold high (register on page 1)
#define LTC2949_REG_E4TH_63_56 0x164

// (register on page 1)
#define LTC2949_REG_E4TH_55_48 0x165

// (register on page 1)
#define LTC2949_REG_E4TH_47_40 0x166

// (register on page 1)
#define LTC2949_REG_E4TH_39_32 0x167

// (register on page 1)
#define LTC2949_REG_E4TH_31_24 0x168

// (register on page 1)
#define LTC2949_REG_E4TH_23_16 0x169

// (register on page 1)
#define LTC2949_REG_E4TH_15_8 0x16A

// (register on page 1)
#define LTC2949_REG_E4TH_7_0 0x16B

// Time Base 4 threshold high (register on page 1)
#define LTC2949_REG_TB4TH_31_24 0x16C

// (register on page 1)
#define LTC2949_REG_TB4TH_23_16 0x16D

// (register on page 1)
#define LTC2949_REG_TB4TH_15_8 0x16E

// (register on page 1)
#define LTC2949_REG_TB4TH_7_0 0x16F

// Energy 4 threshold low (register on page 1)
#define LTC2949_REG_E4TL_63_56 0x174

// (register on page 1)
#define LTC2949_REG_E4TL_55_48 0x175

// (register on page 1)
#define LTC2949_REG_E4TL_47_40 0x176

// (register on page 1)
#define LTC2949_REG_E4TL_39_32 0x177

// (register on page 1)
#define LTC2949_REG_E4TL_31_24 0x178

// (register on page 1)
#define LTC2949_REG_E4TL_23_16 0x179

// (register on page 1)
#define LTC2949_REG_E4TL_15_8 0x17A

// (register on page 1)
#define LTC2949_REG_E4TL_7_0 0x17B

// Temperature coefficient of Current2 sense resistor 2nd order (register on page 1)
#define LTC2949_REG_RS2TC2_23_16 0x17C

// (register on page 1)
#define LTC2949_REG_RS2TC2_15_8 0x17D

// (register on page 1)
#define LTC2949_REG_RS2TC2_7_0 0x17E

// Current1 threshold high (register on page 1)
#define LTC2949_REG_I1TH_15_8 0x180

// (register on page 1)
#define LTC2949_REG_I1TH_7_0 0x181

// Current1 threshold low (register on page 1)
#define LTC2949_REG_I1TL_15_8 0x182

// (register on page 1)
#define LTC2949_REG_I1TL_7_0 0x183

// Power1 threshold high (register on page 1)
#define LTC2949_REG_P1TH_15_8 0x184

// (register on page 1)
#define LTC2949_REG_P1TH_7_0 0x185

// Power1 threshold low (register on page 1)
#define LTC2949_REG_P1TL_15_8 0x186

// (register on page 1)
#define LTC2949_REG_P1TL_7_0 0x187

// Current2 threshold high (register on page 1)
#define LTC2949_REG_I2TH_15_8 0x188

// (register on page 1)
#define LTC2949_REG_I2TH_7_0 0x189

// Current2 threshold low (register on page 1)
#define LTC2949_REG_I2TL_15_8 0x18A

// (register on page 1)
#define LTC2949_REG_I2TL_7_0 0x18B

// Power2 threshold high (register on page 1)
#define LTC2949_REG_P2TH_15_8 0x18C

// (register on page 1)
#define LTC2949_REG_P2TH_7_0 0x18D

// Power2 threshold low (register on page 1)
#define LTC2949_REG_P2TL_15_8 0x18E

// (register on page 1)
#define LTC2949_REG_P2TL_7_0 0x18F

// Voltage at BAT (BATP - BATM) threshold high (register on page 1)
#define LTC2949_REG_BATTH_15_8 0x190

// (register on page 1)
#define LTC2949_REG_BATTH_7_0 0x191

// Voltage at BAT (BATP - BATM) threshold low (register on page 1)
#define LTC2949_REG_BATTL_15_8 0x192

// (register on page 1)
#define LTC2949_REG_BATTL_7_0 0x193

// Temperature threshold high (register on page 1)
#define LTC2949_REG_TEMPTH_15_8 0x194

// (register on page 1)
#define LTC2949_REG_TEMPTH_7_0 0x195

// Temperature threshold low (register on page 1)
#define LTC2949_REG_TEMPTL_15_8 0x196

// (register on page 1)
#define LTC2949_REG_TEMPTL_7_0 0x197

// VCC Voltage threshold high (register on page 1)
#define LTC2949_REG_VCCTH_15_8 0x198

// (register on page 1)
#define LTC2949_REG_VCCTH_7_0 0x199

// VCC Voltage threshold low (register on page 1)
#define LTC2949_REG_VCCTL_15_8 0x19A

// (register on page 1)
#define LTC2949_REG_VCCTL_7_0 0x19B

// Slot1 Voltage threshold high (register on page 1)
#define LTC2949_REG_SLOT1TH_15_8 0x1A0

// (register on page 1)
#define LTC2949_REG_SLOT1TH_7_0 0x1A1

// Slot1 Voltage threshold low (register on page 1)
#define LTC2949_REG_SLOT1TL_15_8 0x1A2

// (register on page 1)
#define LTC2949_REG_SLOT1TL_7_0 0x1A3

// SLOT2 Voltage threshold high (register on page 1)
#define LTC2949_REG_SLOT2TH_15_8 0x1A4

// (register on page 1)
#define LTC2949_REG_SLOT2TH_7_0 0x1A5

// SLOT2 Voltage threshold low (register on page 1)
#define LTC2949_REG_SLOT2TL_15_8 0x1A6

// (register on page 1)
#define LTC2949_REG_SLOT2TL_7_0 0x1A7

// NTC Rref1, reference resistor to be connected between Vref and NTC1. 24bit float format (register on page 1)
#define LTC2949_REG_RREF1_23_16 0x1AA

// (register on page 1)
#define LTC2949_REG_RREF1_15_8 0x1AB

// (register on page 1)
#define LTC2949_REG_RREF1_7_0 0x1AC

// NTC Rref2, reference resistor to be connected between Vref and NTC2. F24: 24bit float format (register on page 1)
#define LTC2949_REG_RREF2_23_16 0x1AD

// (register on page 1)
#define LTC2949_REG_RREF2_15_8 0x1AE

// (register on page 1)
#define LTC2949_REG_RREF2_7_0 0x1AF

// Gain correction factor for sense resistor of Current1. F24: 24bit float format (register on page 1)
#define LTC2949_REG_RS1GC_23_16 0x1B0

// (register on page 1)
#define LTC2949_REG_RS1GC_15_8 0x1B1

// (register on page 1)
#define LTC2949_REG_RS1GC_7_0 0x1B2

// Gain correction factor for sense resistor of Current2. F24: 24bit float format (register on page 1)
#define LTC2949_REG_RS2GC_23_16 0x1B3

// (register on page 1)
#define LTC2949_REG_RS2GC_15_8 0x1B4

// (register on page 1)
#define LTC2949_REG_RS2GC_7_0 0x1B5

// Scaling factor for Current2/Current1 sense resistors. F24: 24bit float format (register on page 1)
#define LTC2949_REG_RSRATIO_23_16 0x1B6

// (register on page 1)
#define LTC2949_REG_RSRATIO_15_8 0x1B7

// (register on page 1)
#define LTC2949_REG_RSRATIO_7_0 0x1B8

// Gain correction of the resistor divider for BAT (affects also P1 and P2) F24: 24bit float format (register on page 1)
#define LTC2949_REG_BATGC_23_16 0x1B9

// (register on page 1)
#define LTC2949_REG_BATGC_15_8 0x1BA

// (register on page 1)
#define LTC2949_REG_BATGC_7_0 0x1BB

// Gain correction factor for MUX setting 1 (register on page 1)
#define LTC2949_REG_MUX1GC_23_16 0x1C0

// (register on page 1)
#define LTC2949_REG_MUX1GC_15_8 0x1C1

// (register on page 1)
#define LTC2949_REG_MUX1GC_7_0 0x1C2

// Gain correction factor for MUX setting 2 (register on page 1)
#define LTC2949_REG_MUX2GC_23_16 0x1C3

// (register on page 1)
#define LTC2949_REG_MUX2GC_15_8 0x1C4

// (register on page 1)
#define LTC2949_REG_MUX2GC_7_0 0x1C5

// Gain correction factor for MUX setting 3 (register on page 1)
#define LTC2949_REG_MUX3GC_23_16 0x1C6

// (register on page 1)
#define LTC2949_REG_MUX3GC_15_8 0x1C7

// (register on page 1)
#define LTC2949_REG_MUX3GC_7_0 0x1C8

// Gain correction factor for MUX setting 4 (register on page 1)
#define LTC2949_REG_MUX4GC_23_16 0x1C9

// (register on page 1)
#define LTC2949_REG_MUX4GC_15_8 0x1CA

// (register on page 1)
#define LTC2949_REG_MUX4GC_7_0 0x1CB

// NTC1 Coefficient A for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_REG_NTC1A_23_16 0x1D0

// (register on page 1)
#define LTC2949_REG_NTC1A_15_8 0x1D1

// (register on page 1)
#define LTC2949_REG_NTC1A_7_0 0x1D2

// NTC1 Coefficient B for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_REG_NTC1B_23_16 0x1D3

// (register on page 1)
#define LTC2949_REG_NTC1B_15_8 0x1D4

// (register on page 1)
#define LTC2949_REG_NTC1B_7_0 0x1D5

// NTC1 Coefficient C for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_REG_NTC1C_23_16 0x1D6

// (register on page 1)
#define LTC2949_REG_NTC1C_15_8 0x1D7

// (register on page 1)
#define LTC2949_REG_NTC1C_7_0 0x1D8

// Temperature coefficient of Current1 sense resistor (register on page 1)
#define LTC2949_REG_RS1TC_23_16 0x1D9

// (register on page 1)
#define LTC2949_REG_RS1TC_15_8 0x1DA

// (register on page 1)
#define LTC2949_REG_RS1TC_7_0 0x1DB

// Reference temperature for TC compensation of Current1 sense resistor (F24 format where bits 7..0 are all zero and not stored!) (register on page 1)
#define LTC2949_REG_RS1T0_15_8 0x1DC

// (register on page 1)
#define LTC2949_REG_RS1T0_7_0 0x1DD

// NTC2 Coefficient A for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_REG_NTC2A_23_16 0x1E0

// (register on page 1)
#define LTC2949_REG_NTC2A_15_8 0x1E1

// (register on page 1)
#define LTC2949_REG_NTC2A_7_0 0x1E2

// NTC2 Coefficient B for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_REG_NTC2B_23_16 0x1E3

// (register on page 1)
#define LTC2949_REG_NTC2B_15_8 0x1E4

// (register on page 1)
#define LTC2949_REG_NTC2B_7_0 0x1E5

// NTC2 Coefficient C for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_REG_NTC2C_23_16 0x1E6

// (register on page 1)
#define LTC2949_REG_NTC2C_15_8 0x1E7

// (register on page 1)
#define LTC2949_REG_NTC2C_7_0 0x1E8

// Temperature coefficient of Current2 sense resistor (register on page 1)
#define LTC2949_REG_RS2TC_23_16 0x1E9

// (register on page 1)
#define LTC2949_REG_RS2TC_15_8 0x1EA

// (register on page 1)
#define LTC2949_REG_RS2TC_7_0 0x1EB

// Reference temperature for TC compensation of Current2 sense resistor (F24 format where bits 7..0 are all zero and not stored!) (register on page 1)
#define LTC2949_REG_RS2T0_15_8 0x1EC

// (register on page 1)
#define LTC2949_REG_RS2T0_7_0 0x1ED

/////////////////////////////////////////////////////////
// Definitions of all available multiple byte values   //
// (e.g. accumulators).                                //
//                                                     //
// Note: The address constants are 9 bit values where  //
// the lower 8 bit define the address and the 9th bit  //
// defines the page!                                   //
/////////////////////////////////////////////////////////

// Charge1 (register on page 0)
#define LTC2949_VAL_C1 0x000

// Energy1 (register on page 0)
#define LTC2949_VAL_E1 0x006

// Time1 (register on page 0)
#define LTC2949_VAL_TB1 0x00C

// Charge2 (register on page 0)
#define LTC2949_VAL_C2 0x010

// Energy2 (register on page 0)
#define LTC2949_VAL_E2 0x016

// Time2 (register on page 0)
#define LTC2949_VAL_TB2 0x01C

// Charge3 (register on page 0)
#define LTC2949_VAL_C3 0x024

// Time3 (register on page 0)
#define LTC2949_VAL_TB3 0x02C

// Energy4 (register on page 0)
#define LTC2949_VAL_E4 0x034

// Time4 (register on page 0)
#define LTC2949_VAL_TB4 0x03C

// Maximum Current1 tracking (register on page 0)
#define LTC2949_VAL_I1MAX 0x040

// Minimum Current1 tracking (register on page 0)
#define LTC2949_VAL_I1MIN 0x042

// Maximum Power1 tracking (register on page 0)
#define LTC2949_VAL_P1MAX 0x044

// Minimum Power1 tracking (register on page 0)
#define LTC2949_VAL_P1MIN 0x046

// Maximum Current2 tracking (register on page 0)
#define LTC2949_VAL_I2MAX 0x048

// Minimum Current2 tracking (register on page 0)
#define LTC2949_VAL_I2MIN 0x04A

// Maximum Power2 tracking (register on page 0)
#define LTC2949_VAL_P2MAX 0x04C

// Minimum Power2 tracking (register on page 0)
#define LTC2949_VAL_P2MIN 0x04E

// Maximum Voltage at BAT (BATP - BATM) tracking (register on page 0)
#define LTC2949_VAL_BATMAX 0x050

// Minimum Voltage at BAT (BATP - BATM) tracking (register on page 0)
#define LTC2949_VAL_BATMIN 0x052

// Maximum Temperature tracking (register on page 0)
#define LTC2949_VAL_TEMPMAX 0x054

// Minimum Temperature tracking (register on page 0)
#define LTC2949_VAL_TEMPMIN 0x056

// Maximum VCC voltage tracking (register on page 0)
#define LTC2949_VAL_VCCMAX 0x058

// Minimum VCC voltage tracking (register on page 0)
#define LTC2949_VAL_VCCMIN 0x05A

// Maximum Voltage of Slot1 (register on page 0)
#define LTC2949_VAL_SLOT1MAX 0x05C

// Minimum Voltage at Slot1 (register on page 0)
#define LTC2949_VAL_SLOT1MIN 0x05E

// Maximum Voltage of Slot2 (register on page 0)
#define LTC2949_VAL_SLOT2MAX 0x060

// Minimum Voltage at Slot2 (register on page 0)
#define LTC2949_VAL_SLOT2MIN 0x062

// RAW Current1 (register on page 0)
#define LTC2949_VAL_I1RAW 0x079

// RAW Current2 (register on page 0)
#define LTC2949_VAL_I2RAW 0x07C

// Current1 (register on page 0)
#define LTC2949_VAL_I1 0x090

// Power1 (register on page 0)
#define LTC2949_VAL_P1 0x093

// Current2 (register on page 0)
#define LTC2949_VAL_I2 0x096

// Power2 (register on page 0)
#define LTC2949_VAL_P2 0x099

// Average of Current1: (n-1)+(n-2)+(n-3)+(n-4) (register on page 0)
#define LTC2949_VAL_IAVG1 0x09C

// Voltage at BAT (BATP - BATM) (register on page 0)
#define LTC2949_VAL_BAT 0x0A0

// Temperature (register on page 0)
#define LTC2949_VAL_TEMP 0x0A2

// Voltage at AVCC (register on page 0)
#define LTC2949_VAL_VCC 0x0A4

// Voltage of SLOT1 (register on page 0)
#define LTC2949_VAL_SLOT1 0x0A6

// Voltage of SLOT2 (register on page 0)
#define LTC2949_VAL_SLOT2 0x0A8

// Vref (register on page 0)
#define LTC2949_VAL_VREF 0x0AA

// Average of Current2: (n-1)+(n-2)+(n-3)+(n-4) (register on page 0)
#define LTC2949_VAL_IAVG2 0x0AC

// Current1 moving avergae 4 (register on page 0)
#define LTC2949_VAL_I1AVG 0x0B0

// Current1 History 1 (prev. result '-1') (register on page 0)
#define LTC2949_VAL_I1H1 0x0B3

// Current1 History 2 (prev. result '-1') (register on page 0)
#define LTC2949_VAL_I1H2 0x0B6

// Current1 History 3 (prev. result '-2') (register on page 0)
#define LTC2949_VAL_I1H3 0x0B9

// Current1 History 4 (prev. result '-3') (register on page 0)
#define LTC2949_VAL_I1H4 0x0BC

// Current2 moving avergae 4 (register on page 0)
#define LTC2949_VAL_I2AVG 0x0C0

// Current2 History 1 (prev. result '-1') (register on page 0)
#define LTC2949_VAL_I2H1 0x0C3

// Current2 History 2 (prev. result '-1') (register on page 0)
#define LTC2949_VAL_I2H2 0x0C6

// Current2 History 3 (prev. result '-2') (register on page 0)
#define LTC2949_VAL_I2H3 0x0C9

// Current2 History 4 (prev. result '-3') (register on page 0)
#define LTC2949_VAL_I2H4 0x0CC

// Charge 1 threshold high (register on page 1)
#define LTC2949_VAL_C1TH 0x100

// Charge 1 threshold low (register on page 1)
#define LTC2949_VAL_C1TL 0x106

// Time Base 1 threshold high (register on page 1)
#define LTC2949_VAL_TB1TH 0x10C

// Energy 1 threshold high (register on page 1)
#define LTC2949_VAL_E1TH 0x110

// Energy 1 threshold low (register on page 1)
#define LTC2949_VAL_E1TL 0x116

// Charge 2 threshold high (register on page 1)
#define LTC2949_VAL_C2TH 0x120

// Charge 2 threshold low (register on page 1)
#define LTC2949_VAL_C2TL 0x126

// Time Base 2 threshold high (register on page 1)
#define LTC2949_VAL_TB2TH 0x12C

// Energy 2 threshold high (register on page 1)
#define LTC2949_VAL_E2TH 0x130

// Energy 2 threshold low (register on page 1)
#define LTC2949_VAL_E2TL 0x136

// Charge3 threshold high (register on page 1)
#define LTC2949_VAL_C3TH 0x144

// Time Base 3 threshold high (register on page 1)
#define LTC2949_VAL_TB3TH 0x14C

// Charge3 threshold low (register on page 1)
#define LTC2949_VAL_C3TL 0x154

// Temperature coefficient of Current1 sense resistor 2nd order (register on page 1)
#define LTC2949_VAL_RS1TC2 0x15C

// Energy 4 threshold high (register on page 1)
#define LTC2949_VAL_E4TH 0x164

// Time Base 4 threshold high (register on page 1)
#define LTC2949_VAL_TB4TH 0x16C

// Energy 4 threshold low (register on page 1)
#define LTC2949_VAL_E4TL 0x174

// Temperature coefficient of Current2 sense resistor 2nd order (register on page 1)
#define LTC2949_VAL_RS2TC2 0x17C

// Current1 threshold high (register on page 1)
#define LTC2949_VAL_I1TH 0x180

// Current1 threshold low (register on page 1)
#define LTC2949_VAL_I1TL 0x182

// Power1 threshold high (register on page 1)
#define LTC2949_VAL_P1TH 0x184

// Power1 threshold low (register on page 1)
#define LTC2949_VAL_P1TL 0x186

// Current2 threshold high (register on page 1)
#define LTC2949_VAL_I2TH 0x188

// Current2 threshold low (register on page 1)
#define LTC2949_VAL_I2TL 0x18A

// Power2 threshold high (register on page 1)
#define LTC2949_VAL_P2TH 0x18C

// Power2 threshold low (register on page 1)
#define LTC2949_VAL_P2TL 0x18E

// Voltage at BAT (BATP - BATM) threshold high (register on page 1)
#define LTC2949_VAL_BATTH 0x190

// Voltage at BAT (BATP - BATM) threshold low (register on page 1)
#define LTC2949_VAL_BATTL 0x192

// Temperature threshold high (register on page 1)
#define LTC2949_VAL_TEMPTH 0x194

// Temperature threshold low (register on page 1)
#define LTC2949_VAL_TEMPTL 0x196

// VCC Voltage threshold high (register on page 1)
#define LTC2949_VAL_VCCTH 0x198

// VCC Voltage threshold low (register on page 1)
#define LTC2949_VAL_VCCTL 0x19A

// Slot1 Voltage threshold high (register on page 1)
#define LTC2949_VAL_SLOT1TH 0x1A0

// Slot1 Voltage threshold low (register on page 1)
#define LTC2949_VAL_SLOT1TL 0x1A2

// SLOT2 Voltage threshold high (register on page 1)
#define LTC2949_VAL_SLOT2TH 0x1A4

// SLOT2 Voltage threshold low (register on page 1)
#define LTC2949_VAL_SLOT2TL 0x1A6

// NTC Rref1, reference resistor to be connected between Vref and NTC1. 24bit float format (register on page 1)
#define LTC2949_VAL_RREF1 0x1AA

// NTC Rref2, reference resistor to be connected between Vref and NTC2. F24: 24bit float format (register on page 1)
#define LTC2949_VAL_RREF2 0x1AD

// Gain correction factor for sense resistor of Current1. F24: 24bit float format (register on page 1)
#define LTC2949_VAL_RS1GC 0x1B0

// Gain correction factor for sense resistor of Current2. F24: 24bit float format (register on page 1)
#define LTC2949_VAL_RS2GC 0x1B3

// Scaling factor for Current2/Current1 sense resistors. F24: 24bit float format (register on page 1)
#define LTC2949_VAL_RSRATIO 0x1B6

// Gain correction of the resistor divider for BAT (affects also P1 and P2) F24: 24bit float format (register on page 1)
#define LTC2949_VAL_BATGC 0x1B9

// Gain correction factor for MUX setting 1 (register on page 1)
#define LTC2949_VAL_MUX1GC 0x1C0

// Gain correction factor for MUX setting 2 (register on page 1)
#define LTC2949_VAL_MUX2GC 0x1C3

// Gain correction factor for MUX setting 3 (register on page 1)
#define LTC2949_VAL_MUX3GC 0x1C6

// Gain correction factor for MUX setting 4 (register on page 1)
#define LTC2949_VAL_MUX4GC 0x1C9

// NTC1 Coefficient A for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_VAL_NTC1A 0x1D0

// NTC1 Coefficient B for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_VAL_NTC1B 0x1D3

// NTC1 Coefficient C for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_VAL_NTC1C 0x1D6

// Temperature coefficient of Current1 sense resistor (register on page 1)
#define LTC2949_VAL_RS1TC 0x1D9

// Reference temperature for TC compensation of Current1 sense resistor (F24 format where bits 7..0 are all zero and not stored!) (register on page 1)
#define LTC2949_VAL_RS1T0 0x1DC

// NTC2 Coefficient A for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_VAL_NTC2A 0x1E0

// NTC2 Coefficient B for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_VAL_NTC2B 0x1E3

// NTC2 Coefficient C for calculation of temperature out of NTC resistance (register on page 1)
#define LTC2949_VAL_NTC2C 0x1E6

// Temperature coefficient of Current2 sense resistor (register on page 1)
#define LTC2949_VAL_RS2TC 0x1E9

// Reference temperature for TC compensation of Current2 sense resistor (F24 format where bits 7..0 are all zero and not stored!) (register on page 1)
#define LTC2949_VAL_RS2T0 0x1EC

/////////////////////////////////////////////////////////
// Definition of LSB constants.                        //
/////////////////////////////////////////////////////////
// LTC2949_TBFAC must be set depending on TBCTRL configuration and external clock. See function LTC2949_WriteTbctl of LTC2949.cpp
#define LTC2949_MIN_EXTCLK 30e3
#define LTC2949_MAX_EXTCLK 25e6
#define LTC2949_PRE0_MAX_EXTCLK 1e6
#define LTC2949_PRE1_MAX_EXTCLK 2e6
#define LTC2949_PRE2_MAX_EXTCLK 4e6
#define LTC2949_PRE3_MAX_EXTCLK 8e6
#define LTC2949_PRE4_MAX_EXTCLK 16e6
#define LTC2949_INTC 31e-6
#define LTC2949_TBC_NOM_FREQU 32768.0

// LSB of C1 in Vs
#define LTC2949_LSB_C1           (1.21899e-05 * LTC2949_TBFAC)
// LSB of E1 in V²s
#define LTC2949_LSB_E1           (7.4895e-05 * LTC2949_TBFAC)
// LSB of TB1 in s
#define LTC2949_LSB_TB1          (12.8315 * LTC2949_TBFAC)
// LSB of C2 in Vs
#define LTC2949_LSB_C2           (1.21899e-05 * LTC2949_TBFAC)
// LSB of E2 in V²s
#define LTC2949_LSB_E2           (7.4895e-05 * LTC2949_TBFAC)
// LSB of TB2 in s
#define LTC2949_LSB_TB2          (12.8315 * LTC2949_TBFAC)
// LSB of C3 in Vs
#define LTC2949_LSB_C3           (1.21899e-05 * LTC2949_TBFAC)
// LSB of TB3 in s
#define LTC2949_LSB_TB3          (12.8315 * LTC2949_TBFAC)
// LSB of E4 in V²s
#define LTC2949_LSB_E4           (7.4895e-05 * LTC2949_TBFAC)
// LSB of TB4 in s
#define LTC2949_LSB_TB4          (12.8315 * LTC2949_TBFAC)
// LSB of I1MAX in V
#define LTC2949_LSB_I1MAX        3.8e-06
// LSB of I1MIN in V
#define LTC2949_LSB_I1MIN        3.8e-06
// LSB of P1MAX in V²
#define LTC2949_LSB_P1MAX        2.33472e-05
// LSB of P1MIN in V²
#define LTC2949_LSB_P1MIN        2.33472e-05
// LSB of P1MAX as Voltage in V
#define LTC2949_LSB_P1ASVMAX    (LTC2949_LSB_SLOT1MAX/8.0*4)
// LSB of P1MIN as Voltage in V
#define LTC2949_LSB_P1ASVMIN    (LTC2949_LSB_SLOT1MIN/8.0*4)
// LSB of I2MAX in V
#define LTC2949_LSB_I2MAX        3.8e-06
// LSB of I2MIN in V
#define LTC2949_LSB_I2MIN        3.8e-06
// LSB of P2MAX in V²
#define LTC2949_LSB_P2MAX        2.33472e-05
// LSB of P2MIN in V²
#define LTC2949_LSB_P2MIN        2.33472e-05
// LSB of P1MAX as Voltage in V
#define LTC2949_LSB_P2ASVMAX    (LTC2949_LSB_SLOT2MAX/8.0*4)
// LSB of P1MIN as Voltage in V
#define LTC2949_LSB_P2ASVMIN    (LTC2949_LSB_SLOT2MIN/8.0*4)
// LSB of BATMAX in V
#define LTC2949_LSB_BATMAX       3.75e-04
// LSB of BATMIN in V
#define LTC2949_LSB_BATMIN       3.75e-04
// LSB of TEMPMAX in °C
#define LTC2949_LSB_TEMPMAX      0.2
// LSB of TEMPMIN in °C
#define LTC2949_LSB_TEMPMIN      0.2
// LSB of VCCMAX in V
#define LTC2949_LSB_VCCMAX       0.0022583
// LSB of VCCMIN in V
#define LTC2949_LSB_VCCMIN       0.0022583
// LSB of SLOT1MAX in V
#define LTC2949_LSB_SLOT1MAX     3.75e-04
// LSB of SLOT1MIN in V
#define LTC2949_LSB_SLOT1MIN     3.75e-04
// LSB of SLOT2MAX in V
#define LTC2949_LSB_SLOT2MAX     3.75e-04
// LSB of SLOT2MIN in V
#define LTC2949_LSB_SLOT2MIN     3.75e-04
// LSB of I1 in V
#define LTC2949_LSB_I1           9.5e-07
// LSB of P1 in V²
#define LTC2949_LSB_P1           5.8368e-06
// LSB of P1 as Voltage ADC in V
#define LTC2949_LSB_P1ASV        (LTC2949_LSB_SLOT1/8.0)
// LSB of I2 in V
#define LTC2949_LSB_I2           9.5e-07
// LSB of P2 in V²
#define LTC2949_LSB_P2           5.8368e-06
// LSB of P2 as Voltage ADC in V
#define LTC2949_LSB_P2ASV        (LTC2949_LSB_SLOT2/8.0)
// LSB of IAVG1 in V
#define LTC2949_LSB_IAVG1        2.375e-07
// LSB of BAT in V
#define LTC2949_LSB_BAT          3.75e-04
// LSB of TEMP in °C
#define LTC2949_LSB_TEMP         0.2
// LSB of VCC in V
#define LTC2949_LSB_VCC          0.0022583
// LSB of SLOT1 in V
#define LTC2949_LSB_SLOT1        3.75e-04
// LSB of SLOT2 in V
#define LTC2949_LSB_SLOT2        3.75e-04
// LSB of VREF in V
#define LTC2949_LSB_VREF         3.75e-04
// LSB of IAVG2 in V
#define LTC2949_LSB_IAVG2        2.375e-07
// LSB of I1AVG in V
#define LTC2949_LSB_I1AVG        2.375e-07
// LSB of I1H1 in V
#define LTC2949_LSB_I1H1         9.5e-07
// LSB of I1H2 in V
#define LTC2949_LSB_I1H2         9.5e-07
// LSB of I1H3 in V
#define LTC2949_LSB_I1H3         9.5e-07
// LSB of I1H4 in V
#define LTC2949_LSB_I1H4         9.5e-07
// LSB of I2AVG in V
#define LTC2949_LSB_I2AVG        2.375e-07
// LSB of I2H1 in V
#define LTC2949_LSB_I2H1         9.5e-07
// LSB of I2H2 in V
#define LTC2949_LSB_I2H2         9.5e-07
// LSB of I2H3 in V
#define LTC2949_LSB_I2H3         9.5e-07
// LSB of I2H4 in V
#define LTC2949_LSB_I2H4         9.5e-07
// LSB of C1TH in Vs
#define LTC2949_LSB_C1TH         (1.21899e-05 * LTC2949_TBFAC)
// LSB of C1TL in Vs
#define LTC2949_LSB_C1TL         (1.21899e-05 * LTC2949_TBFAC)
// LSB of TB1TH in s
#define LTC2949_LSB_TB1TH        (12.8315 * LTC2949_TBFAC)
// LSB of E1TH in V²s
#define LTC2949_LSB_E1TH         (7.4895e-05 * LTC2949_TBFAC)
// LSB of E1TL in V²s
#define LTC2949_LSB_E1TL         (7.4895e-05 * LTC2949_TBFAC)
// LSB of C2TH in Vs
#define LTC2949_LSB_C2TH         (1.21899e-05 * LTC2949_TBFAC)
// LSB of C2TL in Vs
#define LTC2949_LSB_C2TL         (1.21899e-05 * LTC2949_TBFAC)
// LSB of TB2TH in s
#define LTC2949_LSB_TB2TH        (12.8315 * LTC2949_TBFAC)
// LSB of E2TH in V²s
#define LTC2949_LSB_E2TH         (7.4895e-05 * LTC2949_TBFAC)
// LSB of E2TL in V²s
#define LTC2949_LSB_E2TL         (7.4895e-05 * LTC2949_TBFAC)
// LSB of C3TH in Vs
#define LTC2949_LSB_C3TH         (1.21899e-05 * LTC2949_TBFAC)
// LSB of TB3TH in s
#define LTC2949_LSB_TB3TH        (12.8315 * LTC2949_TBFAC)
// LSB of C3TL in Vs
#define LTC2949_LSB_C3TL         (1.21899e-05 * LTC2949_TBFAC)
// LSB of RS1TC2 in 1/K
#define LTC2949_LSB_RS1TC2       1.0
// LSB of E4TH in V²s
#define LTC2949_LSB_E4TH         (7.4895e-05 * LTC2949_TBFAC)
// LSB of TB4TH in s
#define LTC2949_LSB_TB4TH        (12.8315 * LTC2949_TBFAC)
// LSB of E4TL in V²s
#define LTC2949_LSB_E4TL         (7.4895e-05 * LTC2949_TBFAC)
// LSB of RS2TC2 in 1/K
#define LTC2949_LSB_RS2TC2       1.0
// LSB of I1TH in V
#define LTC2949_LSB_I1TH         3.8e-06
// LSB of I1TL in V
#define LTC2949_LSB_I1TL         3.8e-06
// LSB of P1TH in V²
#define LTC2949_LSB_P1TH         2.33472e-05
// LSB of P1TL in V²
#define LTC2949_LSB_P1TL         2.33472e-05
// LSB of I2TH in V
#define LTC2949_LSB_I2TH         3.8e-06
// LSB of I2TL in V
#define LTC2949_LSB_I2TL         3.8e-06
// LSB of P2TH in V²
#define LTC2949_LSB_P2TH         2.33472e-05
// LSB of P2TL in V²
#define LTC2949_LSB_P2TL         2.33472e-05
// LSB of BATTH in V
#define LTC2949_LSB_BATTH        3.75e-04
// LSB of BATTL in V
#define LTC2949_LSB_BATTL        3.75e-04
// LSB of TEMPTH in °C
#define LTC2949_LSB_TEMPTH       0.2
// LSB of TEMPTL in °C
#define LTC2949_LSB_TEMPTL       0.2
// LSB of VCCTH in V
#define LTC2949_LSB_VCCTH        0.0022583
// LSB of VCCTL in V
#define LTC2949_LSB_VCCTL        0.0022583
// LSB of SLOT1TH in V
#define LTC2949_LSB_SLOT1TH      3.75e-04
// LSB of SLOT1TL in V
#define LTC2949_LSB_SLOT1TL      3.75e-04
// LSB of SLOT2TH in V
#define LTC2949_LSB_SLOT2TH      3.75e-04
// LSB of SLOT2TL in V
#define LTC2949_LSB_SLOT2TL      3.75e-04
// LSB of RREF1 in Ohm
#define LTC2949_LSB_RREF1        1.0
// LSB of RREF2 in Ohm
#define LTC2949_LSB_RREF2        1.0
// LSB of RS1GC in 
#define LTC2949_LSB_RS1GC        1.0
// LSB of RS2GC in 
#define LTC2949_LSB_RS2GC        1.0
// LSB of RSRATIO in 
#define LTC2949_LSB_RSRATIO      1.0
// LSB of BATGC in 
#define LTC2949_LSB_BATGC        1.0
// LSB of MUX1GC in 
#define LTC2949_LSB_MUX1GC       1.0
// LSB of MUX2GC in 
#define LTC2949_LSB_MUX2GC       1.0
// LSB of MUX3GC in 
#define LTC2949_LSB_MUX3GC       1.0
// LSB of MUX4GC in 
#define LTC2949_LSB_MUX4GC       1.0
// LSB of NTC1A in 
#define LTC2949_LSB_NTC1A        1.0
// LSB of NTC1B in 
#define LTC2949_LSB_NTC1B        1.0
// LSB of NTC1C in 
#define LTC2949_LSB_NTC1C        1.0
// LSB of RS1TC in 1/K
#define LTC2949_LSB_RS1TC        1.0
// LSB of RS1T0 in °C
#define LTC2949_LSB_RS1T0        1.0
// LSB of NTC2A in 
#define LTC2949_LSB_NTC2A        1.0
// LSB of NTC2B in 
#define LTC2949_LSB_NTC2B        1.0
// LSB of NTC2C in 
#define LTC2949_LSB_NTC2C        1.0
// LSB of RS2TC in 1/K
#define LTC2949_LSB_RS2TC        1.0
// LSB of RS2T0 in °C
#define LTC2949_LSB_RS2T0        1.0
// LSB of FIFOI1 in V
#define LTC2949_LSB_FIFOI1       7.60371e-06
// LSB of FIFOI2 in V
#define LTC2949_LSB_FIFOI2       7.60371e-06
// LSB of FIFOBAT in V
#define LTC2949_LSB_FIFOBAT      3.75183e-04
// LSB of FIFOAUX in V
#define LTC2949_LSB_FIFOAUX      3.75183e-04

// Note: there is only one set of fast LSBs, no matter if we read from FIFO or single shot via RDCV.
// just for better readability we define one fest for fast LSBs here:
// LSB of FAST I1 in V
#define LTC2949_LSB_FI1       LTC2949_LSB_FIFOI1
// LSB of FAST I2 in V
#define LTC2949_LSB_FI2       LTC2949_LSB_FIFOI2
// LSB of FAST BAT in V
#define LTC2949_LSB_FBAT      LTC2949_LSB_FIFOBAT
// LSB of FAST AUX in V
#define LTC2949_LSB_FAUX      LTC2949_LSB_FIFOAUX

/////////////////////////////////////////////////////////
// Definition of all bit masks.                        //
/////////////////////////////////////////////////////////

// STATUS bit masks
// 1: Undervoltage in the analog domain including ADCs
#define LTC2949_BM_STATUS_UVLOA 0x1
// 1: Power On Reset due to undervoltage in the analog domain.
#define LTC2949_BM_STATUS_PORA 0x2
// 1: Undervoltage in the standby domain
#define LTC2949_BM_STATUS_UVLOSTBY 0x4
// 1: Undervoltage in the digital domain
#define LTC2949_BM_STATUS_UVLOD 0x8
// 1: Result registers have been updated
#define LTC2949_BM_STATUS_UPDATE 0x10
// ADC Error
#define LTC2949_BM_STATUS_ADCERR 0x20
// 1: External clock provided does not fit to TBC settings.
// Also LSB values of accumulated registers are wrong.
#define LTC2949_BM_STATUS_TBERR 0x40

// STATVT bit masks
// 1: Voltage at BAT (BATP - BATM) high threshold exceeded
#define LTC2949_BM_STATVT_BATH 0x1
// 1: Voltage at BAT (BATP - BATM) low threshold exceeded
#define LTC2949_BM_STATVT_BATL 0x2
// 1: Temperature high threshold exceeded
#define LTC2949_BM_STATVT_TEMPH 0x4
// 1: Temperature low threshold exceeded
#define LTC2949_BM_STATVT_TEMPL 0x8
// 1: Slot1 high threshold exceeded
#define LTC2949_BM_STATVT_SLOT1H 0x10
// 1: Slot1 low threshold exceeded
#define LTC2949_BM_STATVT_SLOT1L 0x20
// 1: Slot2 high threshold exceeded
#define LTC2949_BM_STATVT_SLOT2H 0x40
// 1: Slot2 low threshold exceeded
#define LTC2949_BM_STATVT_SLOT2L 0x80

// STATIP bit masks
// 1: Current1 high threshold exceeded
#define LTC2949_BM_STATIP_I1H 0x1
// 1: Current1 low threshold exceeded
#define LTC2949_BM_STATIP_I1L 0x2
// 1: Power1 high threshold exceeded
#define LTC2949_BM_STATIP_P1H 0x4
// 1: Power1 low threshold exceeded
#define LTC2949_BM_STATIP_P1L 0x8
// 1: Current2 high threshold exceeded
#define LTC2949_BM_STATIP_I2H 0x10
// 1: Current2 low threshold exceeded
#define LTC2949_BM_STATIP_I2L 0x20
// 1: Power2 high threshold exceeded
#define LTC2949_BM_STATIP_P2H 0x40
// 1: Power2 low threshold exceeded
#define LTC2949_BM_STATIP_P2L 0x80

// STATC bit masks
// 1: Charge1 high threshold exceeded
#define LTC2949_BM_STATC_C1H 0x1
// 1: Charge1 low threshold exceeded
#define LTC2949_BM_STATC_C1L 0x2
// 1: Charge2 high threshold exceeded
#define LTC2949_BM_STATC_C2H 0x4
// 1: Charge2 low threshold exceeded
#define LTC2949_BM_STATC_C2L 0x8
// 1: Charge3 high threshold exceeded
#define LTC2949_BM_STATC_C3H 0x10
// 1: Charge3 low threshold exceeded
#define LTC2949_BM_STATC_C3L 0x20

// STATE bit masks
// 1: Energy1 high threshold exceeded
#define LTC2949_BM_STATE_E1H 0x1
// 1: Energy1 low threshold exceeded
#define LTC2949_BM_STATE_E1L 0x2
// 1: Energy2 high threshold exceeded
#define LTC2949_BM_STATE_E2H 0x4
// 1: Energy2 low threshold exceeded
#define LTC2949_BM_STATE_E2L 0x8
// 1: Energy4 high threshold exceeded
#define LTC2949_BM_STATE_E4H 0x40
// 1: Energy4 low threshold exceeded
#define LTC2949_BM_STATE_E4L 0x80

// STATCEOF bit masks
// 1: Charge1 overflow alert
#define LTC2949_BM_STATCEOF_C1OF 0x1
// 1: Charge2 overflow alert
#define LTC2949_BM_STATCEOF_C2OF 0x2
// 1: Charge3 overflow alert
#define LTC2949_BM_STATCEOF_C3OF 0x4
// 1: Energy1 overflow alert
#define LTC2949_BM_STATCEOF_E1OF 0x10
// 1: Energy2 overflow alert
#define LTC2949_BM_STATCEOF_E2OF 0x20
// 1: Energy4 overflow alert
#define LTC2949_BM_STATCEOF_E4OF 0x80

// STATTB bit masks
// 1: Time1 threshold exceeded
#define LTC2949_BM_STATTB_TB1TH 0x1
// 1: Time2 threshold exceeded
#define LTC2949_BM_STATTB_TB2TH 0x2
// 1: Time3 threshold exceeded
#define LTC2949_BM_STATTB_TB3TH 0x4
// 1: Time4 threshold exceeded
#define LTC2949_BM_STATTB_TB4TH 0x8
// 1: Time1 overflow
#define LTC2949_BM_STATTB_TB1OF 0x10
// 1: Time 2 overflow
#define LTC2949_BM_STATTB_TB2OF 0x20
// 1: Time 3 overflow
#define LTC2949_BM_STATTB_TB3OF 0x40
// 1: Time 4 overflow
#define LTC2949_BM_STATTB_TB4OF 0x80

// STATVCC bit masks
// 1: VCC high threshold exceeded
#define LTC2949_BM_STATVCC_VCCH 0x1
// 1: VCC low threshold exceeded
#define LTC2949_BM_STATVCC_VCCL 0x2
// Current1 violated DACO1 threshold for more than deglitch time
#define LTC2949_BM_STATVCC_OCC1 0x4
// Current2 violated DACO2 threshold for more than deglitch time
#define LTC2949_BM_STATVCC_OCC2 0x8

// STATUSM bit masks
// UVLOA mask
#define LTC2949_BM_STATUSM_UVLOAM 0x1
// UVLOD mask
#define LTC2949_BM_STATUSM_UVLODM 0x8
// UPDATE mask
#define LTC2949_BM_STATUSM_UPDATEM 0x10
// ADCERR mask
#define LTC2949_BM_STATUSM_ADCERRM 0x20
// TBERR mask
#define LTC2949_BM_STATUSM_TBERRM 0x40

// STATVTM bit masks
// BATH mask
#define LTC2949_BM_STATVTM_BATHM 0x1
// BATL mask
#define LTC2949_BM_STATVTM_BATLM 0x2
// TEMPH mask
#define LTC2949_BM_STATVTM_TEMPHM 0x4
// TEMPL mask
#define LTC2949_BM_STATVTM_TEMPLM 0x8
// SLOT1H mask
#define LTC2949_BM_STATVTM_SLOT1HM 0x10
// SLOT1L mask
#define LTC2949_BM_STATVTM_SLOT1LM 0x20
// SLOT2H mask
#define LTC2949_BM_STATVTM_SLOT2HM 0x40
// SLOT2L mask
#define LTC2949_BM_STATVTM_SLOT2LM 0x80

// STATIPM bit masks
// I1H mask
#define LTC2949_BM_STATIPM_I1HM 0x1
// I1L mask
#define LTC2949_BM_STATIPM_I1LM 0x2
// P1H mask
#define LTC2949_BM_STATIPM_P1HM 0x4
// P1L mask
#define LTC2949_BM_STATIPM_P1LM 0x8
// I2H mask
#define LTC2949_BM_STATIPM_I2HM 0x10
// I2L mask
#define LTC2949_BM_STATIPM_I2LM 0x20
// P2H mask
#define LTC2949_BM_STATIPM_P2HM 0x40
// P2L mask
#define LTC2949_BM_STATIPM_P2LM 0x80

// STATCM bit masks
// C1H mask
#define LTC2949_BM_STATCM_C1HM 0x1
// C1L mask
#define LTC2949_BM_STATCM_C1LM 0x2
// C2H mask
#define LTC2949_BM_STATCM_C2HM 0x4
// C2L mask
#define LTC2949_BM_STATCM_C2LM 0x8
// C3H mask
#define LTC2949_BM_STATCM_C3HM 0x10
// C3L mask
#define LTC2949_BM_STATCM_C3LM 0x20

// STATEM bit masks
// E1H mask
#define LTC2949_BM_STATEM_E1HM 0x1
// E1L mask
#define LTC2949_BM_STATEM_E1LM 0x2
// E2H mask
#define LTC2949_BM_STATEM_E2HM 0x4
// E2L mask
#define LTC2949_BM_STATEM_E2LM 0x8
// E4H mask
#define LTC2949_BM_STATEM_E4HM 0x40
// E4L mask
#define LTC2949_BM_STATEM_E4LM 0x80

// STATCEOFM bit masks
// C1OF mask
#define LTC2949_BM_STATCEOFM_C1OFM 0x1
// C2OF mask
#define LTC2949_BM_STATCEOFM_C2OFM 0x2
// C3OF mask
#define LTC2949_BM_STATCEOFM_C3OFM 0x4
// E1OF mask
#define LTC2949_BM_STATCEOFM_E1OFM 0x10
// E2OF mask
#define LTC2949_BM_STATCEOFM_E2OFM 0x20
// E4OF mask
#define LTC2949_BM_STATCEOFM_E4OFM 0x80

// STATTBM bit masks
// TB1TH mask
#define LTC2949_BM_STATTBM_TB1THM 0x1
// TB2TH mask
#define LTC2949_BM_STATTBM_TB2THM 0x2
// TB3TH mask
#define LTC2949_BM_STATTBM_TB3THM 0x4
// TB4TH mask
#define LTC2949_BM_STATTBM_TB4THM 0x8
// TB1OF mask
#define LTC2949_BM_STATTBM_TB1OFM 0x10
// TB2OF mask
#define LTC2949_BM_STATTBM_TB2OFM 0x20
// TB3OF mask
#define LTC2949_BM_STATTBM_TB3OFM 0x40
// TB4OF mask
#define LTC2949_BM_STATTBM_TB4OFM 0x80

// STATVCCM bit masks
// VCCH mask
#define LTC2949_BM_STATVCCM_VCCHM 0x1
// VCCL mask
#define LTC2949_BM_STATVCCM_VCCLM 0x2

// EXTFAULTS bit masks
// Hamming decoder 1-bit error
#define LTC2949_BM_EXTFAULTS_HD1BITERR 0x1
// ROM CRC error
#define LTC2949_BM_EXTFAULTS_ROMERR 0x2
// MEM error
#define LTC2949_BM_EXTFAULTS_MEMERR 0x4
// FCA error
#define LTC2949_BM_EXTFAULTS_FCAERR 0x8
// XRAM error
#define LTC2949_BM_EXTFAULTS_XRAMERR 0x10
// IRAM error
#define LTC2949_BM_EXTFAULTS_IRAMERR 0x20
// HW memory BIST executed
#define LTC2949_BM_EXTFAULTS_HWMBISTEXEC 0x80

// FAULTS bit masks
// Fuse Table Error, more than 1 bit has flipped in the fuse table, no FEC was possible
#define LTC2949_BM_FAULTS_PROMERR 0x1
// Thermal shutdown, the previous reset was due to an over temperature shutdown
#define LTC2949_BM_FAULTS_TSD 0x2
// Internal communication parity check failed
#define LTC2949_BM_FAULTS_INTCOMMERR 0x4
// external communication (SPI / isoSPI) PEC error
#define LTC2949_BM_FAULTS_EXTCOMMERR 0x8
// fast channel error
#define LTC2949_BM_FAULTS_FAERR 0x10
// Hardware BIST result: 1 error, 0 ok
#define LTC2949_BM_FAULTS_HWBIST 0x20
// Hardware config memory BIST result: 1 error, 0 ok
#define LTC2949_BM_FAULTS_CRCCFG 0x40
// Hardware memory BIST result: 1 error, 0 ok
#define LTC2949_BM_FAULTS_CRCMEM 0x80

// OCC1CTRL bit masks
// OCC1 enable bit. If enabled GPO5 is configured as OCC Heart beat
#define LTC2949_BM_OCC1CTRL_OCC1EN 0x1
// OCC1 threshold setting bits
// [000]: 0mV 
// [001]: 26mV 
// [010]: 52mV 
// [011]: 78mV 
// [100]: 103mV 
// [101]: 155mV 
// [110]: 207mV 
// [111]: 310mV
#define LTC2949_BM_OCC1CTRL_OCC1DAC0 0x2
// OCC1 threshold setting bits
// [000]: 0mV 
// [001]: 26mV 
// [010]: 52mV 
// [011]: 78mV 
// [100]: 103mV 
// [101]: 155mV 
// [110]: 207mV 
// [111]: 310mV
#define LTC2949_BM_OCC1CTRL_OCC1DAC1 0x4
// OCC1 threshold setting bits
// [000]: 0mV 
// [001]: 26mV 
// [010]: 52mV 
// [011]: 78mV 
// [100]: 103mV 
// [101]: 155mV 
// [110]: 207mV 
// [111]: 310mV
#define LTC2949_BM_OCC1CTRL_OCC1DAC2 0x8
// OCC1 deglitch time setting bits
// [00]: 20us 
// [01]: 80us 
// [10]: 320us 
// [11]: 1.28ms
#define LTC2949_BM_OCC1CTRL_OCC1DGLT0 0x10
// OCC1 deglitch time setting bits
// [00]: 20us 
// [01]: 80us 
// [10]: 320us 
// [11]: 1.28ms
#define LTC2949_BM_OCC1CTRL_OCC1DGLT1 0x20
// OCC1 polarity setting bits
// [00]: Both Polarities 
// [01]: Positive Currents 
// [10]: Negative Currents 
// [11]: RESERVED
#define LTC2949_BM_OCC1CTRL_OCC1POL0 0x40
// OCC1 polarity setting bits
// [00]: Both Polarities 
// [01]: Positive Currents 
// [10]: Negative Currents 
// [11]: RESERVED
#define LTC2949_BM_OCC1CTRL_OCC1POL1 0x80

// OCC2CTRL bit masks
// OCC2 enable bit. If enabled GPO5 is configured as OCC Heart beat
#define LTC2949_BM_OCC2CTRL_OCC2EN 0x1
// OCC2 threshold setting bits
// [000]: 0mV 
// [001]: 26mV 
// [010]: 52mV 
// [011]: 78mV 
// [100]: 103mV 
// [101]: 155mV 
// [110]: 207mV 
// [111]: 310mV
#define LTC2949_BM_OCC2CTRL_OCC2DAC0 0x2
// OCC2 threshold setting bits
// [000]: 0mV 
// [001]: 26mV 
// [010]: 52mV 
// [011]: 78mV 
// [100]: 103mV 
// [101]: 155mV 
// [110]: 207mV 
// [111]: 310mV
#define LTC2949_BM_OCC2CTRL_OCC2DAC1 0x4
// OCC2 threshold setting bits
// [000]: 0mV 
// [001]: 26mV 
// [010]: 52mV 
// [011]: 78mV 
// [100]: 103mV 
// [101]: 155mV 
// [110]: 207mV 
// [111]: 310mV
#define LTC2949_BM_OCC2CTRL_OCC2DAC2 0x8
// OCC2 deglitch time setting bits
// [00]: 20us 
// [01]: 80us 
// [10]: 320us 
// [11]: 1.28ms
#define LTC2949_BM_OCC2CTRL_OCC2DGLT0 0x10
// OCC2 deglitch time setting bits
// [00]: 20us 
// [01]: 80us 
// [10]: 320us 
// [11]: 1.28ms
#define LTC2949_BM_OCC2CTRL_OCC2DGLT1 0x20
// OCC2 polarity setting bits
// [00]: Both Polarities 
// [01]: Positive Currents 
// [10]: Negative Currents 
// [11]: RESERVED
#define LTC2949_BM_OCC2CTRL_OCC2POL0 0x40
// OCC2 polarity setting bits
// [00]: Both Polarities 
// [01]: Positive Currents 
// [10]: Negative Currents 
// [11]: RESERVED
#define LTC2949_BM_OCC2CTRL_OCC2POL1 0x80

// ACCCTRL1 bit masks
// Charge1/Energy1 Current1 control (bit 0)
#define LTC2949_BM_ACCCTRL1_ACC1I10 0x1
// Charge1/Energy1 Current1 control (bit 1)
#define LTC2949_BM_ACCCTRL1_ACC1I11 0x2
// Charge2/Energy2 Current2 control (bit 0)
#define LTC2949_BM_ACCCTRL1_ACC2I20 0x4
// Charge2/Energy2 Current2 control (bit 1)
#define LTC2949_BM_ACCCTRL1_ACC2I21 0x8

// ACCCTRL2 bit masks
// I1 into Charge3 control (bit 0)
#define LTC2949_BM_ACCCTRL2_ACC3I10 0x1
// I1 into Charge3 control (bit 1)
#define LTC2949_BM_ACCCTRL2_ACC3I11 0x2
// I2 into Charge3 control (bit 0)
#define LTC2949_BM_ACCCTRL2_ACC3I20 0x4
// I2 into Charge3 control (bit 1)
#define LTC2949_BM_ACCCTRL2_ACC3I21 0x8
// P1 into Energy4 control (bit 0)
#define LTC2949_BM_ACCCTRL2_ACC4I10 0x10
// P1 into Energy4 control (bit 1)
#define LTC2949_BM_ACCCTRL2_ACC4I11 0x20
// P2 into Energy4 control (bit 0)
#define LTC2949_BM_ACCCTRL2_ACC4I20 0x40
// P2 into Energy4 control (bit 1)
#define LTC2949_BM_ACCCTRL2_ACC4I21 0x80

// GPIO4HBCTRL bit masks
// 0: GPO4 is not configured as heart beat
// 1: GPO4 is configured as heart beat and unmasked alerts (see mask registers) stop heart beat of GPO4
#define LTC2949_BM_GPIO4HBCTRL_GPO4HBEN 0x1

// TBCTRL bit masks
// Prescaler value bit 0, binary coded.
#define LTC2949_BM_TBCTRL_PRE_0 0x1
// Prescaler value bit 1, binary coded.
#define LTC2949_BM_TBCTRL_PRE_1 0x2
// Prescaler value bit 2, binary coded.
#define LTC2949_BM_TBCTRL_PRE_2 0x4
// Divider value bit 0, binary coded.
#define LTC2949_BM_TBCTRL_DIV_0 0x8
// Divider value bit 1, binary coded.
#define LTC2949_BM_TBCTRL_DIV_1 0x10
// Divider value bit 2, binary coded.
#define LTC2949_BM_TBCTRL_DIV_2 0x20
// Divider value bit 3, binary coded.
#define LTC2949_BM_TBCTRL_DIV_3 0x40
// Divider value bit 4, binary coded.
#define LTC2949_BM_TBCTRL_DIV_4 0x80

// SLOT1MUXN bit masks
// SLOT1 mux. setting neg. bit 0
#define LTC2949_BM_SLOT1MUXN_SLOT1MUXN0 0x1
// SLOT1 mux. setting neg. bit 1
#define LTC2949_BM_SLOT1MUXN_SLOT1MUXN1 0x2
// SLOT1 mux. setting neg. bit 2
#define LTC2949_BM_SLOT1MUXN_SLOT1MUXN2 0x4
// SLOT1 mux. setting neg. bit 3
#define LTC2949_BM_SLOT1MUXN_SLOT1MUXN3 0x8
// SLOT1 mux. setting neg. bit 4
#define LTC2949_BM_SLOT1MUXN_SLOT1MUXN4 0x10

// SLOT1MUXP bit masks
// SLOT1 mux. setting pos. bit 0
#define LTC2949_BM_SLOT1MUXP_SLOT1MUXP0 0x1
// SLOT1 mux. setting pos. bit 1
#define LTC2949_BM_SLOT1MUXP_SLOT1MUXP1 0x2
// SLOT1 mux. setting pos. bit 2
#define LTC2949_BM_SLOT1MUXP_SLOT1MUXP2 0x4
// SLOT1 mux. setting pos. bit 3
#define LTC2949_BM_SLOT1MUXP_SLOT1MUXP3 0x8
// SLOT1 mux. setting pos. bit 4
#define LTC2949_BM_SLOT1MUXP_SLOT1MUXP4 0x10

// SLOT2MUXN bit masks
// SLOT2 mux. setting neg. bit 0
#define LTC2949_BM_SLOT2MUXN_SLOT2MUXN0 0x1
// SLOT2 mux. setting neg. bit 1
#define LTC2949_BM_SLOT2MUXN_SLOT2MUXN1 0x2
// SLOT2 mux. setting neg. bit 2
#define LTC2949_BM_SLOT2MUXN_SLOT2MUXN2 0x4
// SLOT2 mux. setting neg. bit 3
#define LTC2949_BM_SLOT2MUXN_SLOT2MUXN3 0x8
// SLOT2 mux. setting neg. bit 4
#define LTC2949_BM_SLOT2MUXN_SLOT2MUXN4 0x10

// SLOT2MUXP bit masks
// SLOT2 mux. setting pos. bit 0
#define LTC2949_BM_SLOT2MUXP_SLOT2MUXP0 0x1
// SLOT2 mux. setting pos. bit 1
#define LTC2949_BM_SLOT2MUXP_SLOT2MUXP1 0x2
// SLOT2 mux. setting pos. bit 2
#define LTC2949_BM_SLOT2MUXP_SLOT2MUXP2 0x4
// SLOT2 mux. setting pos. bit 3
#define LTC2949_BM_SLOT2MUXP_SLOT2MUXP3 0x8
// SLOT2 mux. setting pos. bit 4
#define LTC2949_BM_SLOT2MUXP_SLOT2MUXP4 0x10

// OPCTRL bit masks
// 0: Normal operation, 1: Sleep
// LTC2949 will wake up in SPI mode if CS pulled low and
// in isoSPI mode if a long differential pulse is detected
#define LTC2949_BM_OPCTRL_SLEEP 0x1
// Clear accumulators and tracking registers
#define LTC2949_BM_OPCTRL_CLR 0x2
// Single shot measurement Ix, Px, SLOTx, BAT, TEMP, VCC
#define LTC2949_BM_OPCTRL_SSHOT 0x4
// Continuous measurement. Only in this mode Charge, Energy
// and Time are measured and Fast channel may be activate
#define LTC2949_BM_OPCTRL_CONT 0x8
// Update calibration data.
// Bit is cleared automatically after update was performed.
#define LTC2949_BM_OPCTRL_ADJUPD 0x20
// Write 1 to reset device 
// Only possible after reset was unlocked
// by writing 0x55 to RSTUNLCK
#define LTC2949_BM_OPCTRL_RST 0x80

// FCURGPIOCTRL bit masks
// GPO5 control bit 0
#define LTC2949_BM_FCURGPIOCTRL_GPO5CTRL0 0x1
// GPO5 control bit 1
#define LTC2949_BM_FCURGPIOCTRL_GPO5CTRL1 0x2
// MUXP Current Source Polarity
// 0: MUXP sinks 250uA of current 
// 1: MUXP sources 250uA of current
#define LTC2949_BM_FCURGPIOCTRL_MUXPCURPOL 0x10
// MUXP Current Source Enable
#define LTC2949_BM_FCURGPIOCTRL_MUXPCUREN 0x20
// MUXN Current Source Polarity
// 0: MUXN sinks 250uA of current 
// 1: MUXN sources 250uA of current
#define LTC2949_BM_FCURGPIOCTRL_MUXNCURPOL 0x40
// MUXN Current Source Enable
#define LTC2949_BM_FCURGPIOCTRL_MUXNCUREN 0x80

// FGPIOCTRL bit masks
// GPO1 control bit 0
#define LTC2949_BM_FGPIOCTRL_GPO1CTRL0 0x1
// GPO1 control bit 1
#define LTC2949_BM_FGPIOCTRL_GPO1CTRL1 0x2
// GPO2 control bit 0
#define LTC2949_BM_FGPIOCTRL_GPO2CTRL0 0x4
// GPO2 control bit 1
#define LTC2949_BM_FGPIOCTRL_GPO2CTRL1 0x8
// GPO3 control bit 0
#define LTC2949_BM_FGPIOCTRL_GPO3CTRL0 0x10
// GPO3 control bit 1
#define LTC2949_BM_FGPIOCTRL_GPO3CTRL1 0x20
// GPO4 control bit 0
#define LTC2949_BM_FGPIOCTRL_GPO4CTRL0 0x40
// GPO4 control bit 1
#define LTC2949_BM_FGPIOCTRL_GPO4CTRL1 0x80

// FAMUXN bit masks
// Fast MUXN bit 0, binary coded 
#define LTC2949_BM_FAMUXN_FAMUXN0 0x1
// Fast MUXN bit 1, binary coded 
#define LTC2949_BM_FAMUXN_FAMUXN1 0x2
// Fast MUXN bit 2, binary coded 
#define LTC2949_BM_FAMUXN_FAMUXN2 0x4
// Fast MUXN bit 3, binary coded 
#define LTC2949_BM_FAMUXN_FAMUXN3 0x8
// Fast MUXN bit 4, binary coded 
#define LTC2949_BM_FAMUXN_FAMUXN4 0x10

// FAMUXP bit masks
// Fast MUXP bit 0, binary coded 
#define LTC2949_BM_FAMUXP_FAMUXP0 0x1
// Fast MUXP bit 1, binary coded 
#define LTC2949_BM_FAMUXP_FAMUXP1 0x2
// Fast MUXP bit 2, binary coded 
#define LTC2949_BM_FAMUXP_FAMUXP2 0x4
// Fast MUXP bit 3, binary coded 
#define LTC2949_BM_FAMUXP_FAMUXP3 0x8
// Fast MUXP bit 4, binary coded 
#define LTC2949_BM_FAMUXP_FAMUXP4 0x10

// FACTRL bit masks
// Global enable of continuous fast measurement
#define LTC2949_BM_FACTRL_FACONV 0x1
// Fast auxilliary channel meaurement enable
#define LTC2949_BM_FACTRL_FACHA 0x2
// Fast channel 1 meaurement enable (I1 and P1 as voltage if PASV1=1)
#define LTC2949_BM_FACTRL_FACH1 0x4
// Fast channel 2 meaurement enable (I2 and P2 as voltage if (PASV1=0, PASV2=1)
#define LTC2949_BM_FACTRL_FACH2 0x8

// REGSCTRL bit masks
// Memory Map Page Select.
// 0: PAGE 0 of memory map is selected. 
// 1: PAGE1 of memory map is selected.
#define LTC2949_BM_REGSCTRL_PAGE 0x1
// Broadcast read enable
// 0: broadcast read commands are ignored
// 1: device will respond to broadcast read commands
#define LTC2949_BM_REGSCTRL_BCREN 0x4
// Memory lock control bit 0
#define LTC2949_BM_REGSCTRL_MLK0 0x10
// Memory lock control bit 1
#define LTC2949_BM_REGSCTRL_MLK1 0x20
// 0: indirect memory access mode, RDCV will read data starting at address RDCVIADDR
// 1: RDCV command will read fast channel conversion results
#define LTC2949_BM_REGSCTRL_RDCVCONF 0x80

// EEPROM bit masks
// Write signature to EEPROM
#define LTC2949_BM_EEPROM_INIT 0x1
// Check the signature in EEPROM
#define LTC2949_BM_EEPROM_CHECK 0x2
// Save memory (but special row, STATUS and ALERT registers) to EEPROM 
#define LTC2949_BM_EEPROM_SAVE 0x4
// Restore memory (but special row, STATUS and ALERT registers) from EEPROM 
#define LTC2949_BM_EEPROM_RESTORE 0x8
// Result of INIT (1: operation successful done)
#define LTC2949_BM_EEPROM_INITRSL 0x10
// Result of CHECK (1: operation successful done)
#define LTC2949_BM_EEPROM_CHECKRSL 0x20
// Result of SAVE (1: operation successful done)
#define LTC2949_BM_EEPROM_SAVERSL 0x40
// Result of RESTORE (1: operation successful done)
#define LTC2949_BM_EEPROM_RESTORERSL 0x80

// ISO0 bit masks
// hamming decoder
#define LTC2949_BM_ISO0_FERR 0x1
// inject parity error into ISER master to slave stream
#define LTC2949_BM_ISO0_ISERERRINJ 0x4
// inject error into ESER slave to master stream
#define LTC2949_BM_ISO0_ESERERRINJ 0x8
// inject error in the comparator of the AUX redundant channel
#define LTC2949_BM_ISO0_FCAERRINJ 0x10
// HW BIST; run the test with some error randomly injected
#define LTC2949_BM_ISO0_HWBISTINJ 0x20
// Inject error at check of CRC of XRAM versions of CFG Regs.
#define LTC2949_BM_ISO0_CRCCFGINJ 0x40
// Inject error at check of CRC of MEM
#define LTC2949_BM_ISO0_MEMERRINJ 0x80

// ISO1 bit masks
// push buttons, start the test when set, poll for 0 == completion
//#define LTC2949_BM_ISO1_AFSR1 0x1
// if set also MUXP-MUXN is checked, works only with AFSR1
//#define LTC2949_BM_ISO1_AFSR1S 0x2
// push buttons, start the test when set, poll for 0 == completion
#define LTC2949_BM_ISO1_AFSR2 0x4
// push buttons, start the test when set, poll for 0 == completion
//#define LTC2949_BM_ISO1_VSANITY 0x8
// results; 1 ok, 0 error (when the push button is 0)
//#define LTC2949_BM_ISO1_AFSR1RSL 0x10
// results; 1 ok, 0 error (when the push button is 0)
// note AFSR1SRSL doesn't exist. Result is always AFSR1RSL and AFSR1S won't be cleared!
// #define LTC2949_BM_ISO1_AFSR1SRSL 0x20
// results; 1 ok, 0 error (when the push button is 0)
#define LTC2949_BM_ISO1_AFSR2RSL 0x40
// results; 1 ok, 0 error (when the push button is 0)
//#define LTC2949_BM_ISO1_VSANITYRSL 0x80

// ISO2 bit masks
// push buttons, start the test when set, poll for 0 == completion
//#define LTC2949_BM_ISO2_DATAPUMP 0x1
// when set the OCC BIST is done on OCC1_CTRL setting on both channels
#define LTC2949_BM_ISO2_SHORTOCC 0x2
// results; 1 ok, 0 error (when the push button is 0)
//#define LTC2949_BM_ISO2_RSLDATAPUMP 0x10
// Enable digital filter test 1 for AUX ADC (expected result is 0x4009)
//#define LTC2949_BM_ISO2_AFSR1FT1 0x20
// Enable digital filter test 2 for AUX ADC (expected result is 0x955E)
//#define LTC2949_BM_ISO2_AFSR1FT2 0x40
// Enable digital filter test 3 for AUX ADC (expected result is 0xEAB3)
//#define LTC2949_BM_ISO2_AFSR1FT3 0x80

// // ISORPT bit masks
// // AFSR1:  [0] -> offset error (INL), leakage error; VSANITY: VDIG
// #define LTC2949_BM_ISORPT_ISORPT0 0x1
// // AFSR1:  [1] -> 1.2 error (INL) VSANITY: VSTBY
// #define LTC2949_BM_ISORPT_ISORPT1 0x2
// // AFSR1:  [2] -> Vref error (INL)
// #define LTC2949_BM_ISORPT_ISORPT2 0x4
// // AFSR1:  [3] -> RED REF error, noise
// #define LTC2949_BM_ISORPT_ISORPT3 0x8
// // AFSR1:  [4] -> MUXP-MUXN error, noise
// #define LTC2949_BM_ISORPT_ISORPT4 0x10
// // AFSR1:  [5] -> OS clock (PLL test)
// #define LTC2949_BM_ISORPT_ISORPT5 0x20
// // AFSR1:  [7] -> data path bit toggle
// #define LTC2949_BM_ISORPT_ISORPT6 0x40
// // AFSR1:  [8] -> execution error
// #define LTC2949_BM_ISORPT_ISORPT7 0x80

// ADCCONF bit masks
// 1: P1 ADC configured to voltage mode
// 0: P1 ADC configured to power mode
#define LTC2949_BM_ADCCONF_P1ASV 0x1
// 1: P2 ADC configured to voltage mode
// 0: P2 ADC configured to power mode
#define LTC2949_BM_ADCCONF_P2ASV 0x2
// 1: SLOT1 is considered as NTC voltage and therefore the reported quatity, the thresholds and the tracking registers for SLOT1 have to be considered T (LSB = 0.2°C)
#define LTC2949_BM_ADCCONF_NTC1 0x8
// 1: SLOT2 is considered as NTC voltage and therefore the reported quatity, the thresholds and the tracking registers for SLOT2 have to be considered T (LSB = 0.2°C)
#define LTC2949_BM_ADCCONF_NTC2 0x10
// Reserved, set to 0
#define LTC2949_BM_ADCCONF_RES 0x20
// if 1 also the CH2 will be linked to SLOT1 (given that SLOT1 and SLOT2 are set to NTC); this way the SLOT2 is free to be used as GP ADC IN (one shunt scenario)
#define LTC2949_BM_ADCCONF_NTCSLOT1 0x40



// <------- AUTO-GENERATED-HEADER ENDS HERE!!!! ------>

// [000]: 0mV 
#define LTC2949_BM_OCC_DACTHR_000 (                           0|                           0|                           0)
// [001]: 26mV 
#define LTC2949_BM_OCC_DACTHR_025 (                           0|                           0|LTC2949_BM_OCC2CTRL_OCC2DAC0)
// [010]: 52mV 
#define LTC2949_BM_OCC_DACTHR_050 (                           0|LTC2949_BM_OCC2CTRL_OCC2DAC1|                           0)
// [011]: 78mV 
#define LTC2949_BM_OCC_DACTHR_075 (                           0|LTC2949_BM_OCC2CTRL_OCC2DAC1|LTC2949_BM_OCC2CTRL_OCC2DAC0)
// [100]: 103mV 
#define LTC2949_BM_OCC_DACTHR_100 (LTC2949_BM_OCC2CTRL_OCC2DAC2|                           0|                           0)
// [101]: 155mV 
#define LTC2949_BM_OCC_DACTHR_150 (LTC2949_BM_OCC2CTRL_OCC2DAC2|                           0|LTC2949_BM_OCC2CTRL_OCC2DAC0)
// [110]: 207mV 
#define LTC2949_BM_OCC_DACTHR_200 (LTC2949_BM_OCC2CTRL_OCC2DAC2|LTC2949_BM_OCC2CTRL_OCC2DAC1|                           0)
// [111]: 310mV
#define LTC2949_BM_OCC_DACTHR_300 (LTC2949_BM_OCC2CTRL_OCC2DAC2|LTC2949_BM_OCC2CTRL_OCC2DAC1|LTC2949_BM_OCC2CTRL_OCC2DAC0)
// Disabled
#define LTC2949_BM_OCC_DACTHR_OFF -1

// [00]: 20us 
#define LTC2949_BM_OCC_DGLT_0020 (                            0|                            0)
// [01]: 80us 
#define LTC2949_BM_OCC_DGLT_0080 (                            0|LTC2949_BM_OCC2CTRL_OCC2DGLT0)
// [10]: 320us 
#define LTC2949_BM_OCC_DGLT_0320 (LTC2949_BM_OCC2CTRL_OCC2DGLT1|                            0)
// [11]: 1.28ms
#define LTC2949_BM_OCC_DGLT_1280 (LTC2949_BM_OCC2CTRL_OCC2DGLT1|LTC2949_BM_OCC2CTRL_OCC2DGLT0)

// [00]: Both Polarities 
#define LTC2949_BM_OCC_POL_BOTH (                           0|                           0)
// [01]: Positive Currents 
#define LTC2949_BM_OCC_POL_POS  (                           0|LTC2949_BM_OCC2CTRL_OCC2POL0)
// [10]: Negative Currents 
#define LTC2949_BM_OCC_POL_NEG  (LTC2949_BM_OCC2CTRL_OCC2POL1|                           0)



// GPIO control
// 00: GPIO is tristate
// 11: GPIO is high (DVCC)
// 01: GPIO is low (DGND)
// 10: GPIO toggles at 400kHz between DVCC, DGND (register on page 0)
#define LTC2949_BM_GPIOCFG_INPUT        0
#define LTC2949_BM_GPIOCFG_OUPUT_LOW    1
#define LTC2949_BM_GPIOCFG_OUPUT_TOGGLE 2
#define LTC2949_BM_GPIOCFG_OUPUT_HIGH   3

// GPO5 toggling
#define LTC2949_GPO5_HI  (LTC2949_BM_FCURGPIOCTRL_GPO5CTRL1|LTC2949_BM_FCURGPIOCTRL_GPO5CTRL0)
// GPO5 toggling
#define LTC2949_GPO5_LO  (0                                 |LTC2949_BM_FCURGPIOCTRL_GPO5CTRL0)
// GPO5 toggling 400kHz
#define LTC2949_GPO5_TGL (LTC2949_BM_FCURGPIOCTRL_GPO5CTRL1|0                                 )
// GPO5 input (high impedance)
#define LTC2949_GPO5_IN  (0                                 |0                                 )

// GPO1 toggling
#define LTC2949_GPO1_HI  (LTC2949_BM_FGPIOCTRL_GPO1CTRL1|LTC2949_BM_FGPIOCTRL_GPO1CTRL0)
// GPO1 toggling
#define LTC2949_GPO1_LO  (0                              |LTC2949_BM_FGPIOCTRL_GPO1CTRL0)
// GPO1 toggling 400kHz
#define LTC2949_GPO1_TGL (LTC2949_BM_FGPIOCTRL_GPO1CTRL1|0                              )
// GPO1 input (high impedance)
#define LTC2949_GPO1_IN  (0                              |0                              )

// GPO2 toggling
#define LTC2949_GPO2_HI  (LTC2949_BM_FGPIOCTRL_GPO2CTRL1|LTC2949_BM_FGPIOCTRL_GPO2CTRL0)
// GPO2 toggling
#define LTC2949_GPO2_LO  (0                              |LTC2949_BM_FGPIOCTRL_GPO2CTRL0)
// GPO2 toggling 400kHz
#define LTC2949_GPO2_TGL (LTC2949_BM_FGPIOCTRL_GPO2CTRL1|0                              )
// GPO2 input (high impedance)
#define LTC2949_GPO2_IN  (0                              |0                              )

// GPO3 toggling
#define LTC2949_GPO3_HI  (LTC2949_BM_FGPIOCTRL_GPO3CTRL1|LTC2949_BM_FGPIOCTRL_GPO3CTRL0)
// GPO3 toggling
#define LTC2949_GPO3_LO  (0                              |LTC2949_BM_FGPIOCTRL_GPO3CTRL0)
// GPO3 toggling 400kHz
#define LTC2949_GPO3_TGL (LTC2949_BM_FGPIOCTRL_GPO3CTRL1|0                              )
// GPO3 input (high impedance)
#define LTC2949_GPO3_IN  (0                              |0                              )

// GPO4 toggling
#define LTC2949_GPO4_HI  (LTC2949_BM_FGPIOCTRL_GPO4CTRL1|LTC2949_BM_FGPIOCTRL_GPO4CTRL0)
// GPO4 toggling
#define LTC2949_GPO4_LO  (0                              |LTC2949_BM_FGPIOCTRL_GPO4CTRL0)
// GPO4 toggling 400kHz
#define LTC2949_GPO4_TGL (LTC2949_BM_FGPIOCTRL_GPO4CTRL1|0                              )
// GPO4 input (high impedance)
#define LTC2949_GPO4_IN  (0                              |0                              )

// enable positive/sourcing/pull-up current source on MUXP
#define LTC2949_CURR_PUMUXP (LTC2949_BM_FCURGPIOCTRL_MUXPCURPOL|LTC2949_BM_FCURGPIOCTRL_MUXPCUREN)
// enable negative/sinking/pull-down current source on MUXP
#define LTC2949_CURR_PDMUXP (0                                 |LTC2949_BM_FCURGPIOCTRL_MUXPCUREN)
// disable current source on MUXP
#define LTC2949_CURR_DISMUXP 0
// enable positive/sourcing/pull-up current source on MUXN
#define LTC2949_CURR_PUMUXN (LTC2949_BM_FCURGPIOCTRL_MUXNCURPOL|LTC2949_BM_FCURGPIOCTRL_MUXNCUREN)
// enable negative/sinking/pull-down current source on MUXN
#define LTC2949_CURR_PDMUXN (0                                 |LTC2949_BM_FCURGPIOCTRL_MUXNCUREN)
// disable current source on MUXN
#define LTC2949_CURR_DISMUXN 0


// EXTERNALS
extern byte LTC2949_iAddrRegsCtrl[];
extern boolean LTC2949_onTopOfDaisychain;
extern boolean LTC2949_DebugEnable;
extern uint8_t LTC2949_CellMonitorCount;
extern byte LTC2949_adcConf; // PasV, NTC register

// holds the next timestamp when isoSPI timeout duration has elapsed
extern unsigned long LTC2949_isoSPI_wakeup_timeoutUs;


//extern boolean LTC2949_ntc1;
//extern boolean LTC2949_ntc2;
//extern boolean LTC2949_p1asv;
//extern boolean LTC2949_p2asv;
extern byte LTC2949_gpioCtrl[];
extern boolean LTC2949_forceWrRegsCtrl;
extern uint32_t LTC2949_lastTBx;
extern double LTC2949_TBFAC;
#ifdef ARDUINO
extern SPISettings LTC2949_SPISettings;
#else
extern bool spiBufferBusy;
extern unsigned int spiBufferLen;
#define SPIBUFFERMAXLEN 1024
extern uint8_t spiBuffer[SPIBUFFERMAXLEN];
#endif
extern uint8_t LTC2949_CS;


#ifdef LTC2949_STORETIME_LAST_PEC
extern unsigned long LTC2949_TimeLastPEC;
#endif

extern boolean LTC2949_autoForceWrRegsCtrl;

// <------- FUNCTION PROTOTYPES START HERE!!!! ------>


// 
#define LTC2949_REGSCTRL_MEMLOCK_REQUEST  LTC2949_BM_REGSCTRL_MLK0
// 
#define LTC2949_REGSCTRL_MEMLOCK_ACK      LTC2949_BM_REGSCTRL_MLK1
// 
#define LTC2949_REGSCTRL_MEMLOCK_UNLOCKED 0
// 
#define LTC2949_REGSCTRL_MEMLOCK_ERR      (LTC2949_BM_REGSCTRL_MLK1|LTC2949_BM_REGSCTRL_MLK0)


#define LTC2949_DCWR_ERRORTYPE_CMDADDR  0
#define LTC2949_DCWR_ERRORTYPE_CMDPEC   1
#define LTC2949_DCWR_ERRORTYPE_DATA     2
#define LTC2949_DCWR_ERRORTYPE_DATAPEC  3
#define LTC2949_DCWR_ERRORTYPE_ID2      4
#define LTC2949_DCWR_ERRORTYPE_ID5      5
// number of error types available
#define LTC2949_DCWR_ERRORTYPES_CNT     6
// used to simulate PEC errors (single byte with PEC error)
void LTC2949_DcWR_PECerror(byte addr, byte data, byte errorType);


//! Calculate and write PRE and DIV for given frequency and write PRE/DIV configuration to TBCTRL register
void LTC2949_WriteTbctl(
	double extClk = NAN, //!< External clock frequency
	//!< Allowed values are: 
	//!<  4.0e6 for external crystal
	//!<  30.0e3...25.0e6 for external oscillator
	//!<  NAN for internal oscillator
	//!<  for other frequencies the internal oscillator will be enabled
	boolean verbose = false
);

boolean LTC2949_AnyPasV();


void LTC2949_GpioConfig(byte gpio, byte mode);
void LTC2949_CurrSrcConfig(byte currCtrl);
byte LTC2949_GpioConfigGet(byte gpio);
boolean LTC2949_Abs(byte* bytes, byte length);
double LTC2949_BytesToDouble(byte* bytes, byte length, boolean sig, double lsb);
double LTC2949_UnsignedBytesToDouble(byte* unsignedBytes, byte length, double lsb);
double LTC2949_SignedBytesToDouble(byte* signedBytes, byte length, double lsb);

void LTC2949_init_lib(
	byte cellMonitorCount,
	boolean ltc2949onTopOfDaisychain,
	boolean debugEnable);
void LTC2949_sel_rd_mode(
	byte cellMonitorCount,
	boolean ltc2949onTopOfDaisychain);

void LTC2949_init_device_state();

void LTC2949_ADCConfigWrite();
byte LTC2949_EEPROMCommand(byte command);
byte LTC2949_EEPROMIsReady();
byte LTC2949_EEPROMRead();
byte LTC2949_EEPROMWrite();
byte LTC2949_EEPROMInitCheck();
byte LTC2949_MemLock(boolean lock);
void LTC2949_MemLockRequest();
void LTC2949_MemLockRelease();
byte LTC2949_MemLockStatus(byte* mlk);
byte LTC2949_MemLockStatusInt();
byte LTC2949_OpctlAdjUpd(boolean clear = false);
void LTC2949_OpctlSleep();
byte LTC2949_PollReg(uint16_t addr, byte val, byte mask = 0xFFU, uint16_t timeout = LTC2949_POLL_TIMEOUT, boolean stopOnPecErrors = true, byte * finalRead = NULL);
#ifdef LTCDEF_SPI_POLL_SUPPORT
byte LTC2949_RdFastData(int16_t* data = NULL, uint16_t* cellMonDat = NULL, uint16_t rdcv = LTC2949_68XX_CMD_RDCVA, uint8_t pollTimeout16Us = 0);
#else
byte LTC2949_RdFastData(int16_t* data = NULL, uint16_t* cellMonDat = NULL, uint16_t rdcv = LTC2949_68XX_CMD_RDCVA);
#endif
byte LTC2949_PollFastData(int16_t* data = NULL, boolean clrChkHS = false, uint16_t* cellMonDat = NULL, uint16_t rdcv = LTC2949_68XX_CMD_RDCVA);
byte LTC2949_WakeupAndAck();
byte _LTC2949_DcRW_(byte addr, uint16_t len, byte* data, boolean readNotWrite);
void LTC2949_ADCConfigNTC(byte ntcCfg);
void LTC2949_ADCConfigTC(byte tcCfg);
void LTC2949_ADCConfigPasV(byte pasvCfg);
void LTC2949_ADCConfigAll(byte adcCfg);
//byte LTC2949_ADCConfigGetBuffer();
void LTC2949_F24BytesToFloat(byte* bytes, float* f32);
void LTC2949_FloatToF24Bytes(float f32, byte* bytes);
void LTC2949_wakeup_idle(boolean force = false);
boolean LTC2949_ChkUpdate(byte* error, unsigned long* ms = NULL, byte tbAddr = LTC2949_VAL_TB1);
double LTC2949_GetLastTBxDbl();
uint32_t LTC2949_GetLastTBxInt();
byte LTC2949_PollUpdate(unsigned long* measStart = NULL, byte tbAddr = LTC2949_VAL_TB1);
byte LTC2949_ReadFastCfg(boolean* ch1, boolean* ch2, boolean* pasv, boolean* aux, boolean* cont);
byte LTC2949_GoCont(byte cfgFast = 0, byte adcCfg = 0, unsigned long* startOfMeas = NULL, bool clrAccus = true);

void LTC2949_WriteFastCfg(boolean ch1, boolean ch2, boolean aux, boolean cont);
void LTC2949_WriteFastCfg(byte cfgFast);
void LTC2949_ReportFaults(byte* dataBuffer, byte len);
byte LTC2949_ReadChkStatusFaults(
	boolean lockMemAndClr,
	boolean printResult,
	byte len = 10, // default is to read all status & fault regs
	byte * statFaultsExpAndRd = NULL, // read values are ignored if NULL!!!!
	boolean * expChkFailed = NULL, // no check result if null
	byte expDefaultSet = LTC2949_STATFAULTSCHK_DEFAULT_SETTING);

byte LTC2949_READ(uint16_t ADDR, uint16_t LEN, byte* DATAPOINTER);
void LTC2949_WRITE(uint16_t ADDR, uint16_t LEN, byte* DATAPOINTER);
void spiStart();
void spiStop();
void spi_write(byte data);
byte spi_read(/*byte data*/);
boolean spi_read_chk_pec(uint16_t* pec15, byte* errCnt = NULL);
void spi_write_calc_pec(uint16_t* pec15, byte data);
void spi_write_pec(uint16_t* pec15);
byte spi_read_calc_pec(uint16_t* pec15);
uint16_t LTC2949_CALC_MUL_6(uint16_t len);
byte LTC2949_CALC_OPT_NO_PECS(uint16_t len);
byte LTC2949_CALC_ID_BYTE(byte bytesPerPec, boolean readNotWrite);
void LTC2949_reset();
byte LTC2949_RdFifoCounters(byte fifoAddr, uint16_t* wrCnt, uint16_t* rdCnt);
byte LTC2949_RdFifoSampleCount(byte fifoAddr, uint16_t* samples);
byte LTC2949_ReadFifo(
	byte addr,
	uint16_t* len,
	int16_t* samples,
	boolean* fifoFull1stSample = NULL,
	int8_t rdOvrCntDown = LTC2949_RDFIFO_REQU_SAMPLES,
	byte* cellMonData = NULL);
float LTC2949_PLsb(boolean p1Notp2);
float LTC2949_PLsb(boolean p1Notp2, float rsns);
float LTC2949_SlotLsb(boolean slot1NotSlot2);
void LTC2949_ReorderData(uint16_t* data, uint8_t len);

#ifdef LTCDEF_SPI_POLL_SUPPORT
byte LTC2949_68XX_CMD(
	uint8_t addr,
	uint16_t cmd,
	byte* data = NULL,
	uint16_t len = 0,
	byte * data2949 = NULL,
	uint16_t len2949 = 0,
	uint8_t pollTimeout16us = 0
);
#else
byte LTC2949_68XX_CMD(
	uint8_t addr,
	uint16_t cmd,
	byte* data = NULL,
	uint16_t len = 0,
	byte * data2949 = NULL,
	uint16_t len2949 = 0
);
#endif


#ifdef LTC2949_SWBYPGETPROM
uint8_t LTC2949_SwBypGetPROM(uint32_t* prom, uint8_t len);
#endif

uint8_t LTC2949_SwBypWrite(uint8_t address, uint32_t ui24);
uint8_t LTC2949_SwBypRead(uint8_t address, uint32_t* ui24);
uint8_t LTC2949_SwBypGetTrimms(int8_t* gcaux, int8_t* gci1, int8_t* gci2);
void LTC2949_EnterDebug();
void LTC2949_ExitDebug();
uint8_t LTC2949_GetSiliconRevision(uint8_t* revCode, bool verbose = false);
void LTC2949_ClrRdcvConfBcren();

void LTC2949_OCCConfig(
	int8_t occ1Thr, uint8_t occ1Dgl, uint8_t occ1Pol,
	int8_t occ2Thr, uint8_t occ2Dgl, uint8_t occ2Pol);


// calculate temperature of NTC via Steinhart–Hart equation
// see LTC2949 on how to calculate Steinhart–Hart parameters A, B, C
float LTC2949_CalcNTCTemp(
	float vref, //!< VREF pin voltage (can also be set constant to the nominal value of 3.0V)
	float vntc, //!< NTC voltage versus GND (measured via Vx)
	float rRef, //!< Reference resistor connected between VREF pin and NTC.
	float ntcA, //!< NTC's Steinhart–Hart parameter A
	float ntcB, //!< NTC's Steinhart–Hart parameter B
	float ntcC  //!< NTC's Steinhart–Hart parameter C
);

//void LTC2949_ADCConfigUpdate(byte data);


static inline void LTC2949_isoSPI_wakeup_timeoutUs_init()
{
	// store the next timeout time stamp
		// 68xx tIDLE Idle Timeout Duration  > 4.3 ms
		// resolution of Arduino is 8 us
	LTC2949_isoSPI_wakeup_timeoutUs = micros() + (4300 - 8);
}

static inline int16_t LTC2949_GetRegsCtrlDataOffset(byte addr, uint16_t len)
{
	if (len > 4)
		return -1;

	int16_t i = (int16_t)LTC2949_REG_REGSCTRL - (int16_t)addr;
	if ((i >= 0) && ((uint16_t)i < len))
		return i;
	return -1;
	/* e.g.
	addr len i         ret
	0xF0 1   0xF  (15) false 15
	0xEF 17  0x10 (16) true  16
	0xfc 4   0x3  (3)  true  3
	*/
}

static inline int16_t LTC2949_GetFACTRLDataOffset(byte addr, uint16_t len)
{
	if (len > 7)
		return -1;

	int16_t i = (int16_t)LTC2949_REG_FACTRL - (int16_t)addr;
	if ((i >= 0) && ((uint16_t)i < len))
		return i;
	return -1;
}

static inline int16_t LTC2949_GetADCCONFDataOffset(byte addr, uint16_t len)
{
	if (len > (16 * 15))
		return -1;

	int16_t i = (uint8_t)LTC2949_REG_ADCCONF - (int16_t)addr;
	if ((i >= 0) && ((uint16_t)i < len))
		return i;
	return -1;
}


static inline byte LTC2949_68XX_RdCells(uint16_t rdcv, uint16_t* data)
{
	byte error = LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ rdcv,
		/*byte *data:         */ (byte*)data,
		/*uint16_t len:       */ LTC2949_CellMonitorCount * 6U
	);
	LTC2949_ReorderData(data, LTC2949_CellMonitorCount * 3U);
	return error;
}

static inline byte LTC2949_68XX_RdAux(uint16_t rdaux, uint16_t* data)
{
	byte error = LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ rdaux,
		/*byte *data:         */ (byte*)data,
		/*uint16_t len:       */ LTC2949_CellMonitorCount * 6U
	);
	LTC2949_ReorderData(data, LTC2949_CellMonitorCount * 3U);
	return error;
}

static inline byte LTC2949_68XX_ClrCells()
{
	return LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ LTC2949_68XX_CMD_CLRCELL
	);
}

static inline byte LTC2949_68XX_ClrAux()
{
	return LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ LTC2949_68XX_CMD_CLRAUX
	);
}




/******* for the read configure function *******/

static inline byte LTC2949_68XX_RdCfg(byte* data)
{
	return LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ LTC2949_68XX_CMD_RDCFG,
		/*byte *data:         */ (byte*)data,
		/*uint16_t len:       */ LTC2949_CellMonitorCount * 6U
	);
}

// The new function to be added is the below one
static inline byte LTC2949_68XX_RdCfgb(byte* data)
{
	return LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ LTC2949_68XX_CMD_RDCFGB,
		/*byte *data:         */ (byte*)data,
		/*uint16_t len:       */ LTC2949_CellMonitorCount * 6U
	);
}


/******* for the write configure function *******/

static inline byte LTC2949_68XX_WrCfg(byte* data)
{
	return LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ LTC2949_68XX_CMD_WRCFG,
		/*byte *data:         */ (byte*)data,
		/*uint16_t len:       */ LTC2949_CellMonitorCount * 6U
	);
}

// The new function to be added is the below one
static inline byte LTC2949_68XX_WrCfgb(byte* data)
{
	return LTC2949_68XX_CMD(
		/*uint8_t addr:       */ 0, // broadcast is mandatory in case of reading data from cell monitors
		/*uint16_t cmd:       */ LTC2949_68XX_CMD_WRCFGB,
		/*byte *data:         */ (byte*)data,
		/*uint16_t len:       */ LTC2949_CellMonitorCount * 6U
	);
}



// direct write without changing page!
static inline void _LTC2949_direct_write_(byte addr, uint16_t len, byte* data)
{
	_LTC2949_DcRW_(addr, len, data, LTC2949_DcWrite);
}

// direct write without changing page!
static inline void _LTC2949_direct_write_(byte addr, byte data)
{
	_LTC2949_DcRW_(addr, 1, &data, LTC2949_DcWrite);
}

// direct read without changing page!
//static inline byte _LTC2949_direct_read_(byte addr, uint16_t len, byte *data)
//{
//	return _LTC2949_DcRW_(addr, len, data, LTC2949_DcRead);
//}
// direct read without changing page!
//static inline byte _LTC2949_direct_read_(byte addr, byte *data)
//{
//	return _LTC2949_DcRW_(addr, 1, data, LTC2949_DcRead);
//}

// write with auto page adjust
static inline void LTC2949_WRITE(uint16_t ADDR, byte DATA)
{
	LTC2949_WRITE(ADDR, 1, &DATA);
}

static inline byte LTC2949_ReadFifo(
	byte addr,
	uint16_t len,
	int16_t* samples)
{
	return LTC2949_ReadFifo(
		/*byte addr:        */ addr,
		/*uint16_t* len:    */ &len,
		/*int16_t * samples:*/ samples);
}

static inline boolean LTC2949_Slot1IsTemp()
{
	return bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC1);
}

static inline boolean LTC2949_Slot2IsTemp()
{
	return bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_NTC2);
}

static inline boolean LTC2949_P1IsVolt()
{
	return bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P1ASV);
}

static inline boolean LTC2949_P2IsVolt()
{
	return bitMaskSetChk(LTC2949_adcConf, LTC2949_BM_ADCCONF_P2ASV);
}

float LTC2949_P1DataToReal(byte* data);
float LTC2949_P2DataToReal(byte* data);
float LTC2949_Slot1DataToReal(byte* data);
float LTC2949_Slot2DataToReal(byte* data);

void LTC2949_ProcessErrors(byte err);

byte LTC2949_ADCConfigRead(byte* data);




// <------- FUNCTION PROTOTYPES END HERE!!!! -------->
// INLINES

#ifdef ARDUINO
// Connect SPI pins to QuikEval connector through the Linduino MUX. This will disconnect I2C.
static inline void LinduinoSelectSPI()
{
	digitalWrite(LTC2949_CS, HIGH);
	pinMode(LTC2949_CS, OUTPUT);

#if defined (__AVR__) || (__avr__)
	// Linduino only
	pinMode(8, OUTPUT); //QUIKEVAL_MUX_MODE_PIN
	digitalWrite(8, LOW);
#else
	//for Arduino DUE based on ARM IC
#endif
}
#endif

static inline void LinduinoSelectSPI(uint8_t cs)
{
	LinduinoSelectSPI(); // this ensures the previous is set high (inactive)
	LTC2949_CS = cs;
	LinduinoSelectSPI();
}


//// static inline byte LTC2949_GetADCVCmd0(
//// 	byte MD //ADC Mode
//// )
//// {
//// 	return ((MD & 0x02) >> 1) + 0x02;
//// }
//// 
//// static inline byte LTC2949_GetADCVCmd1(
//// 	byte MD, //ADC Mode
//// 	byte DCP, //Discharge Permit
//// 	byte CH //Cell Channels to be measured
//// )
//// {
//// 	return ((MD & 0x01) << 7) + 0x60 + (DCP << 4) + CH;
//// }

#ifdef LTCDEF_SPI_POLL_SUPPORT
// returns the time in multiples of 16
#define LTC2949_68XX_GETADCVTIMEOUT16US(US) (((US) >> 4) + 1U)
#else
#undef LTC2949_68XX_GETADCVTIMEOUT16US
#endif

// this function is for ADCV cmd

static inline uint16_t LTC2949_GetADCVCmd(
	byte MD, //ADC Mode
	byte DCP, //Discharge Permit
	byte CH //Cell Channels to be measured
#ifdef LTCDEF_SPI_POLL_SUPPORT
	, boolean poll // true for poll end of conversion, false for send ADCV only
#else
#endif
)
{
	return
		/*CMD0 / MSB:*/ (((MD & 0x02U) >> 1) | 0x02U) << 8 |
		/*CMD1 / LSB:*/ ((MD & 0x01U) << 7) | 0x60U | (DCP << 4) | CH |
#ifdef LTCDEF_SPI_POLL_SUPPORT
		(poll ? LTC2949_68XX_CMDTYPE_PL : LTC2949_68XX_CMDTYPE_WR)
#else
		LTC2949_68XX_CMDTYPE_WR
#endif
		;
}

// this function is for ADAX cmd

static inline uint16_t LTC2949_GetADAXCmd(
	byte MD, //ADC Mode
	byte DCP, //Discharge Permit
	byte CH //Cell Channels to be measured
#ifdef LTCDEF_SPI_POLL_SUPPORT
	, boolean poll // true for poll end of conversion, false for send ADCV only
#else
#endif
)
{
	return
		/*CMD0 / MSB:*/ (((MD & 0x02U) >> 1) | 0x02U) << 9 |
		/*CMD1 / LSB:*/ ((MD & 0x01U) << 7) | 0x60U | (DCP << 4) | CH |
#ifdef LTCDEF_SPI_POLL_SUPPORT
		(poll ? LTC2949_68XX_CMDTYPE_PL : LTC2949_68XX_CMDTYPE_WR)
#else
		LTC2949_68XX_CMDTYPE_WR
#endif
		;
}

/*!*********************************************************************************************
\brief Starts cell voltage conversion

Starts ADC conversions of the LTC2949 Cpin inputs.
The type of ADC conversion executed can be changed by setting the associated global variables:
|Variable|Function                                      |
|--------|----------------------------------------------|
| MD     | Determines the filter corner of the ADC      |
| CH     | Determines which cell channels are converted |
| DCP    | Determines if Discharge is Permitted       |

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
***********************************************************************************************/
static inline byte LTC2949_ADxx(
	byte md = MD_NORMAL,
	byte ch = CELL_CH_ALL,
	byte dcp = DCP_DISABLED
#ifdef LTCDEF_SPI_POLL_SUPPORT
	, uint8_t pollTimeout16us = 0
#else
#endif
) {
	// Note: ADCV will always be send as broadcast command
	byte error = LTC2949_68XX_CMD(
		/*uint8_t addr:           */ 0,
		/*uint16_t cmd:           */ LTC2949_GetADCVCmd(md, dcp, ch
#ifdef LTCDEF_SPI_POLL_SUPPORT
			, pollTimeout16us > 0
#else
#endif
		),
		/*byte *data:             */ NULL,
		/*uint16_t len:           */ 0,
		/*byte *data2949:         */ NULL,
		/*uint16_t len2949:       */ 0
#ifdef LTCDEF_SPI_POLL_SUPPORT
		,/*uint8_t pollTimeout16us:*/ pollTimeout16us
#else
#endif
	);
	return error;
}

/**** This function is for the ADAX command ****/

static inline byte LTC2949_ADAX(
	byte md = MD_NORMAL,
	byte ch = CELL_CH_ALL,
	byte dcp = DCP_DISABLED
#ifdef LTCDEF_SPI_POLL_SUPPORT
	, uint8_t pollTimeout16us = 0
#else
#endif
) {
	// Note: ADCV will always be send as broadcast command
	byte error = LTC2949_68XX_CMD(
		/*uint8_t addr:           */ 0,
		/*uint16_t cmd:           */ LTC2949_GetADAXCmd(md, dcp, ch
#ifdef LTCDEF_SPI_POLL_SUPPORT
			, pollTimeout16us > 0
#else
#endif
		),
		/*byte *data:             */ NULL,
		/*uint16_t len:           */ 0,
		/*byte *data2949:         */ NULL,
		/*uint16_t len2949:       */ 0
#ifdef LTCDEF_SPI_POLL_SUPPORT
		,/*uint8_t pollTimeout16us:*/ pollTimeout16us
#else
#endif
	);
	return error;
}


// same as above, but this will trigger LTC2949 only (cell monitors will ignore it!)
static inline byte LTC2949_ADxxAddressed(
	byte md = MD_NORMAL,
	byte ch = CELL_CH_ALL,
	byte dcp = DCP_DISABLED
#ifdef LTCDEF_SPI_POLL_SUPPORT
	, uint8_t pollTimeout16us = 0
#else
#endif
) {
	// Note: ADCV will always be send as broadcast command
	byte error = LTC2949_68XX_CMD(
		/*uint8_t addr:           */ LTC2949_ADDRESS,
		/*uint16_t cmd:           */ LTC2949_GetADCVCmd(md, dcp, ch
#ifdef LTCDEF_SPI_POLL_SUPPORT
			, pollTimeout16us > 0
#else
#endif
		),
		/*byte *data:             */ NULL,
		/*uint16_t len:           */ 0,
		/*byte *data2949:         */ NULL,
		/*uint16_t len2949:       */ 0
#ifdef LTCDEF_SPI_POLL_SUPPORT
		,/*uint8_t pollTimeout16us:*/ pollTimeout16us
#else
#endif
	);

	return error;
}


// fast channel access
static inline void LTC2949_FastOff()
{
	_LTC2949_direct_write_(LTC2949_REG_FACTRL, LTC2949_FACTRL_FAST_OFF);
}

static inline void LTC2949_FastAllEnable()
{
	_LTC2949_direct_write_(LTC2949_REG_FACTRL, LTC2949_FACTRL_FAST_ALLEN);
}

static inline void LTC2949_FastAllEnableCont()
{
	_LTC2949_direct_write_(LTC2949_REG_FACTRL, LTC2949_FACTRL_FAST_ALLCONTEN);
}

static inline void LTC2949_WriteFastCfgCont(boolean ch1, boolean ch2, boolean aux)
{
	LTC2949_WriteFastCfg(ch1, ch2, aux, ch1 | ch2 | aux);
}

static inline void LTC2949_WriteFastCfgSsht(boolean ch1, boolean ch2, boolean aux)
{
	LTC2949_WriteFastCfg(ch1, ch2, aux, false);
}

static inline boolean LTC2949_iAddrRegsCtrl_GetPage()
{
	return bitMaskSetChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_PAGE);
}
/*
static inline boolean LTC2949_GetPageShadowBit()
{
	return bitMaskSetChk(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_PAGE);
}
*/

static inline void LTC2949_PageSelect(boolean page)
{
	bitMaskSetClr(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_PAGE, page);
	_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, LTC2949_iAddrRegsCtrl + LTC2949_REGSCTRL_IX);
}

static inline void LTC2949_Rdconf(boolean en)
{
	bitMaskSetClr(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX], LTC2949_BM_REGSCTRL_RDCVCONF, en);
	_LTC2949_direct_write_(LTC2949_REG_REGSCTRL, 1, LTC2949_iAddrRegsCtrl + LTC2949_REGSCTRL_IX);
}

static inline void LTC2949_OpctlCont()
{
	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_CONT);
}

static inline void LTC2949_OpctlSshot()
{
	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_SSHOT);
}

static inline void LTC2949_OpctlIdle()
{
	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, 0);
}

static inline void LTC2949_OpctlClr()
{
	LTC2949_lastTBx = 0;
	_LTC2949_direct_write_(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_CLR);
}

static inline void LTC2949_OverCurrentCompCfg(byte occ1ctrl, byte occ2ctrl)
{
	byte data[2] = { occ1ctrl, occ2ctrl };

	LTC2949_WRITE(LTC2949_REG_OCC1CTRL, 2, data);
}

static inline void LTC2949_OverCurrentCompCfg(byte* occ12ctrl)
{
	LTC2949_WRITE(LTC2949_REG_OCC1CTRL, 2, occ12ctrl);
}

static inline void LTC2949_GpioCurrConfigWrite()
{
	_LTC2949_direct_write_(LTC2949_REG_FCURGPIOCTRL, 2, LTC2949_gpioCtrl);
}

static inline void LTC2949_GpioCurrConfigClr()
{
	LTC2949_gpioCtrl[0] = 0;
	LTC2949_gpioCtrl[1] = 0;
}

static inline void LTC2949_GpioCurrConfigSet(byte* gpioCurrCtrl)
{
	LTC2949_gpioCtrl[0] = gpioCurrCtrl[0];
	LTC2949_gpioCtrl[1] = gpioCurrCtrl[1];
}

static inline void LTC2949_Gpo5CurrConfigSet(byte gpo5CurrCtrl)
{
	LTC2949_gpioCtrl[0] = gpo5CurrCtrl;
}

static inline void LTC2949_Gpo1to4ConfigSet(byte gpo1to4CurrCtrl)
{
	LTC2949_gpioCtrl[1] = gpo1to4CurrCtrl;
}

void LTC2949_SlotsCfg(byte slot1P, byte slot1N, byte slot2P, byte slot2N);

byte LTC2949_GetSlotsCfg(byte* slot1P, byte* slot1N, byte* slot2P, byte* slot2N);

void LTC2949_SlotFastCfg(byte slotP, byte slotN);

byte LTC2949_GetSlotFastCfg(byte* slotP, byte* slotN);

static inline float LTC2949_Slot1DataToReal(byte* data, byte offset)
{
	return LTC2949_Slot1DataToReal(data + offset);
}

static inline float LTC2949_Slot2DataToReal(byte* data, byte offset)
{
	return LTC2949_Slot2DataToReal(data + offset);
}

static inline float LTC2949_P1DataToReal(byte* data, byte offset)
{
	return LTC2949_P1DataToReal(data + offset);
}

static inline float LTC2949_P2DataToReal(byte* data, byte offset)
{
	return LTC2949_P2DataToReal(data + offset);
}

static inline void LTC2949_FloatToF24Bytes(float f32, byte* bytes, byte offset)
{
	LTC2949_FloatToF24Bytes(f32, bytes + offset);
}

static inline void LTC2949_F24BytesToFloat(byte* bytes, byte offset, float* f32)
{
	LTC2949_F24BytesToFloat(bytes + offset, f32);
}

#define LTC2949_FAST_COMMANDS
#ifdef LTC2949_FAST_COMMANDS

static inline void LTC2949_FastSPI(void* buffer, const void* cmd, size_t len, size_t wrlen, boolean begin = true, boolean end = true)
{
#ifdef ARDUINO
	memcpy(buffer, cmd, wrlen);
	if (begin)
		SPI.beginTransaction(LTC2949_SPISettings);
	digitalWrite(LTC2949_CS, LOW);
	SPI.transfer(buffer, len);
	digitalWrite(LTC2949_CS, HIGH);
	if (end)
		SPI.endTransaction();
#else
	memcpy(spiBuffer, cmd, wrlen);
	spiBufferLen = len;
	spiBufferBusy = true;
	spiStop();
	memcpy(buffer, spiBuffer, len);
#endif
}

static inline void LTC2949_FastSPI(void* buffer, const void* cmd, size_t len, boolean begin = true, boolean end = true)
{
	LTC2949_FastSPI(buffer, cmd, len, len, begin, end);
}

////// static inline void LTC2949_FastSPI_DcRW(void* buffer, size_t len, size_t wrlen, boolean begin = true, boolean end = true)
////// {
////// 	byte cmd[5];
////// 	cmd[0] = 0xFE;
////// 	memcpy(buffer, cmd, wrlen);
////// 	if (begin)
////// 		SPI.beginTransaction(LTC2949_SPISettings);
////// 	digitalWrite(LTC2949_CS, LOW);
////// 	SPI.transfer(buffer, len);
////// 	digitalWrite(LTC2949_CS, HIGH);
////// 	if (end)
////// 		SPI.endTransaction();
////// }


#define LTC2949_RDCV_I_I1   (4)
#define LTC2949_RDCV_I_I2   (6)
#define LTC2949_RDCV_I_BAT  (8)
#define LTC2949_RDCV_I_PECA (10)
#define LTC2949_RDCV_I_AUX  (12)
#define LTC2949_RDCV_I_HS0  (14)
#define LTC2949_RDCV_I_HSE  (LTC2949_RDCV_I_HS0+3)
static inline int16_t LTC2949_FastSPIDecode_RdcvGetAux(byte* buffer)
{
	return buffer[LTC2949_RDCV_I_AUX + 1] << 8 | buffer[LTC2949_RDCV_I_AUX];
}
static inline int16_t LTC2949_FastSPIDecode_RdcvGetBat(byte* buffer)
{
	return buffer[LTC2949_RDCV_I_BAT + 1] << 8 | buffer[LTC2949_RDCV_I_BAT];
}
static inline byte LTC2949_FastSPIDecode_RdcvGetHS0(byte* buffer)
{
	return buffer[LTC2949_RDCV_I_HS0];
}
static inline uint16_t LTC2949_FastSPIDecode_GetPec(byte* buffer, uint8_t bytesPerPec)
{
	return buffer[bytesPerPec] << 8 | buffer[bytesPerPec + 1]; // (PEC0<<8 | PEC1) == pec15;
}
static inline boolean LTC2949_FastSPIDecode_ChkPec(byte* buffer, uint8_t bytesPerPec)
{
	return pec15_calc_byte_array(buffer, bytesPerPec) == LTC2949_FastSPIDecode_GetPec(buffer, bytesPerPec);
}

static const byte _cmd_adcv_a[] = { 0xFB,0x60,0xFA,0xDE };
static const unsigned int _cmd_adcv_a_len = sizeof(_cmd_adcv_a);
static inline void LTC2949_FastSPI_adcv_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_adcv_a, _cmd_adcv_a_len, begin, end); }

/*                                     0    1    2    3    I1L  I1H  I2L  I2H  BL   BH   PL   PH   AL   AH   HS   HS   HS   HS   PL   PH
static const byte _cmd_rdcvhs_a[] = { 0xF8,0x04,0x09,0x70,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };*/
static const byte _cmd_rdcvhs_a[] = { 0xF8,0x04,0x09,0x70 };
static const unsigned int _cmd_rdcvhs_a_len = 20;
static const unsigned int _cmd_rdcvhs_a_wrlen = 4;
static inline void LTC2949_FastSPI_rdcvhs_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_rdcvhs_a, _cmd_rdcvhs_a_len, _cmd_rdcvhs_a_wrlen, begin, end); }

/*                                     0    1    2    3    I1L  I1H  I2L  I2H  BL   BH   PL   PH   AL   AH   HS
static const byte _cmd_rdcv1hs_a[] = { 0xF8,0x04,0x09,0x70,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };*/
static const byte _cmd_rdcv1hs_a[] = { 0xF8,0x04,0x09,0x70 };
static const unsigned int _cmd_rdcv1hs_a_len = 15;// sizeof(_cmd_rdcv1hs_a);
static const unsigned int _cmd_rdcv1hs_a_wrlen = 4;
static inline void LTC2949_FastSPI_rdcv1hs_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_rdcv1hs_a, _cmd_rdcv1hs_a_len, _cmd_rdcv1hs_a_wrlen, begin, end); }

static const byte _cmd_wrfamux_v1vsgnd_a[] = { 0xFE,0xF3,0xC7,0x98,0x45,0x00,0x01,0x3D,0x6E };
static const unsigned int _cmd_wrfamux_v1vsgnd_a_len = sizeof(_cmd_wrfamux_v1vsgnd_a);
static inline void LTC2949_FastSPI_wrfamux_v1vsgnd_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_wrfamux_v1vsgnd_a, _cmd_wrfamux_v1vsgnd_a_len, begin, end); }

static const byte _cmd_wrfamux_vref2vsgnd_a[] = { 0xFE,0xF3,0xC7,0x98,0x45,0x00,0x16,0xC1,0xBA };
static const unsigned int _cmd_wrfamux_vref2vsgnd_a_len = sizeof(_cmd_wrfamux_vref2vsgnd_a);
static inline void LTC2949_FastSPI_wrfamux_vref2vsgnd_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_wrfamux_vref2vsgnd_a, _cmd_wrfamux_vref2vsgnd_a_len, begin, end); }

// FACTRL = FACH1, FACH2, FACHA
static const byte _cmd_wrfactrl0e_a[] = { 0xFE,0xF5,0xEB,0x50,0x40,0x0E,0xE4,0xC6 };
static const unsigned int _cmd_wrfactrl0e_a_len = sizeof(_cmd_wrfactrl0e_a);
static inline void LTC2949_FastSPI_wrfactrl0e_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_wrfactrl0e_a, _cmd_wrfactrl0e_a_len, begin, end); }

// FACTRL = FACH1, FACH2, FACHA, FACONV
static const byte _cmd_wrfactrl0f_a[] = { 0xFE,0xF5,0xEB,0x50,0x40,0x0F,0x6F,0xF4 };
static const unsigned int _cmd_wrfactrl0f_a_len = sizeof(_cmd_wrfactrl0f_a);
static inline void LTC2949_FastSPI_wrfactrl0f_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_wrfactrl0f_a, _cmd_wrfactrl0f_a_len, begin, end); }

static const byte _cmd_wrfamux_gndvsvref2_250k_a[] = { 0xFE,0xF3,0xC7,0x98,0x45,0x17,0x00,0x75,0x12 };
static const unsigned int _cmd_wrfamux_gndvsvref2_250k_a_len = sizeof(_cmd_wrfamux_gndvsvref2_250k_a);
static inline void LTC2949_FastSPI_wrfamux_gndvsvref2_250k_a(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_wrfamux_gndvsvref2_250k_a, _cmd_wrfamux_gndvsvref2_250k_a_len, begin, end); }

static const byte _cmd_wrfamux_vref2_250kvsgnd_a[] = { 0xFE,0xF3,0xC7,0x98,0x45,0x00,0x17,0x4A,0x88 };
static const unsigned int _cmd_wrfamux_vref2_250kvsgnd_a_len = sizeof(_cmd_wrfamux_vref2_250kvsgnd_a);
static inline void LTC2949_FastSPI_wrfamux_vref2_250kvsgnd_(void* buffer, boolean begin = true, boolean end = true) { LTC2949_FastSPI(buffer, _cmd_wrfamux_vref2_250kvsgnd_a, _cmd_wrfamux_vref2_250kvsgnd_a_len, begin, end); }

static const unsigned int _cmd_max_len = _cmd_rdcvhs_a_len;

#endif


#if defined(LTC2949_DCRW_RAW_DATA_DEBUG) && defined(ARDUINO)
uint16_t LTC2949_GetDcRwRawData(byte** dp);
#endif



#endif
