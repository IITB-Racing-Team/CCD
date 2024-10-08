#include <Defination.h>

void cellVoltageLoop (unsigned long timeBuffer)
{
  	byte  error = 0;
    error |= LTC2949_ADxx(
		/*byte md = MD_NORMAL     : */MD_FAST,
		/*byte ch = CELL_CH_ALL   : */CELL_CH_ALL,
		/*byte dcp = DCP_DISABLED : */DCP_DISABLED,
		/*uint8_t pollTimeout = 0 : */LTCDEF_POLL_EOC ? LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_68XX_T6C_27KHZ_US) : 0
	);


#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif


	cellMonDat[0] = 0xFFFFU; // will be used later

#ifdef LTCDEF_LTC681X_ONLY
	timeBuffer = micros() + LTC2949_68XX_T6C_27KHZ_US;
	// for sure we poll for HS of LTC2949's rdcv later, so we do not have to wait here!
	while (!LTC_TIMEOUT_CHECK(micros(), timeBuffer))
		; // wait for all cell voltage measurements to be completed.
	error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVA, cellMonDat);

#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif

#else
if(loopcount){
	// if (!LTCDEF_POLL_EOC)
	// {
	// 	timeBuffer = micros();
	// 	// for sure we poll for HS of LTC2949's rdcv later, so we do not have to wait here!
	// 	while (false && !LTC_TIMEOUT_CHECK(micros(), timeBuffer + LTC2949_68XX_T6C_27KHZ_US))
	// 		; // wait for all cell voltage measurements to be completed.
	// }
	fastData2949[LTC2949_RDFASTDATA_HS] = 0; // clear the HS bytes
	// poll LTC2949 for conversion done
	error |= LTC2949_RdFastData(
		fastData2949,
		cellMonDat,
		LTC2949_68XX_CMD_RDCVA,
    // LTC2949_68XX_CMD_RDAUXA,
		LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_FASTSSHT_RDY_TIME_US));
	// calc cycle time (ADCV to "results ready")
	deltaT = micros() - timeBuffer;
	// check if we already read valid data from LTC2949
	if (LTC2949_FASTSSHT_HS_OK(fastData2949))
	{
		; // all fine, nothing to do.
	}
	else if (LTC2949_FASTSSHT_HS_CLR(fastData2949))
	{
		// we polled for HS==0x0F before, so it cannot be all HS are zero! something went wrong (e.g. timeout)
		error |= LTC2949_ERRCODE_OTHER;
	}
	else if (LTC2949_FASTSSHT_HS_LAST_OK(fastData2949)) // first HS != 0x0F, last HS == 0x0F
	{
		// we have to read data from LTC2949 again, as only the next RDCVx will report the final conversion results
		// also cell voltages have to be read again, as also those most probably were not updated
		// note: here we must not poll HS! (it must be zero now!)
		error |= LTC2949_RdFastData(
			fastData2949,
			cellMonDat,
			LTC2949_68XX_CMD_RDCVA);
      // LTC2949_68XX_CMD_RDAUXA);

		if (!LTC2949_FASTSSHT_HS_CLR(fastData2949)) // HS must be cleared now
			error |= LTC2949_ERRCODE_OTHER; // this must never happen in case of fast single shot events
	}
	else
	{
		// Unexpected HS bytes, something went wrong
		error |= LTC2949_ERRCODE_OTHER;
	}
} 
#endif

	char errorExt = error > 1 ? 'X' : '_';
	if (!LTC2949_onTopOfDaisychain || (cellMonDat[0] == 0xFFFFU))
	{
		if (errorExt == '_') errorExt = 'Y';

		// we have to read cell voltages group A (again)
		// for sure we have to read in case LTC2949 is not on top of daisychain!
		error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVA, cellMonDat);
	}

		String cvs[LTCDEF_CELL_MONITOR_COUNT];

		for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
			cvs[i] = "";

		for (uint8_t rdcvi = 0; ; rdcvi++)
		{
			for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
			{
        cvs[i] += CellVoltagesToString(cellMonDat, i);
        transferV(cellMonDat,i,rdcvi);
      }

			if ((rdcvi > 4) || (rdcvi >= (LTCDEF_CELLS_PER_CELL_MONITOR_COUNT / 3 - 1)))
				break;

			switch (rdcvi)
			{
			case 0:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVB, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'b';
				break;
			case 1:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVC, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'c';
				break;
			case 2:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVD, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'd';
				break;
			case 3:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVE, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'e';
				break;
			case 4:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVF, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'f';
				break;
			
		  }
		// for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
		// {
    //   str += cvs[i]; //Serial.print(cvs[i]);
		// 	cvs[i] = "";
		// }
		// str += errorExt; //Serial.print(errorExt);
		// str += ',';//PrintComma();
	  }
  circularVoltdef();
  cellsVoltSort();
  checkVoltNormalize();
  checkVoltSplit();
  cellsLogging();
  #ifndef GUI_Enabled
  printCells();
  #endif
  

}

void cellsVoltSort(void)
{
  for ( int c_ic = 0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    #ifndef seventhSlave
      for ( int i=0; i < 18; i++ )
      {
        if ( (i != 5) && (i != 11) && (i != 17) )       // these pins are GNDed in the brd file 
        {
          if ( i < 5 )
          {
            cellVoltages[c_ic][i] = cells[c_ic][i];
          }
          else if ( (i < 11) && (i > 5) )
          {
            cellVoltages[c_ic][i-1] = cells[c_ic][i];
          }
          else 
          {
            cellVoltages[c_ic][i-2] = cells[c_ic][i];
          }
        }
      }
    #else
      if ( c_ic < 6 )
      {
        for ( int i=0; i < 18; i++ )
        {
        if ( (i != 5) && (i != 11) && (i != 17) )       // these pins are GNDed in the brd file 
          {
            if ( i < 5 )
            {
              cellVoltages[c_ic][i] = cells[c_ic][i];
            }
            else if ( (i < 11) && (i > 5) )
            {
              cellVoltages[c_ic][i-1] = cells[c_ic][i];
            }
            else 
            {
              cellVoltages[c_ic][i-2] = cells[c_ic][i];
            }
          }
        }
      }

      else if ( c_ic == 6 )
      {
        for ( uint8_t i=0; i<18; i++ )
        {
          if ( (i==0) || (i==1) || (i==6) || (i==7) || (i==12) || (i==13) )
          {
            if ( i < 2 )
            {
              cellVoltages[c_ic][i] = cells[c_ic][i];
            }
            else if ( (i < 8) && ( i > 5) )
            {
              cellVoltages[c_ic][i-4] = cells[c_ic][i];
            }
            else
            {
              cellVoltages[c_ic][i-8] = cells[c_ic][i];
            }
          }
          // else
          // {
          //   if ( i < 15 )
          //   {
          //     cellVoltages[c_ic][i] = 3.5000;
          //   }
          // }
        }
        for ( uint8_t i=6; i<15; i++ )
        {
          cellVoltages[c_ic][i] = 3.5000;
        }
      }
    #endif
  }
}

void printCells(void)
{
  for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    #ifndef seventhSlave
      Serial.println();
      Serial.print("IC : ");
      Serial.print(c_ic+1);

      for (uint8_t i=0; i < 15; i++ )
      {
        Serial.print(" C");
        Serial.print(i+1);
        Serial.print(":");
        Serial.print(cellVoltages[c_ic][i], 4);
        Serial.print(",");
      }
    #else
      if ( c_ic < 6 )
      {
        Serial.println();
        Serial.print("IC : ");
        Serial.print(c_ic+1);

        for (uint8_t i=0; i < 15; i++ )
        {
          Serial.print(" C");
          Serial.print(i+1);
          Serial.print(":");
          Serial.print(cellVoltages[c_ic][i], 4);
          Serial.print(",");
        }
      }

      else if ( c_ic == 6 )
      {
        Serial.println();
        Serial.print("IC : ");
        Serial.print(c_ic+1);

        for (uint8_t i=0; i < 6; i++ )
        {
          Serial.print(" C");
          Serial.print(i+1);
          Serial.print(":");
          Serial.print(cellVoltages[c_ic][i], 4);
          Serial.print(",");
        }
      }
    #endif
  }
}

void voltagePlausibilityCheck(void)
{
  double ti = millis(),delT = 0.0;
  while( delT < voltTimer )
  {
    clearFlagsV();
    cellVoltageLoop(0);
    checkVoltageFlag();

    if ( errorFlag[0] == -1 || errorFlag[1] == -1 || errorFlag[2] == -1 || errorFlag[3] == -1 )
    {
      delT = millis() - ti ;
      voltFlag = true;
    }
    else
    { 
      voltFlag = false;
      break;
    }
  }
}

void cellTempLoop(unsigned long timeBuffer)
{
  for ( int muxSelect = 1; muxSelect >= 0; muxSelect--)
  {
     byte  error = 0;
     error |= CellMonitorCFGB((byte*)cellMonDat, false , muxSelect); 	
        
        error |= LTC2949_ADAX(
		/*byte md = MD_NORMAL     : */MD_FAST,
		/*byte ch = CELL_CH_ALL   : */CELL_CH_ALL,
		/*byte dcp = DCP_DISABLED : */DCP_DISABLED,
		/*uint8_t pollTimeout = 0 : */LTCDEF_POLL_EOC ? LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_68XX_T6C_27KHZ_US) : 0
	);


#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif


	cellMonDat[0] = 0xFFFFU; // will be used later

#ifdef LTCDEF_LTC681X_ONLY
	timeBuffer = micros() + LTC2949_68XX_T6C_27KHZ_US;
	// for sure we poll for HS of LTC2949's rdcv later, so we do not have to wait here!
	while (!LTC_TIMEOUT_CHECK(micros(), timeBuffer))
		; // wait for all cell voltage measurements to be completed.
	error |= LTC2949_68XX_RdAux(LTC2949_68XX_CMD_RDAUXA, cellMonDat);

#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif

#else
if(loopcount){
	if (!LTCDEF_POLL_EOC)
	{
		timeBuffer = micros();
		// for sure we poll for HS of LTC2949's rdcv later, so we do not have to wait here!
		while (false && !LTC_TIMEOUT_CHECK(micros(), timeBuffer + LTC2949_68XX_T6C_27KHZ_US))
			; // wait for all cell voltage measurements to be completed.
	}
	fastData2949[LTC2949_RDFASTDATA_HS] = 0; // clear the HS bytes
	// poll LTC2949 for conversion done
	error |= LTC2949_RdFastData(
		fastData2949,
		cellMonDat,
		// LTC2949_68XX_CMD_RDCVA,
    LTC2949_68XX_CMD_RDAUXA,
		LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_FASTSSHT_RDY_TIME_US));
	// calc cycle time (ADCV to "results ready")
	deltaT = micros() - timeBuffer;
	// check if we already read valid data from LTC2949
	if (LTC2949_FASTSSHT_HS_OK(fastData2949))
	{
		; // all fine, nothing to do.
	}
	else if (LTC2949_FASTSSHT_HS_CLR(fastData2949))
	{
		// we polled for HS==0x0F before, so it cannot be all HS are zero! something went wrong (e.g. timeout)
		error |= LTC2949_ERRCODE_OTHER;
	}
	else if (LTC2949_FASTSSHT_HS_LAST_OK(fastData2949)) // first HS != 0x0F, last HS == 0x0F
	{
		// we have to read data from LTC2949 again, as only the next RDCVx will report the final conversion results
		// also cell voltages have to be read again, as also those most probably were not updated
		// note: here we must not poll HS! (it must be zero now!)
		error |= LTC2949_RdFastData(
			fastData2949,
			cellMonDat,
			// LTC2949_68XX_CMD_RDCVA);
      LTC2949_68XX_CMD_RDAUXA);

		if (!LTC2949_FASTSSHT_HS_CLR(fastData2949)) // HS must be cleared now
			error |= LTC2949_ERRCODE_OTHER; // this must never happen in case of fast single shot events
	}
	else
	{
		// Unexpected HS bytes, something went wrong
		error |= LTC2949_ERRCODE_OTHER;
	}
}
#endif

	char errorExt = error > 1 ? 'X' : '_';
	if (!LTC2949_onTopOfDaisychain || (cellMonDat[0] == 0xFFFFU))
	{
		if (errorExt == '_') errorExt = 'Y';

		// we have to read cell voltages group A (again)
		// for sure we have to read in case LTC2949 is not on top of daisychain!
		error |= LTC2949_68XX_RdAux(LTC2949_68XX_CMD_RDAUXA, cellMonDat);
	}
	
		String cvs[LTCDEF_CELL_MONITOR_COUNT];

		for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
			cvs[i] = "";

		for (uint8_t rdcvi = 0; rdcvi < 3; rdcvi++)
		{
			for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
      {
				cvs[i] += CellVoltagesToString(cellMonDat, i);
        transferT(cellMonDat,i,muxSelect,rdcvi);
      }
			// if ((rdcvi > 4) || (rdcvi >= (LTCDEF_CELLS_PER_CELL_MONITOR_COUNT / 3 - 1)))
			// 	break;

			switch (rdcvi)
			{
			case 0:
				error |= LTC2949_68XX_RdAux(LTC2949_68XX_CMD_RDAUXB, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'b';
				break;
			case 1:
				error |= LTC2949_68XX_RdAux(LTC2949_68XX_CMD_RDAUXC, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'c';
				break;
			case 2:
				error |= LTC2949_68XX_RdAux(LTC2949_68XX_CMD_RDAUXD, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'd';
				break;
			}
		}
		// for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
		// {
    //   str += cvs[i]; //Serial.print(cvs[i]);
		// 	cvs[i] = "";
		// }
		// str += errorExt; //Serial.print(errorExt);
		// str += ',';//PrintComma();

	circularTempdef(muxSelect);
  tempConvertSort(muxSelect);

  }
  
  checkTempNormalize();
  #ifndef GUI_Enabled
  printAux();
  #endif
  
  auxLogging();

 
}

void printAux(void)
{
  for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    #ifndef seventhSlave
      Serial.println();
      Serial.print("IC : ");
      Serial.print(c_ic+1);

      for (uint8_t i=0; i < 15; i++ )
      {
        Serial.print(" T");
        Serial.print(i+1);
        Serial.print(":");
        Serial.print(cellTemperatures[c_ic][i], 3);
        Serial.print(",");
      }
    #else
      if ( c_ic < 6 )
      {
        Serial.println();
        Serial.print("IC : ");
        Serial.print(c_ic+1);

        for (uint8_t i=0; i < 15; i++ )
        {
          Serial.print(" T");
          Serial.print(i+1);
          Serial.print(":");
          Serial.print(cellTemperatures[c_ic][i], 3);
          Serial.print(",");
        } 
      }

      else if ( c_ic == 6 )
      {
        Serial.println();
        Serial.print("IC : ");
        Serial.print(c_ic+1);

        for (uint8_t i=0; i < 6; i++ )
        {
          Serial.print(" T");
          Serial.print(i+1);
          Serial.print(":");
          Serial.print(cellTemperatures[c_ic][i], 3);
          Serial.print(",");
        }
      }
    #endif  
  }
}

void tempPlausibilityCheck(void)
{
  double ti = millis(),delT = 0.0;
  while( delT < tempTimer )
  {
    clearFlagsT();
    cellTempLoop(0);
    checkTempFlag();

    if ( errorFlag[4] == -1 || errorFlag[5] == -1 || errorFlag[6] == -1 )
    {
      delT = millis() - ti ;
      tempFlag = true;
    }
    else
    {
      tempFlag = false;
      break;
    }
  }
}

void tempConvertSort(bool muxSelect)
{
  #ifndef seventhSlave
    if (muxSelect)
    {
      for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
      {
        for ( uint8_t i=0; i < 8; i++ )
        {
          if ( i < 5 )
          {
            cellTemperatures[c_ic][i+8] = auxHigh[c_ic][i];
          }
          else if ( i > 5 )
          {
            cellTemperatures[c_ic][i+7] = auxHigh[c_ic][i];
          }
        }
      }
    }
    else
    {
      for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
      {
        for ( uint8_t i=0; i < 9; i++ )
        {
          if ( i < 5)
          {
            cellTemperatures[c_ic][i] = auxLow[c_ic][i];
          }
          else if ( i > 5 )
          {
            cellTemperatures[c_ic][i-1] = auxLow[c_ic][i];
          }
        }
      }
    }
  #else
    if (muxSelect)
    {
      for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
      {
        if ( c_ic < 6 )
        {
          for ( uint8_t i=0; i < 8; i++ )
          {
            if ( i < 5 )
            {
              cellTemperatures[c_ic][i+8] = auxHigh[c_ic][i];
            }
            else if ( i > 5 )
            {
              cellTemperatures[c_ic][i+7] = auxHigh[c_ic][i];
            }
          }
        }

        else if ( c_ic == 6 )
        {
          for ( uint8_t i=0; i < 8; i++ )
          {
            if ( i < 5 )
            {
              cellTemperatures[c_ic][i+8] = 30.000;
            }
            else if ( i > 5 )
            {
              cellTemperatures[c_ic][i+7] = 30.000;
            }
          }
        }
      }
    }
    else
    {
      for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
      {
        if ( c_ic < 6 )
        {
          for ( uint8_t i=0; i < 9; i++ )
          {
            if ( i < 5)
            {
              cellTemperatures[c_ic][i] = auxLow[c_ic][i];
            }
            else if ( i > 5 )
            {
              cellTemperatures[c_ic][i-1] = auxLow[c_ic][i];
            }
          }
        }

        else if ( c_ic == 6 )
        {
          for ( uint8_t i=0; i < 9; i++ )
          {
            if ( i < 5)
            {
              cellTemperatures[c_ic][i] = auxLow[c_ic][i];
            }
            else if ( i == 6 )
            {
              cellTemperatures[c_ic][i-1] = auxLow[c_ic][i];
            }
            else if ( i > 6 )
            {
              cellTemperatures[c_ic][i-1] = 30.000;  // dummy values to normalize error checks
            }
          }
        }
      }
    }
  #endif
}

void batLoop(bool slowChannelReady )
{
  if (slowChannelReady)
  {
  #ifndef GUI_Enabled
  Serial.println();
  Serial.print("BATTERY VOLTAGE : ");
  Serial.print( batVoltage );
  Serial.println();
  Serial.print("TS CURRENT : ");
  Serial.print( batCurrPower[0] );
  Serial.println();
  Serial.print("TS POWER : ");
  Serial.print( batCurrPower[1] );
  Serial.println();
  #endif

  CellData += batVoltage;
  CellData += ",";
  CellData += batCurrPower[0];
  CellData += ",";
  CellData += batCurrPower[1];
  CellData += ",";
  
  #ifdef GUI_Enabled
  CellData_GUI += batVoltage;
  CellData_GUI += ",";
  CellData_GUI += batCurrPower[0];
  CellData_GUI += ",";
  CellData_GUI += batCurrPower[1];
  CellData_GUI += ",";

  Serial.println(CellData_GUI);
  CellData_GUI="";
  #endif


  
  }
  #ifdef startLogging
  SDcardLogging();
  #endif
}

void checkError (void)
{
  checkVoltageFlag();
  checkTempFlag();
}

void pull_3V3_high(void)
{
  if(bmsFlag==false)
  {
  digitalWriteFast(BMS_FLT_3V3,HIGH);
  }
}

void clearFlags(void)
{
  bmsFlag = false;
  voltFlag = false;
  tempFlag = false;
  //chargerFlag = false;

  //digitalWriteFast(BMS_FLT_3V3,HIGH);
  
  for ( uint8_t i=0; i < 7; i++ )
  {
    errorFlag[i] = 0;
  }

  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i < 15; i++ )
    {
      voltageErrorLoc[c_ic][i] = 0;
      tempErrorLoc[c_ic][i] = 0;
    }
  }
}

void clearFlagsV(void)
{
  for ( uint8_t i=0; i < 4; i++ )
  {
    errorFlag[i] = 0;
  }

  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i < 15; i++ )
    {
      voltageErrorLoc[c_ic][i] = 0;
    }
  }
}

void clearFlagsT(void)
{
  for ( uint8_t i=4; i < 7; i++ )
  {
    errorFlag[i] = 0;
  }

  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i < 15; i++ )
    {
      tempErrorLoc[c_ic][i] = 0;
    }
  }
}

void transferT( uint16_t * data, uint8_t nic, bool muxSelect , uint8_t auxReg)
{
   if (loopcount)
  {
      uint8_t counter = 0;
    if (muxSelect)
    {
      for (uint8_t i = 3 * nic; i < 3 * (nic + 1); i++) 
      {
        auxHigh1[nic][counter + 3 * auxReg] = data[i] * 100e-6;
        counter++;
      }
   } 
   else 
   {
      for (uint8_t i = 3 * nic; i < 3 * (nic + 1); i++) 
      {
        auxLow1[nic][counter + 3 * auxReg] = data[i] * 100e-6;
        counter++;
      }
    }
  }
  else
  {
      uint8_t counter = 0;
    if (muxSelect) 
    {
      for (uint8_t i = 3 * nic; i < 3 * (nic + 1); i++) 
      {
        auxHigh2[nic][counter + 3 * auxReg] = data[i] * 100e-6;
        counter++;
      }
    } 
    else 
    {
      for (uint8_t i = 3 * nic; i < 3 * (nic + 1); i++) 
      {
        auxLow2[nic][counter + 3 * auxReg] = data[i] * 100e-6;
        counter++;
      }
    }
}
}

void transferV( uint16_t * data, uint8_t nic, uint8_t cellReg)
{
 if (loopcount) {
    uint8_t counter = 0;
    for (uint8_t i = 3 * nic; i < 3 * (nic + 1); i++) {
      cells1[nic][counter + 3 * cellReg] = data[i] * 100e-6;
      counter++;
    }
  } else {
    uint8_t counter = 0;
    for (uint8_t i = 3 * nic; i < 3 * (nic + 1); i++) {
      cells2[nic][counter + 3 * cellReg] = data[i] * 100e-6;
      counter++;
    }
  }
}

void circularVoltdef(void){

  for (int c_ic = 0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++) {
    for (int i = 0; i < 18; i++) {
      if( abs( cells1[c_ic][i]-cells2[LTCDEF_CELL_MONITOR_COUNT- c_ic -1][i]) < tolerance){
        cells[c_ic][i]=cells1[c_ic][i];

      }else{

         if(cells1[c_ic][i] > 0.5 && cells2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i] > 0.5) //add some range
         {
          cells[c_ic][i]=cells1[c_ic][i];
          if(cells[c_ic][i]>cells2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i]){
            cells[c_ic][i]=cells2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i];
          }
        }if(cells1[c_ic][i] < 0.5 && cells1[c_ic][i]>0){
          cells[c_ic][i]=cells2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i];
        }if(cells2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i] < 0.5 && cells2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i] >0){
          cells[c_ic][i]=cells1[c_ic][i];
        }

      }
    }
  }
}

void circularTempdef(bool muxSelect){
  if(loopcount)
  {
    double auxRefH, auxRefL;
    for (uint8_t c_ic = 0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++) 
    {
      if (muxSelect) 
      {
        auxRefH = auxHigh1[c_ic][5];
      } 
      else 
      {
        auxRefL = auxLow1[c_ic][5];
      }
      for (uint8_t i = 0; i < 12; i++){
        if (muxSelect) {
          auxHigh1[c_ic][i] = voltToTemp(auxHigh1[c_ic][i], auxRefH);
        } else {
          auxLow1[c_ic][i] = voltToTemp(auxLow1[c_ic][i], auxRefL);
        }
      }
    }
  } 
  else
  {
      double auxRefH, auxRefL;
    for (uint8_t c_ic = 0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++) {
      if (muxSelect) {
        auxRefH = auxHigh2[c_ic][5];
      } else {
        auxRefL = auxLow2[c_ic][5];
      }
      for (uint8_t i = 0; i < 12; i++) 
      {
        if (muxSelect) 
        {
          auxHigh2[c_ic][i] = voltToTemp(auxHigh2[c_ic][i], auxRefH);
        } 
        else 
        {
          auxLow2[c_ic][i] = voltToTemp(auxLow2[c_ic][i], auxRefL);
        }
      }
    }
  }
  for (uint8_t c_ic = 0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++) {
    for (uint8_t i = 0; i < 12; i++) {
        if (muxSelect) {
          if( abs(auxHigh1[c_ic][i]-auxHigh2[LTCDEF_CELL_MONITOR_COUNT- c_ic -1][i]) < tolerance){
            auxHigh[c_ic][i]=auxHigh1[c_ic][i];
          }else{
            auxHigh[c_ic][i]=auxHigh1[c_ic][i];
          if(auxHigh[c_ic][i]<auxHigh2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i]){
            auxHigh[c_ic][i]=auxHigh2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i];
            }
          } 
        } else 
        {
          if( abs(auxLow1[c_ic][i]-auxLow2[LTCDEF_CELL_MONITOR_COUNT- c_ic -1][i]) < tolerance){
            auxLow[c_ic][i]=auxLow1[c_ic][i];
          }
          else
          {
            auxLow[c_ic][i]=auxLow1[c_ic][i];
          if(auxLow[c_ic][i]<auxLow2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i])
          {
            auxLow[c_ic][i]=auxLow2[LTCDEF_CELL_MONITOR_COUNT-c_ic-1][i];
          }
        }
      }
    }
  }
}

void checkTempFlag(void)
{
  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i < 15; i++ )
    {
      if ( cellTemperatures[c_ic][i] < 0.0 )
      {
        errorFlag[4] = -1;
        tempErrorLoc[c_ic][i] = -1;
      }

      if ( cellTemperatures[c_ic][i] == 0.0 )
      {
        errorFlag[5] = -1;
        tempErrorLoc[c_ic][i] = -1;
      }

      if ( (cellTemperatures[c_ic][i] > overTempThreshold) )
      {
        errorFlag[6] = -1;
        tempErrorLoc[c_ic][i] = -1; 
      }
    }
  }
}
void errorCheckingMode(void)
{ 

  enteredChecking=true;
  if ( errorFlag[0] == -1 || errorFlag[1] == -1 || errorFlag[2] == -1 || errorFlag[3] == -1 )
  {
    voltagePlausibilityCheck();
  }

  if ( errorFlag[4] == -1 || errorFlag[5] == -1 || errorFlag[6] == -1 )
  {
    tempPlausibilityCheck();
  }
}

void triggerError(void)
{
  for ( uint8_t i=0; i<7; i++ )
  {
    if ( errorFlag[i] == -1 )
    { 
      errorCheckingMode();
      break;
    }
  }
}

void triggerInterrupt(void)
{
  bmsFlag = voltFlag || tempFlag || chargerFlag;
  
  if ( bmsFlag || chargerFlag )
  { 
    if ( voltFlag && tempFlag && chargerFlag )
    {
      Serial.print('\n');
      Serial.print(" Interrupted due to voltage,temperature and charger flag ");
      printErrLocV();
      printErrLocT();
      //printchargerError();
    }

    else if ( voltFlag && tempFlag )
    {
      Serial.print('\n');
      Serial.print(" Interrupted due to voltage and temperature flag ");
      printErrLocT();
      printErrLocV();
      
    }

    else if ( voltFlag && chargerFlag )
    {
      Serial.print('\n');
      Serial.print(" Interrupted due to voltage and charger flag ");
      printErrLocV();
      //printchargerError();
    }

    else if ( chargerFlag && tempFlag )
    {
      Serial.print('\n');
      Serial.print(" Interrupted due to temperature and charger flag ");
      printErrLocT();
      //printchargerError();
    }

    else if (  tempFlag )
    {
      Serial.print('\n');
      Serial.print(" Interrupted due to temperature flag only ");
      printErrLocT();
      
    }

    else if ( chargerFlag  )
    {
      Serial.print('\n');
      Serial.print(" Interrupted due to charger flag only ");
      
      printchargerError();
    }

    else if ( voltFlag  )
    {
      Serial.print('\n');
      Serial.print(" Interrupted due to voltage flag only ");
      
      printErrLocV();
    }

    #ifdef Interrupt_Debug
    Interrupt_for_Debug();
    #endif

    #ifndef Interrupt_Debug
    Interrupt();
    #endif
  }
}

void printErrLocV(void)
{
  for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    Serial.println();
    Serial.print("IC : ");
    Serial.print(c_ic+1);

    for (uint8_t i=0; i < 15; i++ )
    {
      Serial.print(" C");
      Serial.print(i+1);
      Serial.print(":");
      Serial.print(voltageErrorLoc[c_ic][i]);
      Serial.print(",");
    }
  }
}

void printErrLocT(void)
{
  for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    Serial.println();
    Serial.print("IC : ");
    Serial.print(c_ic+1);

    for (uint8_t i=0; i < 15; i++ )
    {
      Serial.print(" T");
      Serial.print(i+1);
      Serial.print(":");
      Serial.print(tempErrorLoc[c_ic][i]);
      Serial.print(",");
    }
  }
}

void switchErrorLed(void)
{
  for ( uint8_t flags = 0; flags < 3; flags++ )
  {
    switch(flags)
    {
      case 0:
        if( voltFlag )
        {
          digitalWriteFast(VOLT_ERR_PIN,HIGH);
        }
        else
        {
          digitalWriteFast(VOLT_ERR_PIN,LOW);
        }
        break;
      case 1:
        if ( tempFlag )
        {
          digitalWriteFast(TEMP_ERR_PIN,HIGH);
        }
        else
        {
          digitalWriteFast(TEMP_ERR_PIN,LOW);
        }
        break;
      case 2:
        if ( voltFlag && tempFlag )
        {
          digitalWriteFast(VOLT_ERR_PIN,HIGH);
          digitalWriteFast(TEMP_ERR_PIN,HIGH);
        }
        break;
    }
  }
}

void initialiseNormalizeChannel(void)
{
  for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for (uint8_t i=0; i < 15; i++ )
    {
      voltageNormalize[c_ic][i] = 0;
      tempNormalize[c_ic][i] = 0;
    }
  }
}

void setNormalizeFlag( uint8_t nic, uint16_t voltChannels, uint16_t tempChannels )
{
  for ( int i=0; i < 15; i++ )
  {
    if( (voltChannels >> i) & 1 )
    {
      voltageNormalize[nic][i] = -1;
    }
  }

  for ( int i=0; i < 15; i++ )
  {
    if( (tempChannels >> i) & 1 )
    {
      tempNormalize[nic][i] = -1;
    }
  }
}

void checkVoltNormalize(void)
{
  for (int c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( int i=0; i<15; i++ )
    {
      if ( voltageNormalize[c_ic][i] == -1 )
      {
        cellVoltages[c_ic][i] = 3.5000;
      }
    }
  }
}

void checkTempNormalize(void)
{
  for (int c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( int i=0; i<15; i++ )
    {
      if ( tempNormalize[c_ic][i] == -1 )
      {
        cellTemperatures[c_ic][i] = 30.000;
      }
    }
  }
}

void initialiseSplitChannel(void)
{
  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i<15; i++ )
    {
      voltageSplit[c_ic][i] = 0;
    }
  }
}

void setVoltSplitChannels(uint8_t ic, uint16_t * channels )
{
  for ( int c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    if ( (ic >> c_ic) & 1 )
    {
      setVoltSplitFlag(c_ic, channels[c_ic]);
    }
  }
}

void setVoltSplitFlag( uint8_t nic, uint16_t channels )
{
  for ( int i=0; i < 15; i++ )
  {
    if( (channels >> i) & 1 )              
    {
      voltageSplit[nic][i] = -1;
    }
  }
}

void checkVoltSplit(void)
{
  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i<15; i++ )
    {
      if( voltageSplit[c_ic][i] == -1 )      // set split flag for right channel among the two channels of a split
      {
        float splitVal = avg(cellVoltages[c_ic][i-1],cellVoltages[c_ic][i]);
        cellVoltages[c_ic][i-1] = splitVal;
        cellVoltages[c_ic][i] = splitVal;
      }
    }
  }
}

void checkACUsignalStatus(void)
{
  if( amsMasterToACUsignals[0] )
  {
    digitalWriteFast(FAN_MBED, HIGH);
  }
  if( amsMasterToACUsignals[1] )
  {
    digitalWriteFast(CH_EN_MBED, HIGH);
  }
}

void findMax(double array[LTCDEF_CELL_MONITOR_COUNT][15], uint8_t type)
{
  double max = array[0][0];
  uint8_t sLoc=0, cLoc=0;
   if ( type == voltage ){
  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i<cellsPerStack[c_ic]; i++ )
    {
      if(voltageNormalize[c_ic][i]!=-1){
      if ( array[c_ic][i] > max )
      {
        max = array[c_ic][i];
        sLoc = c_ic;
        cLoc = i;
      }
    }
  }}

  
    maxVoltage.val = max;
    maxVoltage.slaveLoc = sLoc+1;
    maxVoltage.cellLoc = cLoc+1;
  }
  if ( type == temp )
  {for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i<cellsPerStack[c_ic]; i++ )
    {
      if(tempNormalize[c_ic][i]!=-1){
      if ( array[c_ic][i] > max )
      {
        max = array[c_ic][i];
        sLoc = c_ic;
        cLoc = i;
      }
    }
  }}

    maxTemp.val = max;
    maxTemp.slaveLoc = sLoc+1;
    maxTemp.cellLoc = cLoc+1;
  }
}

void findMin(double array[LTCDEF_CELL_MONITOR_COUNT][15])
{
  double min = array[0][0];
  uint8_t sLoc=0, cLoc=0;
  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i<cellsPerStack[c_ic]; i++ )
    {if(voltageNormalize[c_ic][i]!=-1){
      if ( array[c_ic][i] < min )
      {
        min = array[c_ic][i];
        sLoc = c_ic;
        cLoc = i;
      }
    }}
  }

  minVoltage.val = min;
  minVoltage.slaveLoc = sLoc+1;
  minVoltage.cellLoc = cLoc+1;

}

void printMaxMinParameters(void)
{
  Serial.println();

  Serial.print(" Max Voltage : ");
  Serial.print(maxVoltage.val , 4);
  Serial.print("  Location --> ");
  Serial.print("S");
  Serial.print(maxVoltage.slaveLoc);
  Serial.print("C");
  Serial.print(maxVoltage.cellLoc);
  
  Serial.println();
  
  Serial.print(" Max Temperature : ");
  Serial.print(maxTemp.val , 4);
  Serial.print("  Location --> ");
  Serial.print("S");
  Serial.print(maxTemp.slaveLoc);
  Serial.print("C");
  Serial.print(maxTemp.cellLoc);

  Serial.println();
  
  Serial.print(" Min Voltage : ");
  Serial.print(minVoltage.val , 4);
  Serial.print("  Location --> ");
  Serial.print("S");
  Serial.print(minVoltage.slaveLoc);
  Serial.print("C");
  Serial.print(minVoltage.cellLoc);
}

void performDynamicCooling(void)
{

if (flag_fan == 0) {
    if (maxTemp.val <= 35)
        digitalWriteFast(FAN_MBED, LOW);
    else {
        digitalWriteFast(FAN_MBED, HIGH);
        flag_fan = 1;
    }
}
else {
    if (maxTemp.val > 33)
        digitalWriteFast(FAN_MBED, HIGH);
    else {
        digitalWriteFast(FAN_MBED, LOW);
        flag_fan = 0;
    }
}

}

void chargerLoop(void)
{
     send_msg.flags.extended = 1;
     receive_msg.flags.extended = 1;
     send_msg.id=send_id;
    if(chargerFlag == false)
    {
    send_msg.buf[0]= (max_voltage)>>8;
    send_msg.buf[1]= (max_voltage)%256;
    send_msg.buf[2]= (max_current_without_decimal)>>8;
    send_msg.buf[3]= (max_current_without_decimal)%256;
    send_msg.buf[4]= 0;
    Serial.println("Set control bit to 0: Charger OK");
    }
    else{
    send_msg.buf[0]= (max_voltage)>>8;
    send_msg.buf[1]= (max_voltage)%256;
    send_msg.buf[2]= (max_current_without_decimal)>>8;
    send_msg.buf[3]= (max_current_without_decimal)%256;
    send_msg.buf[4]= 1;
    Serial.println("Set control bit to 1: Do not charge");
    }

    output_voltage = (receive_msg.buf[0]*256 + receive_msg.buf[1] + 1.5)/10;
    output_current = (receive_msg.buf[2]*256 + receive_msg.buf[3])/10;
    //need to add the fifth control byte also
    //Serial.println(send_msg.buf[2]);
    //Serial.println(send_msg.buf[3]);
  #ifndef GUI_Enabled
  Serial.println("Entered Charger loop");
  #endif
  // #ifdef charger_softEnabled

  //  if ((max_voltage<100))
  //  {
  //if ((can3.write(send_msg)!=1))

  //{
  // double ti = millis(),delT = 0.0;
  // char input='0';
  // #ifndef GUI_Enabled
  // while( delT < chargeTimer )
  // {
    
  //   delT = millis() - ti ;
  //   if (input=='c')
  //   { 
  //     send_msg.flags.extended = 1;
  //    receive_msg.flags.extended = 1;
  //    send_msg.id=send_id;
 
  //     max_voltage=3950;
  //     max_current_without_decimal=100;
  //     send_msg.buf[0]= (max_voltage)>>8;
  //   send_msg.buf[1]= (max_voltage)%256;
  //   send_msg.buf[2]= (max_current_without_decimal)>>8;
  //   send_msg.buf[3]= (max_current_without_decimal)%256;

  //     Serial.println("Charger Enable Received");
  //     can3.write(send_msg);
  //     Serial.println(can3.write(send_msg));
  //     break;

  //   }
  //   else
  //   { max_voltage=0;
  //     max_current_without_decimal=0;
  //     send_msg.buf[0]= (max_voltage)>>8;
  //   send_msg.buf[1]= (max_voltage)%256;
  //   send_msg.buf[2]= (max_current_without_decimal)>>8;
  //   send_msg.buf[3]= (max_current_without_decimal)%256;
  //     Serial.println("BMS OK....Waiting for charger Enable");
  //     can3.write(send_msg);
  //     Serial.println(can3.write(send_msg));
  //   }
  //   if (Serial.available() > 0)
  //   {
  //     input = read_char();
  //   } 
  //   delay(100); 
  // }
  // #endif
  // #ifdef GUI_Enabled
  // while( delT < chargeTimer )
  // {
    
  //   delT = millis() - ti ;
  //   if (input=='c')
  //   {
  //     send_msg.flags.extended = 1;
  //    receive_msg.flags.extended = 1;
  //    send_msg.id=send_id;
 
  //     max_voltage=3950;
  //     max_current_without_decimal=100;
  //     send_msg.buf[0]= (max_voltage)>>8;
  //   send_msg.buf[1]= (max_voltage)%256;
  //   send_msg.buf[2]= (max_current_without_decimal)>>8;
  //   send_msg.buf[3]= (max_current_without_decimal)%256;

      
  //     can3.write(send_msg);
      
  //     break;

  //   }
  //   else
  //   {  max_voltage=0;
  //     max_current_without_decimal=0;
  //     send_msg.buf[0]= (max_voltage)>>8;
  //   send_msg.buf[1]= (max_voltage)%256;
  //   send_msg.buf[2]= (max_current_without_decimal)>>8;
  //   send_msg.buf[3]= (max_current_without_decimal)%256;
  //   can3.write(send_msg);

  //     Serial.println("0,0,0,0,0,0,0,0,0,");
      
  //   }
  //   if (Serial.available() > 0)
  //   {
  //     input = read_char();
  //   } 
  //   delay(100); 
  // }
  //     Serial.println("1,1,1,1,1,1,1,1,1,1,1,");
  //     Serial.println("1,1,1,1,1,1,1,1,1,1,1,");
  //     Serial.println("1,1,1,1,1,1,1,1,1,1,1,");
  // #endif
  
  
  
  
  // }
  // #endif
  //can3.write(send_msg);
  #ifndef GUI_Enabled
  //if(chargerFlag == false){
    if(can3.write(send_msg) == 1){
      Serial.println("CAN Message sent successfully");
    }
    else{
      Serial.println("Failed to send CAN message");
     //Interrupt();
    }
  //}
  #endif
  #ifdef GUI_Enabled
  
    output_voltage = (receive_msg.buf[0]*256 + receive_msg.buf[1] + 1.5)/10;
    output_current = (receive_msg.buf[2]*256 + receive_msg.buf[3])/10;
    Serial.print(output_voltage);
    Serial.print(",");
    Serial.print(output_current);
    Serial.println(",");
    for (int i = 4; i >= 0; i--) {
            int bit = (receive_msg.buf[4] >> i) & 1;
            Serial.print(bit);
            Serial.print(",");
        }

    Serial.println("");
  #endif

 
  //Serial.println(msg.id);
  #ifndef GUI_Enabled
  if(can3.read(receive_msg)){
    //Serial.print("ID: 0x"); Serial.print(msg.id, HEX );
    //Serial.println(receive_msg.id, HEX);
    //Serial.print("LEN: "); Serial.println(receive_msg.len);
    Serial.println("CAN communication with charger successful");
    output_voltage = (receive_msg.buf[0]*256 + receive_msg.buf[1] + 1.5)/10;
    output_current = (receive_msg.buf[2]*256 + receive_msg.buf[3])/10;
    //Serial.print("Output Voltage [CHARGER] : "); Serial.println(output_voltage);
    //Serial.print("Output Current [CHARGER] : "); Serial.println(output_current);
    //Serial.print("Error Byte :");
    //Serial.println(receive_msg.buf[4]);
    }
    else{
      Serial.println("Received no message from charger");
      chargerFlag = true;
    }
  #endif
}

void triggerchargerError(void)
{
  output_voltage = (receive_msg.buf[0]*256 + receive_msg.buf[1])/10;
  output_current = (receive_msg.buf[2]*256 + receive_msg.buf[3])/10;
 
  if(output_voltage>250 && chargingStarted_)
  {
    chargingStarted_=false;
    chargingFlag=true;
    delay(chgrDelay);
    if(can3.read(receive_msg)){
    //Serial.print("ID: 0x"); Serial.print(msg.id, HEX );
    Serial.println(receive_msg.id, HEX);
    Serial.print("LEN: "); Serial.println(receive_msg.len);
    output_voltage = (receive_msg.buf[0]*256 + receive_msg.buf[1] + 1.5)/10;
    output_current = (receive_msg.buf[2]*256 + receive_msg.buf[3])/10;
    Serial.println("Attempting to start charging");
    Serial.print("Output Voltage [CHARGER] : "); Serial.println(output_voltage);
    Serial.print("Output Current [CHARGER] : "); Serial.println(output_current);
    Serial.print("Error Byte :");
    Serial.println(receive_msg.buf[4]);}
    else{
      Serial.println("Received no message from charger(2)");
      chargerFlag = true;
    }
  }
  if(output_voltage < 250 && (receive_msg.buf[4] == 8 || receive_msg.buf[4] == 24) && output_current < 1)
  {
   chargerFlag=false;
   #ifndef GUI_Enabled
   Serial.println("Connect battery");
   printchargerError();
   #endif
  }
  else if(receive_msg.buf[4] != 0){
    chargerFlag=true;
    #ifndef GUI_Enabled
    Serial.println("Received some error from charger check error byte");
    printchargerError();
    #endif
  }
  else if (output_voltage > 250 && receive_msg.buf[4] == 0 && output_current < 0.1)
  {
    chargerFlag= true;
    // if(chargingFlag==true)
    // {
    // // send_msg.flags.extended = 1;
    // //  receive_msg.flags.extended = 1;
    // //  send_msg.id=send_id;
    // //  max_voltage = 0;
    // // send_msg.buf[0]= (max_voltage)>>8;
    // // send_msg.buf[1]= (max_voltage)%256;
    // // send_msg.buf[2]= (max_current_without_decimal)>>8;
    // // send_msg.buf[3]= (max_current_without_decimal)%256;
    // // can3.write(send_msg);
    // }
    #ifndef GUI_Enabled
    Serial.println("Charger is ready | Voltage detected by charger | No current detected");
    printchargerError();
    #endif
  }
  else if (output_voltage > 250 && receive_msg.buf[4] == 0 && output_current > 1)
  {
    chargerFlag=false;
    #ifndef GUI_Enabled
    Serial.println("Charging ... Charging ... Charging ...");
    #endif
  }
  else if (output_voltage > 250 && receive_msg.buf[4] != 0)
  {
    chargerFlag=true;
    #ifndef GUI_Enabled
    Serial.println("Charging stopped due to charger error");
    printchargerError();
    #endif
  }
  else {
    chargerFlag=true;
    #ifndef GUI_Enabled
    Serial.println("Entered confusing state");
    printchargerError();
    #endif
  }


}

void printchargerError(void)
{
  Serial.println("");
  Serial.print("Charger Error Byte:");
  Serial.println(receive_msg.buf[4]);
      if (receive_msg.buf[4] == 1)
      Serial.println("Status: Hardware Failure"); 
      if (receive_msg.buf[4] == 2)
      Serial.println("Status: Charger overheated"); 
      if (receive_msg.buf[4] == 4)
      Serial.println("Status: Incorrect Input Voltage"); 
      if (receive_msg.buf[4] == 8)
      Serial.println("Status: Battery Disconnected"); 
      if (receive_msg.buf[4] == 16)
      Serial.println("Status: Communication failed");
      if (receive_msg.buf[4] == 12)
      Serial.println("Status: Battery disconnected and incorrect input voltage");
      if (receive_msg.buf[4] == 24)
      Serial.println("Status: Battery disconnected and communication failed");
}

void cellsLogging(void)
{
  
  if(BPM_ready)
  {
    for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
    {
      #ifndef seventhSlave
        for (uint8_t i=0; i < 15; i++ )
        {
          CellData += String(cellVoltages[c_ic][i], 4);
          CellData += ",";
        }
      #else
        if ( c_ic < 6 )
        {
          for (uint8_t i=0; i < 15; i++ )
          {
            CellData += String(cellVoltages[c_ic][i], 4);
            CellData += ",";
          }
        }

        else if ( c_ic == 6 )
        {
          for (uint8_t i=0; i < 6; i++ )
          {
            CellData += String(cellVoltages[c_ic][i], 4);
            CellData += ",";
          }
        }
      #endif
    }
  }
  #ifdef LTCDEF_LTC681X_ONLY
  {
    for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
    {
      #ifndef seventhSlave
        for (uint8_t i=0; i < 15; i++ )
        {
          CellData += String(cellVoltages[c_ic][i], 4);
          CellData += ",";
        }
      #else
        if ( c_ic < 6 )
        {
          for (uint8_t i=0; i < 15; i++ )
          {
            CellData += String(cellVoltages[c_ic][i], 4);
            CellData += ",";
          }
        }

        else if ( c_ic == 6 )
        {
          for (uint8_t i=0; i < 6; i++ )
          {
            CellData += String(cellVoltages[c_ic][i], 4);
            CellData += ",";
          }
        }
      #endif
    }
  }
  #endif
    for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
    {
      #ifndef seventhSlave
        for (uint8_t i=0; i < 15; i++ )
        {
          
          CellData_GUI += String(cellVoltages[c_ic][i], 4);
          CellData_GUI += ",";
        }
      #else
        if ( c_ic < 6 )
        {
          for (uint8_t i=0; i < 15; i++ )
          {
            CellData_GUI += String(cellVoltages[c_ic][i], 4);
            CellData_GUI += ",";
            
          }
        }

        else if ( c_ic == 6 )
        {
          for (uint8_t i=0; i < 6; i++ )
          {
            
            CellData_GUI += String(cellVoltages[c_ic][i], 4);
            CellData_GUI += ",";
          }
        }
      #endif
    }
     #ifdef GUI_Enabled
     Serial.println(CellData_GUI);
      // Serial.println();
     CellData_GUI = "";
     #endif

  
}

void auxLogging(void)
{
  if(BPM_ready)
  {
    for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
    {
      #ifndef seventhSlave
        for (uint8_t i=0; i < 15; i++ )
        {
          CellData += String(cellTemperatures[c_ic][i], 4);
          CellData += ",";
        }
      #else
        if ( c_ic < 6 )
        {
          for (uint8_t i=0; i < 15; i++ )
          {
            CellData += String(cellTemperatures[c_ic][i], 4);
            CellData += ",";
          }
        }

        else if ( c_ic == 6 )
        {
          for (uint8_t i=0; i < 6; i++ )
          {
            CellData += String(cellTemperatures[c_ic][i], 4);
            CellData += ",";
          }
        }
      #endif
    }
  }
  #ifdef LTCDEF_LTC681X_ONLY
  for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
    {
      #ifndef seventhSlave
        for (uint8_t i=0; i < 15; i++ )
        {
          CellData += String(cellTemperatures[c_ic][i], 4);
          CellData += ",";
        }
      #else
        if ( c_ic < 6 )
        {
          for (uint8_t i=0; i < 15; i++ )
          {
            CellData += String(cellTemperatures[c_ic][i], 4);
            CellData += ",";
          }
        }

        else if ( c_ic == 6 )
        {
          for (uint8_t i=0; i < 6; i++ )
          {
            CellData += String(cellTemperatures[c_ic][i], 4);
            CellData += ",";
          }
        }
      #endif
    }
    #endif

  
  
    for (uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
    {
      #ifndef seventhSlave
        for (uint8_t i=0; i < 15; i++ )
        {
          
          CellData_GUI += String(cellTemperatures[c_ic][i], 4);
          CellData_GUI += ",";
        }
      #else
        if ( c_ic < 6 )
        {
          for (uint8_t i=0; i < 15; i++ )
          {
            
            CellData_GUI += String(cellTemperatures[c_ic][i], 4);
            CellData_GUI += ",";
            
          }
        }

        else if ( c_ic == 6 )
        {
          for (uint8_t i=0; i < 6; i++ )
          {
            
            CellData_GUI += String(cellTemperatures[c_ic][i], 4);
            CellData_GUI += ",";
          }
        }
      #endif
    }
    #ifdef GUI_Enabled
    CellData_GUI += "0,0,0,";
    Serial.println(CellData_GUI);
    CellData_GUI="";
    #endif
  
}

void SDcardLogging(void)
{
  dataFile.println(CellData);
  // Serial.println();
  // Serial.println(CellData);
  // Serial.println();
  dataFile.flush();

  CellData = "";
}

void initialiseSDcard(void)
{
  #ifdef initialiseEEPROM
  EEPROM.write(0,0);
  #endif
  
  int eeprom_value = EEPROM.read(0);
  eeprom_value++;
  Serial.println(eeprom_value);
  EEPROM.write(0,eeprom_value);
  #ifndef GUI_Enabled
  
   Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) 
  {
     Serial.println("Card failed, or not present");
    // don't do anything more:
    // while (1) ;
  }
  // Serial.println("card initialized.");
  #endif


  String fileName = "AMSCellData" + String(eeprom_value) + ".txt";
  // Open up the file we're going to log to!
  dataFile = SD.open(fileName.c_str(), FILE_WRITE);
  #ifndef GUI_Enabled
  if (! dataFile) 
  {
    Serial.println("error opening file");
    // Wait forever since we cant write data
    // while (1) ;
  }
  #endif
}

float InitialiseEnergy(float min_voltage, float thr_voltage) {
    float cap_mapped = 0.0;
    float cap_mapped_thr = 0.0;
    float SOC_VoltApprox = -1;
    float SOC_ThrVoltApprox = -1;

    float cell_capacity_lookup[87] = 
    {
        10.0, 78.54084795, 121.7597832, 171.4230513, 225.0389946, 280.0221137, 
        338.1715796, 387.3510778, 441.212951, 492.8865338, 548.4655855, 
        602.2419923, 656.0061216, 717.2497924, 770.9822287, 824.5884643, 
        879.4615621, 934.3346599, 992.9889292, 1046.572513, 1101.468263, 
        1156.364012, 1209.934652, 1268.689235, 1323.633523, 1375.96966, 
        1427.093946, 1483.288917, 1540.724863, 1598.141393, 1654.412408, 
        1707.971722, 1761.587666, 1813.946454, 1866.350546, 1919.943838, 
        1973.569489, 2025.952547, 2080.820791, 2131.893302, 2184.274742, 
        2237.890685, 2295.284564, 2348.939338, 2403.88039, 2454.744185, 
        2509.961908, 2563.500189, 2618.457421, 2667.135351, 2724.483927, 
        2782.746003, 2829.330942, 2885.525912, 2946.781861, 3008.07664, 
        3061.731414, 3117.887554, 3171.5488, 3230.199833, 3283.796361, 
        3337.392889, 3391.008832, 3443.377329, 3498.318381, 3551.934324, 
        3613.180565, 3666.767385, 3716.474338, 3766.009787, 3813.381701, 
        3852.360288, 3900.213706, 3944.288866, 3988.391207, 4022.84064, 
        4056.234516, 4100.31194, 4142.094775, 4164.440414, 4190.76138, 
        4213.12967, 4222.824484, 4238.782419, 4254.672399, 4260.459841, 
        4260.154046
    };
    float cell_voltage_lookup[87] = {
        
        4.20, 4.142899317, 4.114316039, 4.090057769, 4.076679267, 4.07170303, 
        4.051197819, 4.05428974, 4.064581985, 4.0490339, 4.03982197, 
        4.04188808, 4.042772486, 4.024432319, 4.022266276, 4.007953401, 
        3.992387619, 3.976821837, 3.95562859, 3.939135515, 3.925749933, 
        3.912364352, 3.894625448, 3.883087374, 3.87437365, 3.861002226, 
        3.854178482, 3.842654565, 3.827386083, 3.810248858, 3.806044185, 
        3.787215181, 3.773836678, 3.762645454, 3.755814631, 3.740255928, 
        3.727811797, 3.718956502, 3.702923534, 3.691116474, 3.682105451, 
        3.668726948, 3.649409522, 3.639768506, 3.630743325, 3.623484956, 
        3.616451686, 3.59559821, 3.588130316, 3.592221731, 3.568543904, 
        3.558877405, 3.558619745, 3.547095828, 3.529937367, 3.516516393, 
        3.506875376, 3.491613973, 3.482595871, 3.461091166, 3.44584392, 
        3.430596674, 3.417218171, 3.406961319, 3.397936138, 3.384557635, 
        3.366464802, 3.350283185, 3.330229587, 3.293668758, 3.270606767, 
        3.228092858, 3.202100338, 3.157377913, 3.11527173, 3.067986154, 
        3.017653961, 2.948511876, 2.904892293, 2.838268764, 2.784705202, 
        2.720261874, 2.667880434, 2.602382398, 2.53034376, 2.471442955, 
        2.442010248
    };

    int SOC_lookup_index=0;
    do{
      SOC_lookup_index++;
      SOC_VoltApprox = min_voltage - cell_voltage_lookup[SOC_lookup_index];
    }
    while(SOC_VoltApprox<0 || SOC_lookup_index>86);
  
    cap_mapped = linear_interpolate(cell_voltage_lookup[SOC_lookup_index],cell_capacity_lookup[SOC_lookup_index] , cell_voltage_lookup[SOC_lookup_index-1], cell_capacity_lookup[SOC_lookup_index-1], min_voltage);

    SOC_lookup_index=0;
    do{
      SOC_lookup_index++;
      SOC_ThrVoltApprox = thr_voltage - cell_voltage_lookup[SOC_lookup_index];
    }
    while(SOC_ThrVoltApprox<0 || SOC_lookup_index>86);

    cap_mapped_thr = linear_interpolate(cell_voltage_lookup[SOC_lookup_index],cell_capacity_lookup[SOC_lookup_index] , cell_voltage_lookup[SOC_lookup_index-1], cell_capacity_lookup[SOC_lookup_index-1], thr_voltage);


    // Calculate Energy Available
    float Energy_available = (cap_mapped_thr - cap_mapped) * 5.5 * min_voltage * 1.05 * 90 / 1000000.0;
  return Energy_available;
}

float CalculateEnergy(float TS_voltage, float TS_current, float EnergyAvailable, float time_previous, float chargerDelay) {
    
    float Power = abs(TS_current * TS_voltage / 1000.0);
    // If TS_current is positive, return Energy_available and Power as 0
    #ifdef charger_active
    if (TS_current < 1) {
        Power = 0.0;
        return;
    }
    #else
    if (TS_current > -1) {
        Power = 0.0;
        return;
    }
    #endif
    // Calculate Power and make it positive
    // Power in KW
    #ifndef charger_active
      chargerDelay=0;
    #endif
    // Calculate Energy Consumed
    float Energy_consumed = (Power * (time_previous + chargerDelay)) / (1000.0 * 3600.0); // Energy consumed in kWh

    // Update Energy Available
    EnergyAvailable += Energy_consumed;
    return EnergyAvailable;

}

float linear_interpolate(float x0, float y0, float x1, float y1, float x) {
    float y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    return y;
}

void initialiseFlags(void)
{
  bmsFlag = false;
  voltFlag = false;
  tempFlag = false;
  chargerFlag = false;
}

void initialiseCAN(void)
{
  can3.begin();
  can3.setBaudRate(250000);
//  can.enableFIFO(); //enabling FIFO
//  can.enableFIFOInterrupt(); //all FIFO interrupt enabled
//  can.onReceive(read_update);
}

String CellVoltagesToString(uint16_t * cellMonDat, uint8_t nic)
{
	String str = "";
	uint8_t i = nic * 3;
	uint8_t lasti = i + 3;
	for (; i < lasti; i++)
	{
		if (cellMonDat[i] != 0xFFFFU)
			str += String(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
      //Serial.println(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
		str += ',';
	}
  // Serial.println("CellData");
  // Serial.println(cellMonDat[0] * 100e-6, LTCDEF_DIGITS_CELL);

	return str;
}

void PrintAuxVoltages(uint16_t * cellMonDat, boolean init)
{
	for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT * 3; i++)
	{
		if (cellMonDat[i] != 0xFFFFU) // don't print non-existing cells
			//Serial.print(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
		if (init)
			cellMonDat[i] = 0xFFFFU; // this allows to check if next cell voltages were read correctly
		//PrintComma();
	}
}
void PrintCellVoltages(uint16_t * cellMonDat, boolean init)
{
	for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT * 3; i++)
	{
    if (cellMonDat[i] != 0xFFFFU) // don't print non-existing cells
			//Serial.print(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
		if (init)
			cellMonDat[i] = 0xFFFFU; // this allows to check if next cell voltages were read correctly
		//PrintComma();
	}
}

void Init(uint8_t selCS, boolean ltc2949onTopOfDaisychain)
{
#ifdef LTCDEF_LTC681X_ONLY
	ltc2949onTopOfDaisychain = true;
#endif

	LTC2949_CS = selCS;
	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,			*/LTCDEF_CELL_MONITOR_COUNT,
		/*boolean ltc2949onTopOfDaisychain, */ltc2949onTopOfDaisychain,
		/*boolean debugEnable				*/false
	);
	LTC2949_init_device_state();
	// 
	Serial.print(F("INIT,"));
#ifndef LTCDEF_LTC681X_ONLY
	if (LTC2949_onTopOfDaisychain)
		Serial.print(F("ON TOP OF"));
	else
		Serial.print(F("PARALLEL TO"));
	Serial.print(F(" DAISYCHAIN,"));
#endif

	Serial.print(F("CS:"));
	Serial.print(LTC2949_CS);
	PrintComma();
	//
	delay(LTC2949_TIMING_BOOTUP);
	byte error = 0;
#ifdef LTCDEF_LTC681X_ONLY
	mcuTime = millis();
	error |= CellMonitorInit();
#else
if(loopcount){
	error = WakeUpReportStatus();
  if(error)
  {
    digitalWriteFast(BMS_FLT_3V3,LOW);
  }
	error |= Cont(false);
	mcuTime = millis();
	delay(LTC2949_TIMING_CONT_CYCLE);
  //Serial.println("code entering CellMonitorInit");
	error |= CellMonitorInit();
	error |= Cont(true);
}
#endif
  //PrintComma();
	//PrintOkErr(error);
	//PrintCSVHeader();
}

void checkVoltageFlag(void)
{
  for ( uint8_t c_ic=0; c_ic < LTCDEF_CELL_MONITOR_COUNT; c_ic++ )
  {
    for ( uint8_t i=0; i < 15; i++ )
    {
      if ( cellVoltages[c_ic][i] < 0.5 )
      {
        errorFlag[0] = -1;
        voltageErrorLoc[c_ic][i] = -1;
      }

      if ( cellVoltages[c_ic][i] > 6.0 )
      {
        errorFlag[1] = -1;
        voltageErrorLoc[c_ic][i] = -1;
      }

      if ( (cellVoltages[c_ic][i] < underVoltageThreshold) && (cellVoltages[c_ic][i] > 0.5 ) )
      {
        errorFlag[2] = -1;
        voltageErrorLoc[c_ic][i] = -1;
      }

      if ( (cellVoltages[c_ic][i] > overVoltageThreshold) && (cellVoltages[c_ic][i] < 6.0 ) )
      {
        errorFlag[3] = -1;
        voltageErrorLoc[c_ic][i] = -1; 
      }
    }
  }
}

byte CellMonitorCFGB(byte * cellMonDat, bool verbose, bool muxSelect)
{
	// read configuration and print
	byte error = LTC2949_68XX_RdCfgb(cellMonDat);
	if (verbose)
	{
		//SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
		//PrintComma();
	}
	// set GPIO[6-9] in configuration registers
  uint8_t k = muxSelect ? 4 : 3;
	for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
	{
		cellMonDat[i * 6 + 0] = 0;
    for ( uint8_t j = 0; j < k; j++ )
    {
      cellMonDat[i * 6 + 0] |= (1 << j) ; // set GPIO[6-9] as 1
    }

    cellMonDat[i * 6 + 1] = 0;
    cellMonDat[i * 6 + 2] = 0;
    cellMonDat[i * 6 + 3] = 0;
		cellMonDat[i * 6 + 4] = 0; 
		cellMonDat[i * 6 + 5] = 0; 
	}
	// write configuration registers
	error |= LTC2949_68XX_WrCfgb(cellMonDat);
	return error;
}

double voltToTemp( double auxVal, double auxRef)
{
  if ( auxVal != auxRef )
  {
  double R = 10000.0;
  double Rt;
  double B = 3420.50726961, C = B/293.15;   // coefficients are calculated using datasheet 103JT thermistor
  
  Rt = ( R * auxVal )/( auxRef - auxVal );
  Rt =  Rt * 0.001 ;
  
  if( Rt > 0 )
  {
    double logR = log(Rt/12.11),T;
    T = B/(logR + C);
    T = T - 273.15;
    return T;
  }
  else
  {
    return 0.0;
  }

  }
  else
  {
    return 0.0;
  }
}


void Interrupt(void)
{
  digitalWriteFast( BMS_FLT_3V3 , LOW );               
  switchErrorLed();
}

float avg( float a, float b)
{
  return (a+b)/2;
}

byte WakeUpReportStatus()
{
	byte  error = LTC2949_WakeupAndAck();
	error |= LTC2949_ReadChkStatusFaults(true, true);
	PrintComma();
	PrintOkErr(error);
	return error;
}

byte Cont(boolean enable)
{
	if (enable)
	{
		byte error = 0;
#ifdef LTCDEF_READ_FROM_EEPROM
		error = LTC2949_EEPROMRead();
#else
		// fast slot not used, still we configure something
		LTC2949_SlotFastCfg(3, 2);
		// SLOT1 measures temperature via NTC between V1 and GND. SLOT2 not used, still we configure something
		LTC2949_SlotsCfg(1, 0, 4, 5);
		// enable NTC temperature measurement via SLOT1
		NtcCfgWrite(1, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);
#endif

#ifdef LTCDEF_WRITE_TO_EEPROM
		error = LTC2949_EEPROMWrite();
#endif

		// read & clear status
		error |= LTC2949_ReadChkStatusFaults(true, false);

		// enable measurement
		return error | LTC2949_GoCont(
			/*cfgFast:      */ LTCDEF_FACTRL_CONFIG,
			/*byte adcCfg:  */ LTCDEF_ADCCFG_CONFIG);
	}
	LTC2949_WriteFastCfg(0);
	LTC2949_OpctlIdle();
	return 0;
}

byte CellMonitorInit()
{
	byte cellMonDat[LTCDEF_CELL_MONITOR_COUNT * 6];
	LTC2949_68XX_RdCfg(cellMonDat); // dummy read
	// dummy read of cell voltage group A
	byte error = ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
	// clear all cell voltage groups
	error |= LTC2949_68XX_ClrCells();
  error |= LTC2949_68XX_ClrAux();

	error |= CellMonitorCFGA(cellMonDat, true);

	// // read configuration and print
	error |= LTC2949_68XX_RdCfg(cellMonDat);
  // for ( uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++ )
  // {
  //   Serial.println();
  //   for (uint8_t j = 0; j < 6; j++ )
  //   {
  //     Serial.println(cellMonDat[i*6+j],BIN);
  //   }
  // }
	//SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
	//PrintComma();
  
  error |= CellMonitorCFGB(cellMonDat, true, false);
  error |= LTC2949_68XX_RdCfgb(cellMonDat);
	//SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
	//PrintComma();
  
	// error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
  // error |= ReadPrintAuxVoltages(LTC2949_68XX_CMD_RDAUXA, (uint16_t*)cellMonDat);

	// // trigger cell voltage measurement of all cells in fast mode
  //Serial.println("Start ADCV");
	error |= LTC2949_ADxx(
		/*byte md = MD_NORMAL     : */MD_FAST,
		/*byte ch = CELL_CH_ALL   : */CELL_CH_ALL,
		/*byte dcp = DCP_DISABLED : */DCP_DISABLED,
		/*uint8_t pollTimeout = 0 : */0
	);
	delay(2); // wait for conversion results ready to be read
	// print all conversion results
  //Serial.println("end ADCV");
	error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
	// // this is just for debugging anyway, so we don't print all values here. See main loop for the actual cyclic measurements
	// //error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVB, (uint16_t*)cellMonDat);
	// //error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVC, (uint16_t*)cellMonDat);
	// //error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVD, (uint16_t*)cellMonDat);
	// //error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVE, (uint16_t*)cellMonDat);
	// //error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVF, (uint16_t*)cellMonDat);
  
  //Serial.println("Start ADAX ");
  error |= LTC2949_ADAX(
		/*byte md = MD_NORMAL     : */MD_FAST,
		/*byte ch = CELL_CH_ALL   : */CELL_CH_ALL,
		/*byte dcp = DCP_DISABLED : */DCP_DISABLED,
		/*uint8_t pollTimeout = 0 : */0
	);
  delay(2);  // wait for conversion results ready to be read
  //Serial.println("end ADAX");
	// print all conversion results
  error |= ReadPrintAuxVoltages(LTC2949_68XX_CMD_RDAUXA, (uint16_t*)cellMonDat);

	return error;
}

void NtcCfgWrite(int ntc1or2, float rref, float a, float b, float c)
{
	byte data[3];
	LTC2949_FloatToF24Bytes(rref, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_RREF2 : LTC2949_VAL_RREF1, 3, data);
	LTC2949_FloatToF24Bytes(a, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2A : LTC2949_VAL_NTC1A, 3, data);
	LTC2949_FloatToF24Bytes(b, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2B : LTC2949_VAL_NTC1B, 3, data);
	LTC2949_FloatToF24Bytes(c, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2C : LTC2949_VAL_NTC1C, 3, data);
}
byte ReadPrintCellVoltages(uint16_t rdcv, uint16_t * cellMonDat)
{
	// in case LTC2949 is parallel to the daisychain, we now read only the cell voltages
	byte error = LTC2949_68XX_RdCells(rdcv, cellMonDat);
	PrintCellVoltages(cellMonDat, true);
	return error;
}

byte CellMonitorCFGA(byte * cellMonDat, bool verbose)
{
	// read configuration and print
	byte error = LTC2949_68XX_RdCfg(cellMonDat);
	if (verbose)
	{
		//SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
		//PrintComma();
	}
	// set REFON & GPIO in configuration registers
	for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
	{
		cellMonDat[i * 6 + 0] = 0xFC;
    // for ( uint8_t j = 2; j < 8; j++ )
    // {
    //   cellMonDat[i * 6 + 0] |= (1 << j) ; // REFON and GPIO[1-5] as 1
    // }

    // cellMonDat[i * 6 + 1] = 0; //clear UV & OV
    // cellMonDat[i * 6 + 2] = 0; //clear UV & OV
    // cellMonDat[i * 6 + 3] = 0; //clear UV & OV
		cellMonDat[i * 6 + 4] = 0; //clear all DCC
		cellMonDat[i * 6 + 5] = 0; //clear all DCC
	}
	// write configuration registers
	error |= LTC2949_68XX_WrCfg(cellMonDat);
	return error;
}

byte ReadPrintAuxVoltages(uint16_t rdaux, uint16_t * cellMonDat)
{
	// in case LTC2949 is parallel to the daisychain, we now read only the cell voltages
	byte error = LTC2949_68XX_RdAux(rdaux, cellMonDat);
	PrintAuxVoltages(cellMonDat, true);
	return error;
}

byte ChkDeviceStatCfg()
{
	byte error;
	byte data[10];
	byte dataOthers;
	boolean expChkFailed;

	// check STATUS (EXT)FAULTS, ALERT registers
	error = LTC2949_ReadChkStatusFaults(
		/*boolean lockMemAndClr:    */ false,
		/*boolean printResult:      */ false,
		/*byte len:                 */ 10,
		/*byte * statFaultsExpAndRd:*/ data,
		/*boolean * expChkFailed:   */ &expChkFailed,
		/*byte expDefaultSet):      */ LTC2949_STATFAULTSCHK_IGNORE_STATUPD | LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR);

	if (err_detected(error))
		return error;

	if (expChkFailed)
	{
		error = LTC2949_ERRCODE_OTHER; // STATUS (EXT)FAULTS, ALERT check failed
		SerialPrintByteArrayHex(data, 10, true); // report the status
		PrintComma();
		return LTC2949_ERRCODE_OTHER;
	}

	// check BRCEN bit
	if (err_detected(error = LTC2949_READ(LTC2949_REG_REGSCTRL, 1, &dataOthers)))
		return error; // PEC error
	if (bitMaskSetClrChk(dataOthers, LTC2949_BM_REGSCTRL_BCREN, !LTC2949_onTopOfDaisychain))
		return LTC2949_ERRCODE_OTHER; // BRCEN != LTC2949_onTopOfDaisychain

	if (err_detected(error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &dataOthers)))
		return error; // PEC error
	if (dataOthers != LTC2949_BM_OPCTRL_CONT)
		return LTC2949_ERRCODE_OTHER; // not in continuous mode

	if (err_detected(error = LTC2949_READ(LTC2949_REG_FACTRL, 1, &dataOthers)))
		return error; // PEC error
	if (dataOthers != LTCDEF_FACTRL_CONFIG)
		return LTC2949_ERRCODE_OTHER;  // not or wrong fast mode

	if (err_detected(error = LTC2949_ADCConfigRead(&dataOthers)))
		return error; // PEC error
	if (dataOthers != LTCDEF_ADCCFG_CONFIG)
		return LTC2949_ERRCODE_OTHER; // wrong ADC configuration

	return 0;
}