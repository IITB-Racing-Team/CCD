#include <ltcmuc_tools.h>
#include <SPI.h>
#include <LTC2949.h>
#include <SD.h>
#include <EEPROM.h>
#include <FlexCAN_T4.h>
#include <Defination.h>
void setup()
{
	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	// wait for serial port to connect. Needed for native USB port only
	// while (!Serial);
	// disable drivers on all SPI pins
	// we do this, as to allow compatibility of Arduino Due
	// with existing Arduino shields that expect SPI signals 
	// on those pins, by routing the SPI header of DUE to
	// those pins.
  delay(50);
	pinMode(11, INPUT);
	pinMode(12, INPUT);
	pinMode(13, INPUT);

	digitalWrite(LTCDEF_GPO, HIGH);
	pinMode(LTCDEF_GPO, OUTPUT);

  pinMode( PEC_ERR_PIN , OUTPUT );
  pinMode( VOLT_ERR_PIN , OUTPUT );
  pinMode( TEMP_ERR_PIN , OUTPUT );
  pinMode( BMS_FLT_3V3 , OUTPUT );     // Bpin defined for sending signal to ACU for BMS error
  pinMode( FAN_MBED , OUTPUT );
  pinMode( CH_EN_MBED , OUTPUT );

  
  initialiseSDcard();

  initialiseNormalizeChannel();
  setNormalizeChannels(slavesToNormalize, voltChannelToNormalize, tempChannelToNormalize );

  initialiseSplitChannel();
  setVoltSplitChannels(slavesOnSplit, voltChannelOnSplit );

  digitalWriteFast( BMS_FLT_3V3 , LOW);
  checkACUsignalStatus();
  delay(10000);
	// configure SPI, also done in LTC2949.cpp:
	// also used for LTC681x
	// LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);
  LTC2949_SPISettings = SPISettings(1000000, MSBFIRST, LTC2949_DEFAULT_SPIMODE);
  #ifndef circular
	Init(LTCDEF__CS, false);
  #endif

  initialiseFlags();//initialises BMS,VOLT,TEMP Flags to False
  initialiseCAN();//enables CAN communication for charger

  for (int i = 0 ; i < LTCDEF_CELL_MONITOR_COUNT ; i++){
    for (int j = 0 ; j<18 ; j++){
      cells2[i][j]=6;
    }
  }
  
}

void loop()
{
  #ifdef circular
   if(loopcount){
    Init(LTCDEF__CS,false);
    Serial.println("");
    Serial.println("Entered normal daisy chain");
  }
  else{
    Init(LTCDEF__CS2,false);
    Serial.println("");
    Serial.println("Entered backward daisy chain");
  }
  #endif

  #ifndef GUI_Enabled
	Serial.print("Start of the code ");
  #endif
  float ti = millis();
  String str = "";

	unsigned long timeBuffer = millis();
	unsigned long timerus = micros();


#ifdef LTCDEF_DO_RESET_TEST
	if (deltaT > LTCDEF_DO_RESET_TEST / 1.0e3 / LTC2949_LSB_TB1)
	{
		LTC2949_reset();
		delay(LTC2949_TIMING_AUTO_SLEEP_MAX*1.5);
		Init(LTC2949_CS, LTC2949_onTopOfDaisychain);
		return;
  }
#endif // LTCDEF_DO_RESET_TEST

	// LTC2949_ChkUpdate checks for changed in TBx (default is to check TB4)
	// this tells if we have updated values available in the slow channel registers
#ifdef LTCDEF_LTC681X_ONLY
	const boolean slowChannelReady = false;
#else
  boolean slowChannelReady = true;
  if(loopcount){
	boolean slowChannelReady = LTC2949_ChkUpdate(&error);
	if (slowChannelReady || LTC_TIMEOUT_CHECK(timeBuffer, mcuTime + LTC2949_TIMING_CONT_CYCLE))
	{
		// in case of any error below we will also enter here! (see last delay(LTC2949_TIMING_IDLE2CONT2UPDATE))
		error |= ChkDeviceStatCfg();
		slowChannelReady = true;
	}
  }
#endif

	// new high precision results available
	// calc difference between last TBx and current TBx, report milliseconds
#ifndef LTCDEF_LTC681X_ONLY
if(loopcount){
	if (slowChannelReady){
		str += String((unsigned int)((LTC2949_GetLastTBxInt() - deltaT) * (1.0e3 * LTC2949_LSB_TB1))); }//Serial.print((unsigned int)((LTC2949_GetLastTBxInt() - deltaT) * (1.0e3 * LTC2949_LSB_TB1)));
	else{}
}
#endif
		str += String(timeBuffer - mcuTime); //Serial.print(timeBuffer - mcuTime);
	mcuTime = timeBuffer;
	str += ',';//PrintComma();

#ifndef LTCDEF_LTC681X_ONLY
if(loopcount){
	// read high precision current I1
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_I1, 3, buffer);
		str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I1SLOW); //Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I1SLOW);
    batCurrPower_num[0] = LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I1SLOW;
    batCurrPower[0] = String(batCurrPower_num[0]);
    
    //if (batCurrPower[0]>-1){
    //  batCurrPower[0] = 0;
    //}

	}
	str += ',';//PrintComma();

	// read high precision power P1
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_P1, 3, buffer);
		str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_P1SLOW); //Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_P1SLOW);
	  batCurrPower[1] = String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1 / LTCDEF_SENSE_RESISTOR * 13.75786, LTCDEF_DIGITS_P1SLOW);
  }
	str += ',';//PrintComma();

	// read voltage BAT
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_BAT, 2, buffer);
    str += " BATV ";
		str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT * POT_DIV_BPM, LTCDEF_DIGITS_BATSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT, LTCDEF_DIGITS_BATSLOW);
	  batVoltage_num = LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT * POT_DIV_BPM, LTCDEF_DIGITS_BATSLOW;
    batVoltage = String(batVoltage_num);
   // batVoltage = String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT * POT_DIV_BPM, LTCDEF_DIGITS_BATSLOW);
  }
	str += ',';//PrintComma();

	// read temperature via SLOT1
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, buffer);
		str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW);
	}
	str += ',';//PrintComma();

	// read internal temperature
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_TEMP, 2, buffer);
		str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW);
	}
	str += ',';//PrintComma();
}
#endif

#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif

	////////////////////////////////////////////////////////////////////
	// fast synchronous cell voltage and current measurement
	////////////////////////////////////////////////////////////////////
	// clear old cell voltage conversion results
  
  if(loopcount){
	error |= LTC2949_68XX_ClrCells();
  error |= LTC2949_68XX_ClrAux();

	error |= CellMonitorCFGA((byte*)cellMonDat, false);
  error |= CellMonitorCFGB((byte*)cellMonDat, false , false);

	if (err_detected(error))
	{
		str = "";
		// not yet initialized or communication error or.... (e.g. ON TOP OF instead of PARALLEL TO DAISYCHAIN)
		PrintOkErr(error);
		delay(100); // we wait 0.1 second, just avoid too many trials in case of error.

		if (retries--)
		{
			Init(LTC2949_CS, LTC2949_onTopOfDaisychain);
		}
		else
		{
			// lets try to toggle isoSPI bus configuration and LTC6820 master (four possible combinations)
			// Note: here we treat all possible combinations to be equally likely. This way the boards
			// can be connected together in any way.
			// In a real system only two combinations would make sense.
			switch (LTC2949_CS + (LTC2949_onTopOfDaisychain ? (uint8_t)128 : (uint8_t)0))
			{
			case (LTCDEF__CS + 0):
				Init(LTCDEF_GPO, false);
				break;
			case (LTCDEF_GPO + 0):
				Init(LTCDEF__CS, true);
				break;
			case (LTCDEF__CS + 128):
				Init(LTCDEF_GPO, true);
				break;
			case (LTCDEF_GPO + 128):
			default:
				Init(LTCDEF__CS, false);
				break;
			}
			retries = LTCDEF_ERR_RETRIES;
		}
		return;
	}
  }

	if (LTCDEF_POLL_EOC)
		timeBuffer = micros();

	// trigger measurement (broadcast command will trigger cell voltage and current measurement)
  BPM_ready=slowChannelReady;
  pull_3V3_high();
  switchErrorLed();
  clearFlags();
  cellVoltageLoop(timeBuffer);
  #ifndef GUI_Enabled
  Serial.println();
  #endif
  
 
  cellTempLoop(timeBuffer);

  

  findMax(cellVoltages, voltage);
  findMax(cellTemperatures, temp);
  findMin(cellVoltages);
   #ifndef GUI_Enabled
   printMaxMinParameters();
   #endif

  #ifdef dynamicCooling
  performDynamicCooling();
  #endif

  checkError();
  #ifndef GUI_Enabled
  triggerError();
  #endif
  #ifdef GUI_Enabled
  triggerError_GUI();
  #endif

  
  batLoop(slowChannelReady);

  // #ifdef GUI_Enabled
  // Serial.print(CellData);
  //   CellData = "";
  // #endif

  

  

  #ifdef charger_active
  if (!(voltFlag || tempFlag))
  {

  chargerLoop();
  triggerchargerError();
  }
  #endif


  #ifndef GUI_Enabled
  triggerInterrupt();
  #endif

  #ifdef GUI_Enabled
  triggerInterrupt_GUI();
  #endif


  // #ifdef GUI_Enabled
  // Serial.println(CellData);
  //   CellData = "";
  // #endif

  

  

#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif


#ifndef LTCDEF_LTC681X_ONLY
if(loopcount){
	// print fast I2
	str += String(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I2FAST); //Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I2FAST);
	str += ',';//PrintComma();
	// print fast BAT
	str += String(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFAST); //Serial.print(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFAST);
	// clear the EOC by reading again if necessary:
	if (!LTC2949_FASTSSHT_HS_CLR(fastData2949))
		error |= LTC2949_RdFastData(fastData2949);
	if (!LTC2949_FASTSSHT_HS_CLR(fastData2949)) // for sure HS bytes must be cleared now
		error |= LTC2949_ERRCODE_OTHER;
	str += ',';//PrintComma();
	str += String(deltaT); //Serial.print(deltaT);
	str += ',';//PrintComma();
}
#endif
	// Serial.print(str);
  // Print
  // Serial.println(myCvs[0]);
  // Serial.print("BPM : ");
  // Serial.println(bpmDat[2]);
	str = "";
	// PrintOkErr(error);
	if (err_detected(error)) // in case of error we sleep to avoid too many error reports and also to make sure we call ChkDeviceStatCfg in the next loop 
		delay(LTC2949_TIMING_IDLE2CONT2UPDATE);

  #ifdef GUI_Enabled
  
  Serial.println();
  
  float tf = millis() - ti;
  
  CellData_GUI = "";
  CellData_GUI += String(tf,2);
  CellData_GUI += ",";
  Serial.print(CellData_GUI);
  

  

  CellData_GUI = "";
  
  Serial.println();
  
  #endif


  #ifdef startSOC
  if (SOC_init_flag==false)
  {
    EnergyAvailable = InitialiseEnergy(minVoltage.val, underVoltageThreshold);
    Serial.print("Energy Available :");
    Serial.print(EnergyAvailable);
    Serial.print("Kwh");
    SOC_init_flag=true;
  }
  else
  {

    EnergyAvailable = CalculateEnergy(batVoltage_num, batCurrPower_num[0], EnergyAvailable, tf, chgrDelay);
    Serial.print("Energy Available in Kwh:");
    Serial.print(EnergyAvailable);
    Serial.print("Kwh");
  }

  Serial.println();
  Serial.print("SoC: ");
  Serial.print(EnergyAvailable*1000*1000*100/(4200*5.5*3.7*90));
  Serial.print("%");
  #endif
  
  #ifndef GUI_Enabled
  Serial.println();
  tf = millis() - ti;
  Serial.print("LOOP TIME : ");
  Serial.print(tf);
  Serial.println("ms");
  #endif

  #ifdef charger_active
  delay(chgrDelay);
  #endif

  #ifdef circular
  loopcount=!loopcount ;
  #endif
}

