#include <ltcmuc_tools.h>
#include <SPI.h>
#include <LTC2949.h>
#include <SD.h>
#include <EEPROM.h>
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3 , RX_SIZE_256, TX_SIZE_16> can3;//can
CAN_message_t send_msg,receive_msg;//CAN message strut
#define send_id 0x1806E5F4 
#define receive_id 0x18FF50E5 

//  int max_voltage = 3950;                  //1900 coresponds to 190.0V
// int max_current_without_decimal = 100  ; // 100 corresponds to 10.0 Amp
//  float output_voltage=0, output_current=0;
//  bool hw_failure=0, temp_charger=0, input_voltage=0, starting_state=0, comm=0;
 

// IMPORTANT MODES OF CODE

// number of cell monitors
#define LTCDEF_CELL_MONITOR_COUNT 6

#define seventhSlave
#undef seventhSlave// undef only if u dont need seventh slave

#define dynamicCooling
//#undef dynamicCooling //undef only if u dont need to turn fans on 

// CHARGER SECTION

#define charger_active
#undef charger_active //undef in case u are not charging

#define charger_softEnabled
#undef charger_softEnabled



//CHARGER SECTION ENDS

#define Interrupt_Debug
#undef Interrupt_Debug //undef if u want the code to run 

//GUI SECTION
#define GUI_Enabled
#undef GUI_Enabled

//SD CARD SECTION

#define startLogging
//#undef startLogging

#define initialiseEEPROM
#undef initialiseEEPROM

//SOC section
#define startSOC
//#undef startSOC

// circular daisy chain 
#define circular
//#undef circular 

// number of cells per cell monitor (e.g. LTC6811: 12, LTC6813: 18)
// !!!must be multiple of 3!!!
// Its always possible to set less, if not all voltages are of interest 
// for debugging/evaluation. So here we set 6 only.
#define LTCDEF_CELLS_PER_CELL_MONITOR_COUNT 18

// define this for usage of LTC681X only, without LTC2949
#define LTCDEF_LTC681X_ONLY
#undef  LTCDEF_LTC681X_ONLY

//IMORTANT MODES OF CODE ENDS


#define LTCDEF_TIMING_DEBUG
#undef LTCDEF_TIMING_DEBUG

// serial baudrate
//Note: on Arduino Due, 250000 baud seems to be the maximum
#define LTCDEF_BAUDRATE 250000

// default chip select of the LTC6820
// Note: SS is mapped differently on Arduino Zero / DUE
//       thus direct pin number (10) should be used instead
#define LTCDEF__CS 10
#define LTCDEF__CS2 37
// chip select of the optional 2nd LTC6820
#define LTCDEF_GPO 37

// set sense resistor here
#define LTCDEF_SENSE_RESISTOR 1000e-6
// significant digits
// voltages
#define LTCDEF_DIGITS_BATSLOW     3
#define LTCDEF_DIGITS_BATFAST     2
#define LTCDEF_DIGITS_BATFIFO_AVG 2
#define LTCDEF_DIGITS_CELL        4
// currents
#define LTCDEF_DIGITS_I1SLOW      3
#define LTCDEF_DIGITS_I2FAST      0
#define LTCDEF_DIGITS_I2FIFO_AVG  0
// power
#define LTCDEF_DIGITS_P1SLOW      3
// temperature
#define LTCDEF_DIGITS_TEMPSLOW    1

#ifdef LTCDEF_LTC681X_ONLY
// polling not supported in case of LTCDEF_LTC681X_ONLY!!!
#define LTCDEF_POLL_EOC false
#else
// true for making ADCV command and poll EOC
// false for making ADCV command and wait until EOC
#define LTCDEF_POLL_EOC true
#endif

// define LTCDEF_IGNORE_PEC_ERRORS 
// quick work around for new 6815 style devices with PEC+CMDcnt 
// that is currently not supported by this source
#define LTCDEF_IGNORE_PEC_ERRORS
// #undef LTCDEF_IGNORE_PEC_ERRORS
//
#ifdef LTCDEF_IGNORE_PEC_ERRORS
// ignore all PEC errors
inline bool err_detected(byte error) { return error > 0x3; }
// or just ignore single PEC errors
//inline bool err_detected(byte error) { return (ALLOW_ONE_PEC_ERROR ? ((error) > 1) : ((error) != 0)); }
#else
inline bool err_detected(byte error) { return error != 0; }
#endif

// NTC: NTCALUG01A104F/TDK (SMD)
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#define NTC_RREF   100e3

// fast channel configuration: fast single shot, channel 2: I1, BAT (via P2 as voltage see below)
#define LTCDEF_FACTRL_CONFIG LTC2949_BM_FACTRL_FACH2
// ADC configuration (SLOT1 measures temperature via NTC, P2 measures voltage)
#define LTCDEF_ADCCFG_CONFIG (LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_P2ASV)

#define LTCDEF_ERR_RETRIES 1


// defined by me 
#define BMS_FLT_3V3 18              // PIN 18 ( A4 ) selected as BMS ERROR pin on teensy can check on master
#define FAN_MBED 19                 // FAN mbed signal to ACU
#define CH_EN_MBED 22               // charger enable signal to ACU
#define PEC_ERR_PIN 9               // to switch error state led on master
#define VOLT_ERR_PIN 7              // to switch error state led on master
#define TEMP_ERR_PIN 8              // to switch error state led on master

#define voltTimer 400               // Time for voltage loop to go on in its error checking mode
#define tempTimer 900                // Time for temperature loop to go on in its error checking mode
#define chargeTimer 5000

#define voltage 0
#define temp 1

#define UI_BUFFER_SIZE 64

#define RES 30000                   // Resistor divider for BATP and BATM
#define RES_SUM 6520000
#define POT_DIV_BPM 76.5   // Voltage Multiplier for BPM

//CELL COUNT OF STACK
int cellsPerStack[7]={15,15,15,15,15,15,6};


//NORMALIZATION SECTION

const uint8_t slavesToNormalize = 0x33; // S1 , S3, S6, S7 ->The least significant bit(first from right) represents the first slave in daisy chain and so on...
//                                     // In binary system if it is 1 it means that slave needs normalize convert the binary to hex and put the value  here
//                                     // For ex. if I have slave 1 and 3 to normalize the binary equivalent will be 00000101 convert binary to hex and proceed

// const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00};//
const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00,0x00,0x00,0x00};
// //const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00,0x00,0x00};
// // const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00,0x00};
// // const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00};
// // const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00};
//const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00};

// const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x40,0x00,0x00,0x00,0x00,0x05,0x00};
const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x201,0x00,0x00,0x00,0x20,0x00};
// //const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00,0x00,0x00};
// // const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00,0x00};
// // const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00,0x00};
// // const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00,0x00};
//const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00};

// //NORMALIZATION SECTION ENDS

//SPLIT SECTION

const uint8_t slavesOnSplit = 0x01;// Least significant bit is first slave 

//const uint16_t voltChannelOnSplit[LTCDEF_CELL_MONITOR_COUNT] = { 0x00}; // if SPlit between ch-1 and ch-2 pass "ch-2" here
const uint16_t voltChannelOnSplit[LTCDEF_CELL_MONITOR_COUNT] = { 0x80,0x00,0x00,0x00,0x00,0x00 };
//const uint16_t voltChannelOnSplit[LTCDEF_CELL_MONITOR_COUNT] = { 0x02,0x00,0x00,0x00,0x00 };
//const uint16_t voltChannelOnSplit[LTCDEF_CELL_MONITOR_COUNT] = { 0x02,0x00,0x00,0x00 };
//const uint16_t voltChannelOnSplit[LTCDEF_CELL_MONITOR_COUNT] = { 0x02,0x00,0x00};
//const uint16_t voltChannelOnSplit[LTCDEF_CELL_MONITOR_COUNT] = { 0x02,0x00 };
//const uint16_t voltChannelOnSplit[LTCDEF_CELL_MONITOR_COUNT] = { 0x0 };

// Normalize all channels
// const uint8_t slavesToNormalize = 0xff; // S3 and S6
// const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0xffff, 0xffff,0xffff,0xffff,0xffff,0xffff,0xffff };
// const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0xffff, 0xffff,0xffff,0xffff,0xffff,0xffff,0xffff };

//const uint8_t slavesToNormalize = 0x00;
//const uint16_t voltChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00};
//const uint16_t tempChannelToNormalize[LTCDEF_CELL_MONITOR_COUNT] = { 0x00};

//SPLIT SECTION ENDS

// THRESHOLD SECTION

const double overVoltageThreshold = 4.200;
const double underVoltageThreshold = 2.8000;
const double overTempThreshold = 45.000;
const double dynamicCoolingThreshold = 35.000;

//CHARGER THRESHOLD
int max_voltage = 3600;                  //1900 coresponds to 190.0V
int max_current_without_decimal = 100  ; // 100 corresponds to 10.0 Amp
float output_voltage=0, output_current=0;
bool hw_failure=0, temp_charger=0, input_voltage=0, starting_state=0, comm=0;






//THRESHOLD SECTION ENDS

const bool fanMbed = true;
const bool chargerEnMbed = false;
const bool amsMasterToACUsignals[2] = { fanMbed , chargerEnMbed };
int flag_fan =0;


// SD Card logging 
int chgrDelay = 1000;
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
bool BPM_ready;
bool enteredChecking=false;
bool chargingStarted_=true;
bool chargingFlag=false;

//SOC estimation
bool SOC_init_flag=false;

float EnergyAvailable = 0;

float tf=0;

double cellVoltages[LTCDEF_CELL_MONITOR_COUNT][15];     // cell voltages data
int8_t voltageErrorLoc[LTCDEF_CELL_MONITOR_COUNT][15];  // gives location of voltage error (-1 in case of error / 0 in case of no error)
int8_t voltageNormalize[LTCDEF_CELL_MONITOR_COUNT][15];
int8_t voltageSplit[LTCDEF_CELL_MONITOR_COUNT][15];
double cellTemperatures[LTCDEF_CELL_MONITOR_COUNT][15]; // cell temp data
int8_t tempErrorLoc[LTCDEF_CELL_MONITOR_COUNT][15];  // gives location of voltage error (-1 in case of error / 0 in case of no error)
int8_t tempNormalize[LTCDEF_CELL_MONITOR_COUNT][15];
float batVoltage_num;
float batCurrPower_num[1];
String batVoltage;                                  // TS voltage
String batCurrPower[2];                             // 0:TS_curr ; 1:TS_power

String CellData = "";
String CellData_GUI="";

// circular daisy chain 
bool loopcount = true;
float tolerance = 1;
double cells[LTCDEF_CELL_MONITOR_COUNT][18];
double cells1[LTCDEF_CELL_MONITOR_COUNT][18];
double cells2[LTCDEF_CELL_MONITOR_COUNT][18];

double auxHigh1[LTCDEF_CELL_MONITOR_COUNT][12];
double auxLow1[LTCDEF_CELL_MONITOR_COUNT][12];
double auxHigh2[LTCDEF_CELL_MONITOR_COUNT][12];
double auxLow2[LTCDEF_CELL_MONITOR_COUNT][12];
double auxHigh[LTCDEF_CELL_MONITOR_COUNT][12];
double auxLow[LTCDEF_CELL_MONITOR_COUNT][12];

struct maxMinParameters
{
  double val;
  uint8_t slaveLoc;
  uint8_t cellLoc;
};

struct maxMinParameters maxVoltage;
struct maxMinParameters minVoltage;
struct maxMinParameters maxTemp;

int8_t errorFlag[9] = {0};
bool bmsFlag;
bool voltFlag;
bool tempFlag;
bool chargerFlag;
char ui_buffer[UI_BUFFER_SIZE];

uint16_t cellMonDat[LTCDEF_CELL_MONITOR_COUNT * 3]; // = LTCDEF_CELL_MONITOR_COUNT * 6 bytes
byte * buffer = (byte *)cellMonDat; // buffer and cellMonDat can share the same memory
byte  error = 0;  

#ifndef LTCDEF_LTC681X_ONLY
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
#endif

#ifndef LTCDEF_LTC681X_ONLY
	// store the last TBx value that was read (to be able to calc time difference later...)
	uint32_t deltaT = LTC2949_GetLastTBxInt();
#endif

unsigned long mcuTime;
uint8_t retries = LTCDEF_ERR_RETRIES;




void cellVoltageLoop(unsigned long timeBuffer);    
void cellsVoltSort(void);  
void printCells(void);
void voltagePlausibilityCheck(void);
void cellTempLoop(unsigned long timeBuffer);  
void printAux(void);
void tempPlausibilityCheck(void);
void tempConvertSort(bool muxSelect);
void batLoop(bool slowChannelReady);
void checkError(void); 
void pull_3V3_high(void);             
void clearFlags(void);   
void clearFlagsV(void);
void clearFlagsT(void);
void transferT(uint16_t * data, uint8_t nic, bool muxSelect, uint8_t auxReg);            
void transferV(uint16_t * data, uint8_t nic, uint8_t cellReg);                       
void circularVoltdef(void);
void circularTempdef(bool muxSelect);
void chechVoltageFlag(void);
void checkTempFlag(void);
void errorCheckingMode(void);
void triggerError(void);
void triggerInterrupt(void);
void printErrLocV(void);
void printErrLocT(void);
void initialiseFlags(void);
void initialiseCAN(void);
void Initialisation(void); //left not found function in main code
void switchErrorLed(void);
void initialiseNormalizeChannel(void);
void setNormalizeChannels(uint8_t ic, uint16_t * vChannels,uint16_t * tChannels );
void setNormalizeFlag(uint8_t nic, uint16_t voltChannels, uint16_t tempChannels);
void checkVoltNormalize(void);
void checkTempNormalize(void);
void initialiseSplitChannel(void);
void setVoltSplitChannels(uint8_t ic, uint16_t * channels );
void setVoltSplitFlag(uint8_t nic, uint16_t channels );
void checkVoltSplit(void);
void checkACUsignalStatus(void);
void findMax(double array[LTCDEF_CELL_MONITOR_COUNT][15], uint8_t type );
void findMin(double array[LTCDEF_CELL_MONITOR_COUNT][15]);
void printMaxMinParameters(void);
void performDynamicCooling(void);
void chargerLoop(void);
void triggerchargerError(void);
void printchargerError(void);
void cellsLogging(void);
void auxLogging(void);
void maxMinLogging(void); // not found
void SDcardLogging(void);
void initialiseSDcard(void);
float InitialiseEnergy(float, float);
float CalculateEnergy(float, float, float, float, float);
float linear_interpolate(float, float, float, float, float);
String CellVoltagesToString(uint16_t , uint8_t);
void PrintAuxVoltages(uint16_t, boolean);
void PrintCellVoltages(uint16_t, boolean);
void Init(uint8_t, boolean);
void checkVoltageFlag(void);
byte CellMonitorCFGB(byte * cellMonDat, bool verbose, bool muxSelect);
double voltToTemp( double, double);
void Interrupt(void);
float avg( float, float);
byte WakeUpReportStatus();
byte Cont(boolean enable);
byte CellMonitorInit();
void NtcCfgWrite(int ntc1or2, float rref, float a, float b, float c);
byte ReadPrintCellVoltages(uint16_t rdcv, uint16_t * cellMonDat);
byte CellMonitorCFGA(byte * cellMonDat, bool verbose);
byte ReadPrintAuxVoltages(uint16_t rdaux, uint16_t * cellMonDat);
byte ChkDeviceStatCfg();









