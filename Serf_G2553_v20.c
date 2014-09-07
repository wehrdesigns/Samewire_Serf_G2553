/*
* * *
The MIT License (MIT)

Copyright (c) <2014> <Nathan A. Wehr>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
* * *

There might be inconsistencies between the documentation and the code (sometimes the documentation is written in the planning stage before the code is written and does not get updated when the code changes)

Data Acquisition for MSP430 G2553:
	Features:
		1 Mhz
		9600 baud UART (no external clock)
		Half Duplex
		Command and response terminated with Carriage Return
		Addressable - to facilitate multiple devices on the same bus
			single character, almost all characters in 8-bit ASCII set
			Will always respond to the address '!' so only use this address with one serf on the samewire network
			Master is addressed with character '~'
		The command list is at the bottom of this file
		Processor remains in low power mode unless taking measurements or processing communication
		The following functions are available:
			Digital Input
			Digital Output
			Analog Input
			Timing in microseconds
			Capacitive Measurement
			60Hz Wave Measurement for the purpose of measuring power with a current transformer
		Periodic measurements can be averaged.
		Before averaging, data is left shifted by 4 bits to reduce round-off error.
		Long running measurements are interleaved.

	Overview of functionality
		First configure all the ports and then start the service loop if it is set to Run mode

		Service Loop (runs in the Watchdog Timer Interrupt routine)
			Periodic operations can run at different periods (multiples of the WDT period)
				Each service item (referenced by a Port and Pin) has a increment variable and a counter variable and a period count
					The counter is incremented every WDT interrupt period
					When the counter is greater then the period count, that service item will be executed and the counter reset
			Ideally, there will be no long delays in this service loop, it will only initialize, monitor, record and finalize actions

			Goto LPM (Low Power Mode) until:
				timer indicates a periodic measurement should be made
				a Cmd is ready to parse and process
				a trigger pulse is received

			Capacitive measurement by Timer A0
				Prevent discharge until several times the maximum discharge time from the last discharge
					# of watchdog timer flags
				Initialize Discharge cycle that will capture discharge time from Vcc to 0.25*Vcc
					Monitor flag 'bCapTimer'
				Stop timer and set to recharge

			60Hz Wave Average by Timer A0
				Collect N samples in 1/60 second
				Calculate and subtract DC offset (maintain running average of DC offset voltage)
				Full-wave rectify the waveform
				Average the rectified waveform
				Measure AC voltage at the same time as current so that they can be multiplied together for real power.

			Stopwatch Timing of trigger pulses by Timer A1
				First pulse starts the timer
				Next pulses record elapsed time and restart the timer

			PIR trigger counting
				Count triggers
				Reset count when queried

			*More functionality is available but the details are in the code

		RX UART Interrupt
			Filter incoming characters
			Parse commands, then execute commands which place response into SendBuf, then send content of SendBuf

Firmware Configuration (the following section was used for the firmware design and may be superceeded by comments and changes in the code)
	Configuration will be set at initialization
	Configuration change will cause a reset
	Sense/Control Configuration							Config Bits						Byte Offset		Left Shift		And with
		Port											2								0x0000							0x03
		Pin												3								0x0000			2				0x07
		Pin Function (Enumerated) 						3								0x0000			5				0x07
			0 - Digital Input (Usable Port.Pins: P1.0,3,4,5,6,7,P2.1,2,3,4,5,6,7)
					Set Pulldown=0/Pullup=1/Disable=3	2								0x0000
					Interrupt Edge LowToHigh=0/HtoL=1	1								0x0000			2				0x01
					Filter								8								0x0001
					WDT Period (Measurement Frequency)	8								0x0002
					subFunction							4								0x0003
						0 - Read Only by Command
						1 - Timing Measurement
							(Usable Port.Pins: P2.2)
							Note: Uses Timer 1 -- not compatible with 60Hz Wave Measurement
						2 - Filter High - Input must be high for "Filter" number of WDT Cycles before the state is considered a high
						3 - Filter Low - Input must be low for "Filter" number of WDT Cycles before the state is considered a low
						15 - Erased state of information memory
					struct DigitalInput{
						unsigned char PullEdge;
						unsigned char Filter;
						unsigned char subFunction;
					};
			1 - Digital Output - Threshold and/or Command (Usable Port.Pins: P1.0,3,4,5,6,7,P2.1,2,3,4,5,6,7)
					(The input port&pin must be set to an input function)
					Input Port  						2								0x0000							0x03
					Input Pin							3								0x0000			2				0x07
					State								1								0x0000			5				0x01
					Upper Threshold value				16								0x0001			5				0x0A
					Lower Threshold Value 				16								0x0003							0x0A
					subFunction							8								0x0004
						0 - Control by Command Only
								Initial state set by 'State' parameter
						1 - Control by the average value of a Digital Input Pin
						2 - Use Thresholds with A2D Input Port
								state between thresholds is set by the 'State' parameter
						3 - Control by Capacitive Measurement
					struct DigitalOutput{
						unsigned char StatePinPort;
						unsigned char subFunction;
						unsigned int UpperThreshold;
						unsigned int LowerThreshold;
						unsigned char Hysteresis
					};
			2 - A2D Input  (Usable Port.Pins: P1.0,3,4,5,6,7,8)
					Multiplier							16								0x0001
					Offset								16								0x0003
					Points Averaging					8								0x0004
					WDT Period (Measurement Frequency)	8								0x0005
					Decimal Places						4								0x0006							0x0F
					+Vref								2								0x0006			4				0x03
					-Vref								2								0x0006			6				0x03
					Power Port							2								0x0007							0x03
					Power Pin							3								0x0007			2				0x07
					WDT Intervals						3								0x0007			5				0x07
					Power Port and Pin is used to turn off power to a sensor between measurements (ignored if Port is 0)
						Power is turned on WDT interval(s) before a measurement
					struct Measurement{
						unsigned int Multiplier;
						unsigned int Offset;
						unsigned char Points Average;
						unsigned char WDTPeriod;
						unsigned char DecimalPlacesVref;
						unsigned char ArgPinPort;
					};
			3 - Capacitive Measurement (Usable Port.Pins: P1.5,6) Note: Uses Timer 0
					Multiplier							16								0x0001
					Offset								16								0x0003
					Points Averaging					8								0x0004
					WDT Period (Measurement Frequency)	8								0x0005
					Decimal Places						4								0x0006							0x0F
					Charge Pin							3								0x0007							0x07
					Charge Port							2								0x0007			3				0x03
					Use Measurement structure
			4 - 60Hz Wave Measurement (Usable Port.Pins: P1.0,3,4,5,6,7,8) Note: Uses Timer 1 -- not compatible with Trigger Timing Function
					Multiplier							16								0x0001
					Offset								16								0x0003
					Points Averaging					8								0x0004
					WDT Period (Measurement Frequency)	8								0x0005
					Decimal Places						4								0x0006							0x0F
					+Vref								2								0x0006			4				0x03
					-Vref								2								0x0006			6				0x03
					Use Measurement structure

	Available ports: P1.0,3,4,5,6,7,P2.0,1,2,3,4,5,6,7 total: 14


Flash Configuration Status (located Seg B offset 0x00FF)
	I - Initialize all pins to input
		If the flash status is not set to a known configuration status, then the configuration is initialized to all inputs and the status set to II
	R - Run from Configuration
		this status is set by command
	S - Stop
		this status is set by command
		all ports are configured as inputs
		The WDT loop is disabled

Command Format
	C - Command
	F - Fixed Length parameter
	V - Variable Length parameter(s) can be from 1 to 25 characters
	: - separator
	I - ID
	R - Carriage Return

	Basic				ICCR
	Intermediate		ICC:FFR
	Advanced			ICCC:VVVVVVVVVVVVVVVVVVVVVVVVVR

ID stored in flash memory
	the ID for the first command after restart will always be the space character, then the ID will be read from flash memory
	so, if the programmed ID is unknown, it can be read by resetting the board and sending the command ' ID'
	The ID can be reprogrammed by sending 'APID:x' (where x is the new ID and A is the address or ID character in this example)

*/

#include "msp430g2553.h"

#define		Bit_time	104     // 9600 Baud, SMCLK=1MHz (1MHz/9600)=104
#define		Bit_time_5	52      // Time for half a bit.

char *Flash_ptrA = (char *) 0x10C0;                         // Segment A pointer
char *Flash_ptrB = (char *) 0x1080;                         // Segment B pointer
char *Flash_ptrC = (char *) 0x1040;                         // Segment C pointer			holds 4 char configuration for a pin for ports 1,2
char *Flash_ptrD = (char *) 0x1000;                         // Segment D pointer			holds 2 ints for Line and Threshold for ports 1,2

char *FlashConfigStatus = (char *) 0x10BE;
char *FlashID = (char *) 0x10BF;
char *FlashPortConfig = (char *) 0x1080; // to 0x108F -- array of 16 bytes
unsigned int *FlashWDTPeriod = (unsigned int *) 0x1090; // to 0x10AF -- array of 16 words (32 bytes)

unsigned int *FlashTempMultiplier = (unsigned int *) 0x10BC;		// put integers on an address divisible by 2
unsigned int *FlashTempOffset = (unsigned int *) 0x10BA;
unsigned int *FlashTempPeriod = (unsigned int *) 0x10B8;
unsigned int *FlashTempPointsAverage = (unsigned int *) 0x10B6;
char *FlashRedundantComm = (char *) 0x10B5;
char *FlashThisIsAvailableChar = (char *) 0x10B4;
unsigned int *FlashTimingFilter = (unsigned int *) 0x10B2;
//Available Flash_ptrB 0x10B0 and 0x10B1

#define		FWType1			'D'	// DAQ
#define		FWType0			'D'	// Development // Service
#define		Version1		'2'
#define		Version0		'0'

char ID = '?';

char CmdBuf[]="                    ";     // Buffer to store received command and parameters
signed int iCmd = -1;	  		// Index for CmdBuf
volatile char SendBuf[]="                    ";    // Buffer for communications
signed int iSend = -1;			// Index for SendBuf[]
char boolResetNow = 0;

const unsigned char NumPorts = 16;
unsigned int WDTCount[16];		// A counter that increments as long as it is less than WDTPeriod
unsigned long PinAve[16];
unsigned int AveTemp = 0;
unsigned int cntAveTemp;
unsigned char CapActiveIndex = 0;
unsigned char WaveActiveIndex = 0;
unsigned int PinAve2[8];
unsigned int PinAve3[8];
unsigned int TimingMax = 0;
unsigned int TimingMin = 0xFFFF;

char arrConfig[16] = {0x00,	//P1.0	0
					0x00,	//P1.1	0
					0x00,	//P1.2	0
					0x00,	//P1.3	0
					0x00,	//P1.4	0
					0x00,	//P1.5	0
					0x00,	//P1.6	0
					0x00,	//P1.7	0
					0x00,	//P2.0	0
					0x00,	//P2.1	0
					0x00,	//P2.2	0
					0x00,	//P2.3	0
					0x00,	//P2.4	0
					0x00,	//P2.5	0
					0x00,	//P2.6	0
					0x00,	//P2.7	0
					};

struct DigitalInput{
	unsigned char Filter;
	unsigned char PullEdge;
	unsigned char subFunction;
	unsigned char NotUsed;
};

struct DigitalOutput{
	unsigned char StatePinPort;
	unsigned char subFunction;
	unsigned char Hysteresis;
	unsigned char NotUsed;
};

struct Measurement{
	unsigned char PointsAverage;
	unsigned char Sign_MultiplierDecimalPlaces;
	unsigned char DecimalPlacesVref;
	unsigned char ArgPinPort;
};

struct Threshold{
	unsigned int Upper;
	unsigned int Lower;
};

struct Line{
	unsigned int Multiplier;
	unsigned int Offset;
};

const unsigned char StructSizeSegC = 4;
const unsigned char StructSizeSegD = 4;
//const int RelayPulseTime = 50000;

unsigned int StartupDelay = 60;
char WDTActive = 0;
unsigned char Port2DOmem = 0;  // Port 2 Digital Output Memory -- used for latched relay control, so the relay is not continuously pulsed after it is latched
unsigned long Port1Multiplier = 0;

unsigned int uiCapTARRO;
unsigned int cntCapRecharge;
char statCap = 'I';
unsigned int errcntCap;
unsigned char ucCapOverflow;
char bCapOverflow;
char bCapTimer;					// Used for Capacitor Discharge Timer
char bMeasCap;
char bTimerA0inuse = 0;
char ADCDone;					// ADC Done flag
unsigned int ADCValue;			// Measured ADC Value
char errCode;
char bNoComm;
char bCapTimer;					// Used for Capacitor Discharge Timer
unsigned int lastTA0R;

unsigned long lWaveSum;
unsigned long lWaveAve;
unsigned long lVoltSum;
unsigned long lVoltAve;
unsigned long VoltOff = 0;
unsigned long WaveOff = 0;
char cWave;						// Index for iWave
int DCOffset;
int iCurrent;
unsigned int ADCAve;
unsigned int uiCompTimer;
unsigned int uiCapacitiveHumidity;
unsigned char ucCapacitiveHumidity;
unsigned int uiCDS;
unsigned char ucCDS;

char statWave = 'I';
unsigned int	iCycles;		// Count Timer Interrupts
unsigned int	iTriggers;		// Number of port interrupts
unsigned int	iCyclesLast = 0xFFFF;	// Last number of cycles
//unsigned int	iCyclesMax;		// Max cycle count since last read of the max value
//unsigned int	iCyclesMin;		// Min cycle count since last read of the min value
//unsigned int	iCyclesSum;		// Sum 2^x measurements
//unsigned int	iCyclesAve;		// Cycle Average
char bTriggerCounter;			// Count the timer interrupts for the Trigger Counter
unsigned int	uiCCR1;			// First trigger value from TAR
unsigned int	uiTARRO;		// Counter for TAR Roll Over

unsigned int uiWaveMax;
unsigned int uiWaveMin;

//unsigned int DebugBufIndex;
//unsigned int DebugBuf[60];

// Function Definitions
void Transmit(void);
void TransmitDecimalFromLong(unsigned long, char);
void ExecuteCommand(void);
void Single_Measure(unsigned int, unsigned char);
void EnableTriggerCounting(void);
void GetWaveAverage(unsigned char wavePin, unsigned char voltPin, unsigned char wavePave, unsigned char voltPave);
void WaveADC(void);
void MeasCompTimer(unsigned char ucBit);

void InitializeCompTimer(unsigned char ucBit, unsigned char chport, unsigned char chpin);
unsigned long MovingAveFilter(unsigned long ave, unsigned long value, unsigned char PowerOf2Samples);
char ProgramFlashInfoSegment(char *ptrDestSeg,char *ptrDestAddr,char *ptrSource,char NumItems);
unsigned long ConvertAdvCmdParameterFloatToHex(char CmdBufOffset, char MultipleOfTen);
void SendOKNO(char PF);
void AdjustPort1Multiplier(char pin, unsigned long *ave, unsigned long *value);
unsigned long GetAdjustedValue(char pin);
void PortPinOutState(unsigned char Port, unsigned char Pin, char State);

void main(void)
{
	unsigned int i;
//	unsigned long l;
	WDTCTL = WDTPW + WDTHOLD;	// Stop WDT
	if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)
		while(1);                               // If calibration constants erased do not load, trap CPU!!
	FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
	BCSCTL1 = CALBC1_1MHZ;		// Set range
	DCOCTL = CALDCO_1MHZ;		// SMCLK = DCO = 1MHz
	// Initialize USCI UART
    UCA0CTL1 |= UCSWRST;				// Disable USCI
    UCA0CTL1 = UCSSEL_2 + UCSWRST;		//SMCLK
    UCA0MCTL = UCBRF_0 + UCBRS_1;
    UCA0BR0 = 104;
    UCA0CTL1 &= ~UCSWRST;
    IFG2 &= ~(UCA0RXIFG);
//    IE2 |= UCA0RXIE;
    IE2 &= ~UCA0RXIE;			// Disable RX Interrupt until the Startup delay has expired

    // Select Secondary Peripheral Module for USCI P1.1 RX
    P1SEL = BIT1;// + BIT2;
    P1SEL2 = BIT1;// + BIT2;
    // Set P1.2 as output for TX
    P1DIR |= BIT2;
    P1OUT &= ~BIT2;

// decide how to deal with port configuration
    if ((*FlashConfigStatus == 'R') | (*FlashConfigStatus == 'S')){
    	// Load arrConfig from Flash
    	for(i=0;i<16;i++){
    		if(*(FlashPortConfig + i) != 1)  // Set all but the Digital Output to perform a measurement immediately so that any outputs that depend on them will have valid data
    			WDTCount[i] = *(FlashWDTPeriod + i) - 1;  // give the loop 1 cycle for power stabilization
    	}
    }else{// Initialize to Digital Inputs if (&FlashConfigStatus == 'I') | any other value
    	// Write the default arrConfig to Flash
    	ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,arrConfig,16);
    	char S = 'S';
    	ProgramFlashInfoSegment(Flash_ptrB,FlashConfigStatus,&S,1);
		__delay_cycles (40000);
		WDTCTL = 0; // Reset
    }

	if (*FlashConfigStatus == 'R'){
	// Configure Ports
		unsigned char pin;
		unsigned char bit;
		unsigned char port;
		void *ptrStruct;
		for (i=0;i<16;i++){
			pin = i;
			port = 1;
			if (i>7){
				pin = i-8;
				port = 2;
			}
			bit = 1<<pin;
			ptrStruct = Flash_ptrC + StructSizeSegC * i;
			if(*(FlashPortConfig + i)==1){		// Digital Output
				struct DigitalOutput *DO = ptrStruct;
				char b = (DO->StatePinPort >> 5) & 0x01;

				if(port==1){
					if (b){
						P1OUT |= bit;
					}else{
						P1OUT &= ~bit;}
					if (b) P1OUT |= bit;
					P1REN &= ~bit;
					P1DIR |= bit;
					P1SEL &= ~bit;
					P1SEL2 &= ~bit;
				}
				if(port==2){
					if (b){
						P2OUT |= bit;
					}else{
						P2OUT &= ~bit;}
					if (b) P2OUT |= bit;
					P2REN &= ~bit;
					P2DIR |= bit;
					P2SEL &= ~bit;
					P2SEL2 &= ~bit;
				}
			}else if(*(FlashPortConfig + i)==2){	// A2D Input
				if(port==1){ // configure the A2D port
					P1DIR &= ~bit;
					P1REN &= ~bit;
					ADC10AE0 |= bit;
					ADC10CTL0 &= ~ENC;				// Disable ADC
					ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE + REFON; //Turn on the reference all the time so it is not necessary to wait for it to stabilize
				}
				// **** Don't configure the power pin here because it might be overwritten later
				//			Use the Pin Configuration to set the power pin as an output pin
//				// configure the pin that is used to provide power to the sensor
//				struct Measurement *M = ptrStruct;
//				unsigned char pwrbit = 1<<((M->ArgPinPort>>2)&7);
//				if (M->ArgPinPort>0){
//					if(M->ArgPinPort&3){//port 1
//						P1DIR |= pwrbit;
//	//					P1OUT &= ~pwrbit;
//					}else{ // port 2
//						P2DIR |= pwrbit;
//	//					P2OUT &= ~pwrbit;
//					}
//				}
			}else if(*(FlashPortConfig + i)==3){	// Capacitive Measurement
	//			M = ptrStruct;

			}else if(*(FlashPortConfig + i)==4){	// 60Hz Wave Measurement
	//			M = ptrStruct;

				if(port==1){
					P1DIR &= ~bit;
					P1REN &= ~bit;
					ADC10AE0 |= bit;
				}
			}else{ 			// Digital Input; f==0 and f==undefined
				struct DigitalInput *DI = ptrStruct;
				unsigned char b = DI->PullEdge & 0x03;
				char e = (DI->PullEdge >> 2) & 0x01;

				if(port == 1){
					if((bit != BIT1) && (bit != BIT2)){//Pins 1 and 2 are used for serial comm
						P1DIR &= ~bit;
						if (b == 3){
							P1REN &= ~bit;
						}else{
							if(b == 1) {P1OUT |= bit;}
							else{P1OUT &= ~bit;}
							P1REN |= bit;
						}
						if (e){P1IES |= e;}
						else{P1IES &= ~e;}
						P1IFG &= ~bit;
					}
				}
				if(port == 2){
					P2DIR &= ~bit;
					if (b == 3){
						P2REN &= ~bit;
					}else{
						if(b) {P2OUT |= bit;}
						else{P2OUT &= ~bit;}
						P2REN |= bit;
					}
					if (e){P2IES |= e;}
					else{P2IES &= ~e;}
					P2IFG &= ~bit;
					if(pin == 2 && DI->subFunction == 1){ // Configure for Timing Measurement
//						P2DIR &= ~BIT2;				// Set as an input
						P2SEL &= ~BIT2;				// Set as IO to use as a trigger for timing
//						P2IES |= BIT2;				// Set Hi/Lo edge select
						iCyclesLast = 0;
//						iCyclesMax = 0;
//						iCyclesMin = 0xFFFF;
//						iCyclesSum = 0;
//						iCyclesAve = 0;
						EnableTriggerCounting();
					}
				}
			}
		}
		WDTCTL = WDT_MDLY_32;
	}else{
		IE2 |= UCA0RXIE;
	}

	if (*FlashID > 32 && *FlashID < 126)
		ID = *FlashID;

	IE1 |= WDTIE;

	__bis_SR_register(SCG0+CPUOFF+GIE); //LPM1 + Interrupts

	while(1);
}

void ExecuteCommand(void){
	char CmdFound = 1;

//	IE2 &= ~UCA0RXIE;			// Disable RX Interrupt
	IE1 &= ~WDTIE;				// Disable WDT
	SendBuf[++iSend]=CmdBuf[0];		// First load the Comm ID

//Simple Commands     ***********************************************
	if (CmdBuf[3] == 0x0D){
		if((CmdBuf[1] == 'F') && (CmdBuf[2] == 'V')){   // Firmware Version
			SendBuf[++iSend]=FWType1;
			SendBuf[++iSend]=FWType0;
			SendBuf[++iSend]=Version1;
			SendBuf[++iSend]=Version0;
		}
		else if((CmdBuf[1] == 'C') && (CmdBuf[2] == 'S')){ // get flash Configuration Status
			SendBuf[++iSend] = *FlashConfigStatus;
		}
		else if((CmdBuf[1] == 'I') && (CmdBuf[2] == 'D')){ // get ID from flash
			SendBuf[++iSend] = *FlashID;
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'V')){ // Temperature Value -- Internal Temperature
			unsigned long m = *FlashTempMultiplier;
			unsigned long am = m * AveTemp;
			unsigned long o = *FlashTempOffset;
			o = o*1000;
			signed long v;
			if(am >= o){
				v = (am - o)/1000;
			}else{
				v = (o - am)/1000;
				SendBuf[++iSend]='-';
			}
			TransmitDecimalFromLong(v,1);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'C')){ // Temperature a2d Count -- Internal Temperature
			TransmitDecimalFromLong(AveTemp,0);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'P')){ // Temperature Period for WDT
			TransmitDecimalFromLong(*FlashTempPeriod,0);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'A')){ // Temperature points Average
			TransmitDecimalFromLong(*FlashTempPointsAverage,0);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'M')){ // Temperature Multiplier
			TransmitDecimalFromLong(*FlashTempMultiplier,4);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'O')){ // Temperature Offset
			SendBuf[++iSend] = '-';
			TransmitDecimalFromLong(*FlashTempOffset,1);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'F')){ // Timing Filter
			TransmitDecimalFromLong(*FlashTimingFilter,0);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'F')){ // Pin Function
			char i;
			for (i=0;i<16;i++)
				SendBuf[++iSend]=*(FlashPortConfig + i)+'0';
		}
		else if((CmdBuf[1] == 'R') && (CmdBuf[2] == 'C')){ // Redundant Communication
			TransmitDecimalFromLong(*FlashRedundantComm,0);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'X')){  // Timing maX (only applies to timing on P2.2)
			TransmitDecimalFromLong(TimingMax,0);
			TimingMax = 0;
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'N')){  // Timing miN (only applies to timing on P2.2)
			TransmitDecimalFromLong(TimingMin,0);
			TimingMin = 0XFFFF;
		}
	}
//Intermediate Commands     *****************************************
	else if((CmdBuf[3] == ':') && (CmdBuf[6] == 0x0D)){

		if((CmdBuf[1] == 'A') && (CmdBuf[2] == 'D')){

		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'V')){  // PinAve Value (Count * Multiplier + Offset)
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i);
			if(*(FlashPortConfig + i)==0){
				struct DigitalInput *DI = (void *)(Flash_ptrC + StructSizeSegC * i);
				unsigned char sf = DI->subFunction;
				unsigned char f = DI->Filter;
				unsigned char Pvalue = 0;
				if(sf == 0)
					Pvalue = PinAve[i];
				if (sf == 2 || sf == 3){
					if (PinAve[i] == f){
						Pvalue = 1;
					}else{
						Pvalue = 0;
					}
					if (sf == 2)
						PinAve[i] = 0; // reset the value when read
				}
				if (sf == 4 || sf == 5){
					if (PinAve[i] == f){
						Pvalue = 0;
					}else{
						Pvalue = 1;
					}
					if (sf == 4)
						PinAve[i] = 0; // reset the value when read
				}
				SendBuf[++iSend] = '0' + Pvalue;

			}else if(f == 2 || f == 3 || f == 4 || f == 5){
				struct Measurement *M = (void *)(Flash_ptrC + StructSizeSegC * i);
				struct Line *L = (void *)(Flash_ptrD + StructSizeSegD * i);
				unsigned long mult = L->Multiplier & 0x0000FFFF;
				unsigned char multdec = M->Sign_MultiplierDecimalPlaces & 0x0F;
				unsigned char sign = (M->Sign_MultiplierDecimalPlaces >> 4) & 0x03;
				unsigned char dec = M->DecimalPlacesVref & 0x0F;
				unsigned long off = L->Offset & 0x0000FFFF;
				unsigned long avemult = mult * GetAdjustedValue(i);
				unsigned long v;
				unsigned char x;
				for (x=1;x<=(multdec-dec);x++)
					off *= 10;
				if(sign == 0){// positive multiplier, positive offset
						v = (avemult + off);
				}
				if(sign == 1){// positive multiplier, negative offset
					if(avemult >= off){
						v = (avemult - off);
					}else{
						v = (off - avemult);
						SendBuf[++iSend]='-';
					}
				}
				if(sign == 2){// negative multiplier, positive offset
					if(avemult > off){
						v = (avemult - off);
						SendBuf[++iSend]='-';
					}else{
						v = (off - avemult);
					}
				}
				if(sign == 3){// negative multiplier, negative offset
						v = (avemult + off);
						SendBuf[++iSend]='-';
				}
				for (x=1;x<=multdec-dec;x++)
					v /= 10;
				TransmitDecimalFromLong(v,dec);
			}
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'C')){  // PinAve Count (A2D, Comparator, Timer)
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			unsigned long val = PinAve[i];
			if (i<8)
				val = GetAdjustedValue(i);
			TransmitDecimalFromLong(val,0);
		}
		else if((CmdBuf[1] == 'P') && ((CmdBuf[2] == 'M') || (CmdBuf[2] == 'O'))){  // Pin (Multiplier or Offset)
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i);
			struct Measurement *M = (void *)(Flash_ptrC + StructSizeSegC * i);
			struct Line *L =  (void *)(Flash_ptrD + StructSizeSegD * i);
			if(f == 2 || f == 3 || f == 4 || f == 5){
				if (CmdBuf[2] == 'M'){
					unsigned char sign = (M->Sign_MultiplierDecimalPlaces >> 4) & 0x03;
					if (sign > 1)
						SendBuf[++iSend] = '-';
					TransmitDecimalFromLong(L->Multiplier,( M->Sign_MultiplierDecimalPlaces & 0x0F));
				}
				if(CmdBuf[2] == 'O'){
					unsigned char sign = (M->Sign_MultiplierDecimalPlaces >> 4) & 0x03;
					if ((sign == 1) || (sign == 3))
						SendBuf[++iSend] = '-';
					TransmitDecimalFromLong(L->Offset,(M->DecimalPlacesVref & 0x0F));
				}
			}else{SendOKNO(0);}

		}
		else if(((CmdBuf[1] == 'U') || (CmdBuf[1] == 'L')) && (CmdBuf[2] == 'T')){  // (Upper or Lower) Threshold
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i);
			struct Threshold *T =  (void *)(Flash_ptrD + StructSizeSegD * i);
			if((f == 1) && ((CmdBuf[4] - '0') == 2)){
				if (CmdBuf[1] == 'U'){
					TransmitDecimalFromLong(T->Upper,0);
				}
				if(CmdBuf[1] == 'L'){
					TransmitDecimalFromLong(T->Lower,0);
				}
			}else{SendOKNO(0);}

		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'P')){  // Pin Period
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			TransmitDecimalFromLong(*(FlashWDTPeriod + i),0);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'A')){  // Pin points Average
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i);
			struct Measurement *M = (void *)(Flash_ptrC + StructSizeSegC * i);
			if(f == 2 || f == 3 || f == 4 || f == 5){
				TransmitDecimalFromLong(M->PointsAverage,0);
			}else{SendOKNO(0);}
		}
		else if((CmdBuf[1] == 'C') && (CmdBuf[2] == 'W')){  // Calculate Watts
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i);
			if(f == 4){
				struct Measurement *M = (void *)(Flash_ptrC + StructSizeSegC * i);
				struct Line *L = (void *)(Flash_ptrD + StructSizeSegD * i);
				unsigned long mult = L->Multiplier & 0x0000FFFF;
				unsigned char multdec = M->Sign_MultiplierDecimalPlaces & 0x0F;
				unsigned char sign = (M->Sign_MultiplierDecimalPlaces >> 4) & 0x03;
				unsigned char dec = M->DecimalPlacesVref & 0x0F;
				unsigned long off = L->Offset & 0x0000FFFF;
				unsigned long avemult = mult * GetAdjustedValue(i);
				unsigned long v;
				unsigned char x;
				for (x=1;x<=(multdec-dec);x++)
					off *= 10;
				if(sign == 0)// positive multiplier, positive offset
					v = (avemult + off);
				if(sign == 1)// positive multiplier, negative offset
					v = (avemult - off);
				if(avemult < off)
					v = 0;
				// reduce the current by 10 to prevent overflow at 20amps
				// assumes current will have at least 1 decimal place
//				for (x=1;x<=1;x++)
//					v /= 10;
				unsigned long current = v;

				i = M->ArgPinPort>>2 & 0x7; // assumes port is 1
				M = (void *)(Flash_ptrC + StructSizeSegC * i);
				L = (void *)(Flash_ptrD + StructSizeSegD * i);
				mult = L->Multiplier & 0x0000FFFF;
				unsigned char multdecV = M->Sign_MultiplierDecimalPlaces & 0x0F;
				sign = (M->Sign_MultiplierDecimalPlaces >> 4) & 0x03;
				unsigned char decV = M->DecimalPlacesVref & 0x0F;
				off = L->Offset & 0x0000FFFF;
				avemult = mult * GetAdjustedValue(i);
				for (x=1;x<=(multdecV-decV);x++)
					off *= 10;
				if(sign == 0)// positive multiplier, positive offset
					v = (avemult + off);
				if(sign == 1)// positive multiplier, negative offset
					v = (avemult - off);
				// reduce the voltage by 10 to prevent overflow when current is 20amps
				// assumes voltage will have at least one decimal place
				for (x=1;x<=(multdecV-decV);x++)
					v /= 10;
				unsigned long watts = v * current;
				// divide current by 10 at this location to limit roundoff error
				// also divide by the number of decimal places; the decimal places of the Voltage will be applied in the transmit function
				for (x=1;x<=(multdec);x++)
					watts /= 10;
				TransmitDecimalFromLong(watts,decV);
			}else{SendOKNO(0);}
		}
		else if((CmdBuf[1] == 'S') && (CmdBuf[2] == 'P')){  // Set Pin
			char PF = 1;
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			if(*(FlashPortConfig + i) == 1){
				if((CmdBuf[4] - '0')==1)
					P1OUT |= 1<<(CmdBuf[5] - '0');
				if((CmdBuf[4] - '0')==2)
					P2OUT |= 1<<(CmdBuf[5] - '0');
			}else{PF=0;}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'R') && (CmdBuf[2] == 'P')){  // Reset Pin
			char PF = 1;
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			if(*(FlashPortConfig + i) == 1){
				if((CmdBuf[4] - '0')==1)
					P1OUT &= ~(1<<(CmdBuf[5] - '0'));
				if((CmdBuf[4] - '0')==2)
					P2OUT &= ~(1<<(CmdBuf[5] - '0'));
			}else{PF=0;}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'T') && (CmdBuf[2] == 'P')){  // Toggle Pin
			char PF = 1;
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			if(*(FlashPortConfig + i) == 1){
				if((CmdBuf[4] - '0')==1)
					P1OUT ^= 1<<(CmdBuf[5] - '0');
				if((CmdBuf[4] - '0')==2)
					P2OUT ^= 1<<(CmdBuf[5] - '0');
			}else{PF=0;}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'B') && (CmdBuf[2] == 'P')){  // Beep (or pulse) pin - subFunction 5,6,7 only  (PP was already used for Pin Period so we have Beep Pin)
			char PF = 1;
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			if(*(FlashPortConfig + i) == 1){
				struct DigitalOutput *DO = (void *)(Flash_ptrC + StructSizeSegC * i);
				char b = ((DO->StatePinPort>>5 & 1) == 1);
				PortPinOutState((CmdBuf[4] - '0'),(CmdBuf[5] - '0'),~b);
				__delay_cycles (50000);
				PortPinOutState((CmdBuf[4] - '0'),(CmdBuf[5] - '0'),b);
			}else{PF=0;}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'S')){  // Pin Setup
			// index 4 is the port and index 5 is the pin
			unsigned int i = 8 * (CmdBuf[4] - '1') + (CmdBuf[5] - '0');
			char x;
			char m;
			char l;
			char h;
			for(x=0;x<4;x++){
				m = (char)*(Flash_ptrC + i * StructSizeSegC + x);
				l = m & 0xF;
				h = m >> 4;
				if(l>9)
					l += ('a' - 10);
				else
					l += '0';
				if(h>9)
					h += ('a' - 10);
				else
					h += '0';
				SendBuf[++iSend] = h;
				SendBuf[++iSend] = l;
			}
		}
	}
//Advanced Commands     *********************************************
	else if((CmdBuf[4] == ':') && (CmdBuf[iCmd] == 0x0D)){
		if((CmdBuf[1] == 'R') && (CmdBuf[2] == 'S') && (CmdBuf[3] == 'T')){ // ReSeT
			SendBuf[++iSend]='O';
			SendBuf[++iSend]='K';
			boolResetNow = 1;
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'R') && (CmdBuf[3] == 'C')){ // Program Redundant Communication status
			//0 = Off, 1 = On
	    	char S = CmdBuf[5] - '0';
	    	char PF = ProgramFlashInfoSegment(Flash_ptrB,FlashRedundantComm,&S,1);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'S') && (CmdBuf[2] == 'C') && (CmdBuf[3] == 'I')){ // Set Configuration to Inputs
	    	char S = 'I';
	    	char PF = ProgramFlashInfoSegment(Flash_ptrB,FlashConfigStatus,&S,1);
//			char arrCP[64];
//			for (x=0;x<64;x++){
//				arrCP[x] = 0;}
//			PF &= ProgramFlashInfoSegment(Flash_ptrC,Flash_ptrC,arrCP,64);
//			PF &= ProgramFlashInfoSegment(Flash_ptrD,Flash_ptrD,arrCP,64);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'C') && (CmdBuf[3] == 'S')){ // Program Configuration Status
			char S = CmdBuf[5];
	    	char PF = ProgramFlashInfoSegment(Flash_ptrB,FlashConfigStatus,&S,1);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'I') && (CmdBuf[3] == 'D')){ // Program ID
			char S = CmdBuf[5];
			char PF = ProgramFlashInfoSegment(Flash_ptrB,FlashID,&S,1);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'P') && (CmdBuf[3] == 'C')){ // Program Pin Count (for testing)
			//decode Port Pin and comma
			unsigned int i = 8 * (CmdBuf[5] - '1') + (CmdBuf[6] - '0');
			unsigned long l;
			char PF = 0;
			if (CmdBuf[7] == ','){
				PinAve[i] = 0;
				// Convert hexadecimal values so that they will be interpreted correctly
				char c;
				for(c=8;c<=15;c++){
					if (CmdBuf[c] > 57)
						CmdBuf[c] -= 7;
					l = CmdBuf[c] - '0';
					l = l  << ((15 - c) * 4);
					PinAve[i] += l;
				}
				if((CmdBuf[5] - '1') == 0)
					AdjustPort1Multiplier((CmdBuf[6] - '0'),&PinAve[i],&l);
				PF = 1;
			}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'T') && (CmdBuf[3] == 'M')){ // Program internal Temperature sensor Multiplier
			unsigned int i = ConvertAdvCmdParameterFloatToHex(0,4);
			char a[2];
			a[1] = i>>8;
			a[0] = i;
			void *ptr = FlashTempMultiplier;
			char PF = ProgramFlashInfoSegment(Flash_ptrB,ptr,a,2);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'T') && (CmdBuf[3] == 'O')){ // Program internal Temperature sensor Offset
			unsigned int i;
			if(CmdBuf[5]=='-')
				i = ConvertAdvCmdParameterFloatToHex(1,1);
			else
				i = ConvertAdvCmdParameterFloatToHex(0,1);
			char a[2];
			a[1] = i>>8;
			a[0] = i;
			void *ptr = FlashTempOffset;
			char PF = ProgramFlashInfoSegment(Flash_ptrB,ptr,a,2);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'T') && (CmdBuf[3] == 'P')){ // Program Temperature Period for internal temperature sensor
//			if (CmdBuf[5] > 57) // Convert Hexadecimal A-F
//				CmdBuf[5] -= 7;
			unsigned int i = ConvertAdvCmdParameterFloatToHex(0,0);
			char a[2];
			a[1] = i>>8;
			a[0] = i;
			void *ptr = FlashTempPeriod;
			char PF = ProgramFlashInfoSegment(Flash_ptrB,ptr,a,2);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'T') && (CmdBuf[3] == 'A')){ // Program Temperature points Average for internal temperature sensor
//			if (CmdBuf[5] > 57) // Convert Hexadecimal A-F
//				CmdBuf[5] -= 7;
			unsigned int i = ConvertAdvCmdParameterFloatToHex(0,0);
			char a[2];
			a[1] = i>>8;
			a[0] = i;
			void *ptr = FlashTempPointsAverage;
			char PF = ProgramFlashInfoSegment(Flash_ptrB,ptr,a,2);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'T') && (CmdBuf[3] == 'F')){ // Program Timing Filter for P2.2 Timing Function
//			if (CmdBuf[5] > 57) // Convert Hexadecimal A-F
//				CmdBuf[5] -= 7;
			unsigned int i = ConvertAdvCmdParameterFloatToHex(0,0);
			char a[2];
			a[1] = i>>8;
			a[0] = i;
			void *ptr = FlashTimingFilter;
			char PF = ProgramFlashInfoSegment(Flash_ptrB,ptr,a,2);
			boolResetNow = 1;
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'P') && (CmdBuf[3] == 'P')){ // Program Pin wdt Period
			// format: port pin,period in decimal
			// index 5 in the port
			// index 6 is the pin
			char PF = 0;
			unsigned char c = CmdBuf[6] - '0';
			if (CmdBuf[5] == '2')
				c += 8;
			if (CmdBuf[7] == ','){
				unsigned int i = ConvertAdvCmdParameterFloatToHex(3,0);
				char a[2];
				a[1] = i>>8;
				a[0] = i;
				void *ptr = FlashWDTPeriod + c;
				PF = ProgramFlashInfoSegment(Flash_ptrB,ptr,a,2);
				boolResetNow = 1;
			}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && ((CmdBuf[2] == 'M') || (CmdBuf[2] == 'O')) && (CmdBuf[3] == '1')){ // Program (Multiplier or Offset) port 1
			// format: pin,(multiplier or offset) in decimal
			// index 5 is the pin
			char PF = 0;
			unsigned int i = (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i);
			if (CmdBuf[6] == ',' && (f == 2 || f == 3 || f == 4 || f == 5)){
				struct Measurement *pM = (void *)(Flash_ptrC + StructSizeSegC * i);
				struct Line *pL = (void *)(Flash_ptrD + StructSizeSegD * i);
				struct Line L;
				L.Offset = pL->Offset;
				L.Multiplier = pL->Multiplier;
				char argindex = 2;
				if (CmdBuf[2] == 'M'){
					if(CmdBuf[7]=='-')
						argindex++;
					L.Multiplier = ConvertAdvCmdParameterFloatToHex(argindex,pM->Sign_MultiplierDecimalPlaces & 0x0F);
				}
				if (CmdBuf[2] == 'O'){
					if(CmdBuf[7]=='-')
						argindex++;
					L.Offset = ConvertAdvCmdParameterFloatToHex(argindex,pM->DecimalPlacesVref & 0x0F);
				}
				PF = ProgramFlashInfoSegment(Flash_ptrD,(void *)pL,(void *)&L,StructSizeSegD);
				boolResetNow = 1;
			}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && ((CmdBuf[2] == 'U') || (CmdBuf[2] == 'L')) && (CmdBuf[3] == '2')){ // Program (Upper or Lower) Threshold port 2
			// format: pin,(Upper or Lower) threshold in decimal
			// index 5 is the pin
			char PF = 0;
			unsigned int i = (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i + 8);
			if (CmdBuf[6] == ',' && (f == 1)){
				struct Threshold *pT = (void *)(Flash_ptrD + StructSizeSegD * (i + 8));
				struct Threshold T;
				T.Upper = pT->Upper;
				T.Lower = pT->Lower;
				if (CmdBuf[2] == 'U')
					T.Upper = ConvertAdvCmdParameterFloatToHex(2,0);
				if (CmdBuf[2] == 'L')
					T.Lower = ConvertAdvCmdParameterFloatToHex(2,0);
				PF = ProgramFlashInfoSegment(Flash_ptrD,(void *)pT,(void *)&T,StructSizeSegD);
				boolResetNow = 1;
			}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'A') && (CmdBuf[3] == '1')){ // Program points Average port 1
			// format: pin,average in decimal
			// index 5 is the pin
			char PF = 0;
			unsigned int i = (CmdBuf[5] - '0');
			unsigned char f = *(FlashPortConfig + i);
			if (CmdBuf[6] == ',' && (f == 2 || f == 3 || f == 4 || f == 5)){
				struct Measurement *pM = (void *)(Flash_ptrC + StructSizeSegC * i);
				struct Measurement M;
				M.PointsAverage = ConvertAdvCmdParameterFloatToHex(2,0);
				M.DecimalPlacesVref = pM->DecimalPlacesVref;
				M.Sign_MultiplierDecimalPlaces = pM->Sign_MultiplierDecimalPlaces;
				M.ArgPinPort = pM->ArgPinPort;
				PF = ProgramFlashInfoSegment(Flash_ptrC,(void *)pM,(void *)&M,StructSizeSegC);
				boolResetNow = 1;
			}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'P') && (CmdBuf[3] == 'F')){	// Program Pin Function
			char offset = (CmdBuf[5] - '1') * 8 + (CmdBuf[6] - '0');
			char arrCP[1];
			char PF = 0;
			if (CmdBuf[7] == ',' && CmdBuf[9] == 0x0D){
				arrCP[0] = CmdBuf[8] - '0';
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig + offset,arrCP,1);
				boolResetNow = 1;
			}
			SendOKNO(PF);
		}
		else if((CmdBuf[1] == 'C') && (CmdBuf[2] == 'P') && (CmdBuf[3] == 'P')){
			unsigned int i = 8 * (CmdBuf[5] - '1') + (CmdBuf[6] - '0');
			// Convert hexadecimal values so that they will be interpreted correctly
			char c;
			for(c=0;c<=20;c++){
				if (CmdBuf[c] > 57)
					CmdBuf[c] -= 7;
			}
			if(*(FlashPortConfig + i) == 0){	// Configure Pin function 0
				//Digital Input
	/*
				CmdBuf[7] = Pulldown=0/Pullup=1/Disable=3
				CmdBuf[8] = Interrupt Edge LowToHigh=0/HtoL=1
				CmdBuf[9] = Filter			(high nibble)
				CmdBuf[10] = Filter			(low nibble)
				CmdBuf[11] = subFunction	(high nibble)
				CmdBuf[12] = subFunction	(low nibble)
	*/
				char arrCP[4];
				arrCP[0] = (CmdBuf[8] - '0') << 2 | (CmdBuf[7] - '0');
				arrCP[1] = (CmdBuf[9] - '0') << 4 | (CmdBuf[10] - '0');
				arrCP[2] = (CmdBuf[11] - '0') << 4 | (CmdBuf[12] - '0');
				arrCP[3] = 0;
				char PF = ProgramFlashInfoSegment(Flash_ptrC,Flash_ptrC + StructSizeSegC * i,arrCP,4);
				boolResetNow = 1;
				SendOKNO(PF);
			}
			if(*(FlashPortConfig + i) == 1){ // Configure Pin function 1
				//Digital Output
	/*
				CmdBuf[7] = Port
				CmdBuf[8] = Pin
				CmdBuf[9] = State
				CmdBuf[10] = Hysteresis					(high nibble)
				CmdBuf[11] = Hysteresis					(low nibble)
				CmdBuf[12] = subFunction
	*/
				char arrCP[4];
				arrCP[0] = (CmdBuf[7] - '0') | (CmdBuf[8] - '0') << 2 | (CmdBuf[9] - '0') << 5;
				arrCP[1] = CmdBuf[12] - '0';
				arrCP[2] = (CmdBuf[10] - '0') << 4 | (CmdBuf[11] - '0');
				arrCP[3] = 0;
				char PF = ProgramFlashInfoSegment(Flash_ptrC,Flash_ptrC + i * StructSizeSegC,arrCP,4);
				boolResetNow = 1;
				SendOKNO(PF);
			}
			if(*(FlashPortConfig + i) == 2){ // Configure Pin function 2
				//A2D Input
	/*
				CmdBuf[7] = Points Average		(high nibble)
				CmdBuf[8] = Points Average		(low nibble)
				CmdBuf[9] = Power Port
				CmdBuf[10] = Power Pin			*** Every sensor must have it's own power port/pin because the software does not track if another sensor has the pin turned on.  One sensor might turn off the power for another sensor if they are combined.
				CmdBuf[11] = WDT Intervals
				CmdBuf[12] = Decimal Places
				CmdBuf[13] = +Vref
				CmdBuf[14] = -Vref
				CmdBuf[15] = Multiplier Decimal Places
				CmdBuf[16] = Sign
	*/
				char PF = 0;
				if ((CmdBuf[5] - '0') == 1){
					char arrCP[4];
					arrCP[0] = (CmdBuf[7] - '0') << 4 | (CmdBuf[8] - '0');
					arrCP[1] = (CmdBuf[16] - '0') << 4 | (CmdBuf[15] - '0');
					arrCP[2] = (CmdBuf[12] - '0') | (CmdBuf[13] - '0') << 4 | (CmdBuf[14] - '0') << 6;
					arrCP[3] = (CmdBuf[9] - '0') | (CmdBuf[10] - '0') << 2 | (CmdBuf[11] - '0') << 5;
					PF = ProgramFlashInfoSegment(Flash_ptrC,Flash_ptrC + i * StructSizeSegC,arrCP,4);
					boolResetNow = 1;
				}
				SendOKNO(PF);
			}
			if(*(FlashPortConfig + i) == 3){ // Configure Pin function 3
				//Capacitive Measurement
	/*
				CmdBuf[7] = Points Average		(high nibble)
				CmdBuf[8] = Points Average		(low nibble)
				CmdBuf[9] = Charge Port
				CmdBuf[10] = Charge Pin
				CmdBuf[11] = Decimal Places
				CmdBuf[12] = Multiplier Decimal Places
				CmdBuf[13] = Sign
				CmdBuf[14] = Arg (1 = switch pin to output to top off capacitor; 0 = don't switch to output for capacitive sensor that cannot handle DC voltage)
	*/
				char PF = 0;
				if ((CmdBuf[5] - '0') == 1 && ((CmdBuf[6] - '0') == 5 || (CmdBuf[6] - '0') == 6)){
					char arrCP[4];
					arrCP[0] = (CmdBuf[7] - '0') << 4 | (CmdBuf[8] - '0');
					arrCP[1] = (CmdBuf[13] - '0') << 4 | (CmdBuf[12] - '0');
					arrCP[2] = (CmdBuf[11] - '0');
					arrCP[3] = (CmdBuf[9] - '0') | (CmdBuf[10] - '0') << 2 | (CmdBuf[14] - '0') << 5;
					PF = ProgramFlashInfoSegment(Flash_ptrC,Flash_ptrC + i * StructSizeSegC,arrCP,4);
					boolResetNow = 1;
				}
				SendOKNO(PF);
			}
			if(*(FlashPortConfig + i) == 4){ // Configure Pin function 4
				//60Hz Wave Measurement
	/*
				CmdBuf[7] = Points Average		(high nibble)
				CmdBuf[8] = Points Average		(low nibble)
				CmdBuf[9] = Decimal Places
				CmdBuf[10] = +Vref
				CmdBuf[11] = -Vref
				CmdBuf[12] = Multiplier Decimal Places
				CmdBuf[13] = Sign
				CmdBuf[14] = Port
				CmdBuf[15] = Pin
				CmdBuf[16] = subFunction
	*/
				char PF = 0;
				if ((CmdBuf[5] - '0') == 1){
					char arrCP[4];
					arrCP[0] = (CmdBuf[7] - '0') << 4 | (CmdBuf[8] - '0');
					arrCP[1] = (CmdBuf[13] - '0') << 4 | (CmdBuf[12] - '0');
					arrCP[2] = (CmdBuf[9] - '0') | (CmdBuf[10] - '0') << 4 | (CmdBuf[11] - '0') << 6;
					arrCP[3] = (CmdBuf[16] - '0') << 5 | (CmdBuf[14] - '0') | (CmdBuf[15] - '0') << 2;
					PF = ProgramFlashInfoSegment(Flash_ptrC,Flash_ptrC + i * StructSizeSegC,arrCP,4);
					boolResetNow = 1;
				}
				SendOKNO(PF);
			}
		}
		//Preset
		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'R') && (CmdBuf[3] == 'E') && (CmdBuf[4] == ':') && (CmdBuf[5] == 'S') && (CmdBuf[6] == 'E') && (CmdBuf[7] == 'T')){
			unsigned int s = ConvertAdvCmdParameterFloatToHex(3,0);
			char PF = 0;
			if(s==0 || s==100){//Internal Temp Sensor
				//Multiplier (4 decimal places)
				char a[2] = {0x04,0x1D};
				PF = ProgramFlashInfoSegment(Flash_ptrB,(void *)FlashTempMultiplier,a,2);
				//Offset (1 decimal place)
				char b[2] = {0x2A,0x12};
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)FlashTempOffset,b,2);
				//Period
				char c[2] = {0x09,0};
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)FlashTempPeriod,c,2);
				//Points Average
				char d[2] = {0x01,0};
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)FlashTempPointsAverage,d,2);
				boolResetNow = 1;
			}
			if(s==1 || s==100){//Capacitive Humidity P1.5,P2.0
				char a[16]; // Needed because arrConfig is overwritten during flash programming
				char x;
				for(x=0;x<16;x++)
					a[x] = *(FlashPortConfig + x);
				a[5] = 3;
				a[8] = 1;
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,a,16);
				//P1.5
				struct Measurement M;
				M.PointsAverage = 4;
				M.Sign_MultiplierDecimalPlaces = 1<<4 | 4;
				M.DecimalPlacesVref = 0;
				M.ArgPinPort = 2 | 0<<2 | 0<<5;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 5),(void *)&M,StructSizeSegC);
				struct Line L;
				L.Multiplier = 2373;
				L.Offset = 465;
				PF &= ProgramFlashInfoSegment(Flash_ptrD,(void *)(Flash_ptrD + StructSizeSegD * 5),(void *)&L,StructSizeSegD);
				char p[2] = {0x1E,0}; // Pin Period
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashWDTPeriod + 5),p,2);
				//P2.0
				struct DigitalOutput DO;
				DO.StatePinPort = 0 | 0<<2 | 0<<5;
				DO.subFunction = 0;
				DO.Hysteresis = 0;
				DO.NotUsed = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 8),(void *)&DO,StructSizeSegC);
				boolResetNow = 1;
			}
			if(s==2 || s==100){//CDS Light Sensor P1.6,P2.5
				char a[16]; // Needed because arrConfig is overwritten during flash programming
				char x;
				for(x=0;x<16;x++)
					a[x] = *(FlashPortConfig + x);
				a[6] = 3;
				a[13] = 1;
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,a,16);
				//P1.6
				struct Measurement M;
				M.PointsAverage = 0;
				M.Sign_MultiplierDecimalPlaces = 2<<4 | 0;
				M.DecimalPlacesVref = 0;
				M.ArgPinPort = 2 | 5<<2 | 1<<5;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 6),(void *)&M,StructSizeSegC);
				struct Line L;
				L.Multiplier = 1;
				L.Offset = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrD,(void *)(Flash_ptrD + StructSizeSegD * 6),(void *)&L,StructSizeSegD);
				char p[2] = {0x1E,0}; // Pin Period
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashWDTPeriod + 6),p,2);
				//P2.5
				struct DigitalOutput DO;
				DO.StatePinPort = 0 | 0<<2 | 0<<5;
				DO.subFunction = 0;
				DO.Hysteresis = 0;
				DO.NotUsed = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 13),(void *)&DO,StructSizeSegC);
				boolResetNow = 1;
			}
			if(s==3){//PIR Motion Sensor P2.1
				char a[16]; // Needed because arrConfig is overwritten during flash programming
				char x;
				for(x=0;x<16;x++)
					a[x] = *(FlashPortConfig + x);
				a[9] = 0;
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,a,16);
				//P2.1
				struct DigitalInput DI;
				DI.Filter = 3;
				DI.PullEdge = 0 | 0<<2; // a pull down is needed for the Panasonic EKMC PIR Motion Sensor
				DI.subFunction = 2;
				DI.NotUsed = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 9),(void *)&DI,StructSizeSegC);
				char p[2] = {0x09,0}; // Pin Period
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashWDTPeriod + 9),p,2);
				boolResetNow = 1;
			}
			if(s==4){//Voltage Humidity Sensor P1.3, power on P2.3
				char a[16]; // Needed because arrConfig is overwritten during flash programming
				char x;
				for(x=0;x<16;x++)
					a[x] = *(FlashPortConfig + x);
				a[3] = 2;
				a[11] = 1;
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,a,16);
				//P1.3
				struct Measurement M;
				M.PointsAverage = 3;
				M.Sign_MultiplierDecimalPlaces = 1<<4 | 4;
				M.DecimalPlacesVref = 0x31;
				M.ArgPinPort = 2 | 3<<2 | 3<<5;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 3),(void *)&M,StructSizeSegC);
				struct Line L;
				L.Multiplier = 1537;
				L.Offset = 238;
				PF &= ProgramFlashInfoSegment(Flash_ptrD,(void *)(Flash_ptrD + StructSizeSegD * 3),(void *)&L,StructSizeSegD);
				char p[2] = {0x1E,0}; // Pin Period
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashWDTPeriod + 3),p,2);
				//P2.3
				struct DigitalOutput DO;
				DO.StatePinPort = 0 | 0<<2 | 0<<5;
				DO.subFunction = 0;
				DO.Hysteresis = 0;
				DO.NotUsed = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 11),(void *)&DO,StructSizeSegC);
				boolResetNow = 1;
			}
			if(s==5){//Voltage Temperature Sensor P1.4, power on P2.4
				char a[16]; // Needed because arrConfig is overwritten during flash programming
				char x;
				for(x=0;x<16;x++)
					a[x] = *(FlashPortConfig + x);
				a[4] = 2;
				a[12] = 1;
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,a,16);
				//P1.4
				struct Measurement M;
				M.PointsAverage = 3;
				M.Sign_MultiplierDecimalPlaces = 1<<4 | 4;
				M.DecimalPlacesVref = 0x11;
				M.ArgPinPort = 2 | 4<<2 | 3<<5;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 4),(void *)&M,StructSizeSegC);
				struct Line L;
				L.Multiplier = 2637;
				L.Offset = 540;
				PF &= ProgramFlashInfoSegment(Flash_ptrD,(void *)(Flash_ptrD + StructSizeSegD * 4),(void *)&L,StructSizeSegD);
				char p[2] = {0x1E,0}; // Pin Period
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashWDTPeriod + 4),p,2);
				//P2.3
				struct DigitalOutput DO;
				DO.StatePinPort = 0 | 0<<2 | 0<<5;
				DO.subFunction = 0;
				DO.Hysteresis = 0;
				DO.NotUsed = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 12),(void *)&DO,StructSizeSegC);
				boolResetNow = 1;
			}
			if(s==6){//Voltage Temperature Sensor P1.7, power on P2.7
				char a[16]; // Needed because arrConfig is overwritten during flash programming
				char x;
				for(x=0;x<16;x++)
					a[x] = *(FlashPortConfig + x);
				a[7] = 2;
				a[15] = 1;
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,a,16);
				//P1.7
				struct Measurement M;
				M.PointsAverage = 3;
				M.Sign_MultiplierDecimalPlaces = 1<<4 | 4;
				M.DecimalPlacesVref = 0x11;
				M.ArgPinPort = 2 | 7<<2 | 3<<5;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 7),(void *)&M,StructSizeSegC);
				struct Line L;
				L.Multiplier = 2637;
				L.Offset = 540;
				PF &= ProgramFlashInfoSegment(Flash_ptrD,(void *)(Flash_ptrD + StructSizeSegD * 7),(void *)&L,StructSizeSegD);
				char p[2] = {0x1E,0}; // Pin Period
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashWDTPeriod + 7),p,2);
				//P2.7
				struct DigitalOutput DO;
				DO.StatePinPort = 0 | 0<<2 | 0<<7;
				DO.subFunction = 0;
				DO.Hysteresis = 0;
				DO.NotUsed = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 15),(void *)&DO,StructSizeSegC);
				boolResetNow = 1;
			}
			if(s==7){//Timing P2.2 (configured for anemometer)
				char a[16]; // Needed because arrConfig is overwritten during flash programming
				char x;
				for(x=0;x<16;x++)
					a[x] = *(FlashPortConfig + x);
				a[10] = 0;
				PF = ProgramFlashInfoSegment(Flash_ptrB,FlashPortConfig,a,16);
				//P2.2
				struct DigitalInput DI;
				DI.Filter = 1;
				DI.PullEdge = 1 | 1<<2; // using Pull-up and Interrupt Edge High to Low
				DI.subFunction = 1;
				DI.NotUsed = 0;
				PF &= ProgramFlashInfoSegment(Flash_ptrC,(void *)(Flash_ptrC + StructSizeSegC * 10),(void *)&DI,StructSizeSegC);
				char p[2] = {0x09,0}; // Pin Period
				PF &= ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashWDTPeriod + 10),p,2);
				boolResetNow = 1;
			}
			SendOKNO(PF);
		}else{CmdFound = 0;}
	}else{CmdFound = 0;}
	unsigned int i;
	for(i=0;i<20;i++)
		CmdBuf[i] = ' ';
	
	if (CmdFound){
		if(*FlashConfigStatus == 'S'){
			SendBuf[++iSend] = ';';
			SendBuf[++iSend] = 'S';
			SendBuf[++iSend] = 'T';
			SendBuf[++iSend] = 'O';
			SendBuf[++iSend] = 'P';
		}
		// If redundant data turned on then duplicate data and add special character
		char SC = 31; // Special Character
		if (*FlashRedundantComm > 0){
			char middle = iSend+2;
			for(i=1;i<=iSend;i++) // copy
				SendBuf[middle + i] = SendBuf[i];
			SendBuf[middle+i] = SC;
			char lastchar = middle + i;
			for(i=iSend;i>0;i--) // shift original data down one index
				SendBuf[i+1] = SendBuf[i];
			SendBuf[1] = SC;
			SendBuf[middle] = SC;
			iSend = lastchar;
		}
		SendBuf[++iSend]=0x0D;
		bTriggerCounter = 0;
		P1SEL |= BIT2;				// Set Port1 Bit2 to USCI UART function
		P1SEL2 |= BIT2;				// Set Port1 Bit2 to USCI UART function
		P1OUT &= ~BIT2;				// NOT SURE WHY THIS HELPS BUT WITHOUT IT THE PORT IS INTERMITTENTLY SET HIGH ABOUT 4 BIT-WIDTHS EARLY (TRIED PUTTING IT AFTER SELECTING THE IO FUNCTION BUT IT DIDN'T HELP THERE)
		__delay_cycles (Bit_time);  // Make sure the Master catches the "NO" start bit - because there is a glitch in the above instructions that is only 10-20us (which is too short for the interrupt) and causes the master leave the line high to too many bits (no start bit is sent)
		for(i=0;i<=iSend;i++){
			UCA0TXBUF = SendBuf[i];
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
		}
		while(UCA0STAT&UCBUSY);		// Wait until TX is complete
		P1SEL2 &= ~BIT2;			// Set Port1 Bit2 to zero volts
		P1SEL &= ~BIT2;				// Set Port1 Bit2 to zero volts by selecting IO function
	}
	iSend=-1;				// Reset SendBuf Index pointer
	iCmd=-1;				// reset RX byte counter
	__delay_cycles (10);
//	IE2 |= UCA0RXIE;		// Enable RX Interrupt

	if (boolResetNow){
		__delay_cycles (40000);
		WDTCTL = 0;
	}
	IE1 |= WDTIE;				// Enable WDT
}

//Watch Dog Timer Interrupt
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	WDTActive = 1;
	char state;
	unsigned char inport;
	unsigned char inpin;
    unsigned char pin;	//Pin of a Port
    unsigned char bit;	//Binary Bit of the Pin
	unsigned char f;	//Function
    unsigned char port;	//Port 1 or 2
    unsigned char sf;	//subFunction
    unsigned char i;
    unsigned char p;
	unsigned char nv;
	unsigned char pv;
	unsigned char chport;
	unsigned char chpin;
	unsigned char arg;
//	unsigned int ut;
//	unsigned int lt;
	unsigned long l;
	struct DigitalInput *DI;
	struct DigitalOutput *DO;
	struct Measurement *M;

    for (i=0;i<16;i++){
    	if (i==1 || i==2) // skip P1.1 and P1.2 which are TX and RX
    		continue;
    	if (WDTCount[i] < *(FlashWDTPeriod + i)){
    		WDTCount[i]++;}
    	else{
        	pin = i;
        	port = 1;
        	if(i>7){
        		pin = i-8;
        		port = 2;
        	}
        	bit = 1<<pin;
			if(*(FlashPortConfig + i)==0){ 			// Digital Input; f==0
				DI = (void *)(Flash_ptrC + StructSizeSegC * i);
				sf = DI->subFunction;
				f = DI->Filter;
				/* Functions of DI Function
							0 - Read Only by Command
							1 - Timing Measurement
								(Usable Port.Pins: P2.2)
								Filter is used for EWMA
								Note: Uses Timer 1 -- not compatible with 60Hz Wave Measurement
							2 - Filter High - Input must be high for "Filter" number of WDT Cycles before the state is considered a high
							3 - 		same as above with reset when condition goes away (PinAve counts down to zero)
							4 - Filter Low - Input must be low for "Filter" number of WDT Cycles before the state is considered a low
							5 - 		same as above with reset when condition goes away (PinAve counts down to zero)
							6 - An edge/interruptFlag was captured during the WDTPeriod
				*/
				if (sf == 0){
					if (port == 1){
						if ((P1IN & bit) == bit){
							PinAve[i] = 1;
						}else{
							PinAve[i] = 0;
						}
					}
					if (port == 2){
						if ((P2IN & bit) == bit){
							PinAve[i] = 1;
						}else{
							PinAve[i] = 0;
						}
					}
				}
				if (sf == 1){ // Timing Measurement

					//if a trigger occurred
					if(TA1CCTL1 && CCIFG){
						TA1CCTL1 &= ~(CCIFG + COV);
						if(TA1CCR1 != 0xFFFF){
							//multiply the number of TAR roll overs times 2^16 plus the difference between the trigger TAR values
							l = uiTARRO * 65535;
							//determine if the first triggers should be subtracted from the second
							//or if we should subtract the difference of the two from 2^16
							if(uiTARRO > 0){
								l += (65535 - TA1CCR1) + uiCCR1;
							}else{
								l += TA1CCR1 - uiCCR1;
							}
							// record the TAR for the next trigger calculation
							uiCCR1 = TA1CCR1;
							uiTARRO=0; // reset the roll over counter

							// store l in a variable as the current timing
							// the timer divide by 8 is a >> 3 so multiply by << 3 to get to microseconds
							// divide by 1024 (>>10) to get result in milliseconds (error is 2.4%)
							// resulting >> is 7
							// >> 11 is what works according to the oscilloscope (need to figure this out why my math is wrong)
							unsigned int temp_iCyclesLast = l >> 11;
							//Create a brick wall filter.  Any timing less than the filter value is set equal to the filter value
							if (*FlashTimingFilter != 0xFFFF){
								if(temp_iCyclesLast >= *FlashTimingFilter)
									iCyclesLast = temp_iCyclesLast;
							}else{
								iCyclesLast = temp_iCyclesLast;
							}
							// calculate a moving average
							if (PinAve[i] == 0)
								PinAve[i] = iCyclesLast;
							PinAve[i] = MovingAveFilter(PinAve[i],iCyclesLast,f);

							if (iCyclesLast > TimingMax)
								TimingMax = iCyclesLast;
							if (iCyclesLast < TimingMin)
								TimingMin = iCyclesLast;

							TA1CCR1 = 0xFFFF;
						}else{
							uiTARRO++;
						}
					}

				}
				if (port == 1){
					if(sf == 2 || sf == 3){ // Filter High
						if ((P1IN & bit) == bit){ //The pin has been high since the last check
							if (PinAve[i] < DI->Filter)
								PinAve[i]++;}
						else{
							if (PinAve[i] > 0 && sf == 3)
								PinAve[i]--;
						}
					}
					if(sf == 4 || sf == 5){ // Filter Low
						if ((P1IN & bit) == 0){ //The pin has been low since the last check
							if (PinAve[i] < DI->Filter)
								PinAve[i]++;}
						else{
							if (PinAve[i] > 0 && sf == 5)
								PinAve[i]--;
						}
					}
				}
				if (port == 2){
					if(sf == 2 || sf == 3){ // Filter High
						if ((P2IN & bit) == bit){
							if (PinAve[i] < DI->Filter)
								PinAve[i]++;
						}else{
							if (PinAve[i] > 0 && sf == 3)
								PinAve[i]--;
						}
					}
					if(sf == 4 || sf == 5){ // Filter Low
						if ((P2IN & bit) == 0){ //The pin has been low since the last check
							if (PinAve[i] < DI->Filter)
								PinAve[i]++;}
						else{
							if (PinAve[i] > 0 && sf == 5)
								PinAve[i]--;
						}
					}
				}
				WDTCount[i] = 0;

			}else if(*(FlashPortConfig + i)==1){		// Digital Output
				DO = (void *)(Flash_ptrC + StructSizeSegC * i);
				struct Threshold *T = (void *)(Flash_ptrD + StructSizeSegD * i);
				state = DO->StatePinPort>>5 & 0x01;
				inport = DO->StatePinPort & 0x03;
				inpin = (DO->StatePinPort >> 2) & 0x07;
				sf = DO->subFunction;
				unsigned int ut = T->Upper;
				unsigned int lt = T->Lower;
				unsigned char lh = DO->Hysteresis;
				unsigned char uh = DO->Hysteresis;
/*				0 - Control by Command Only
						Initial state set by 'State' parameter
				1 - Control by the average value of a Digital Input Pin
				2 - Use Thresholds with A2D Input Port
						Initial state set by 'State' parameter
						state between thresholds is set by the 'State' parameter
						Hysteresis is applied to Upper and Lower Thresholds
				3 - Use Hysteresis only with Upper Threshold
						Initial state set by 'State' parameter
				4 - Use hysteresis only with Lower Threshold
						Initial state set by 'State' parameter
				5 - (Port 2 only) pulse (50000 cycles) instead of state high (use with latching relay)
					the pulse is opposite the state value
					Hysteresis not used
					pulse when upper threshold is reached (occurs once each pin period) set to 1023 to disable
					pulse when lower threshold is reached (occurs once each pin period) set to 0 to disable
				6 - Same as function 5, but adding memory so that a relay pulse will only occur once
					hysteresis is used
				7 - Control by Command Only -- but pulse output
						Initial state set by 'State' parameter
*/
				if(port==1){
					if (sf == 1){
						unsigned int value = (unsigned int) GetAdjustedValue((inport-1)*8+inpin);
						PortPinOutState(1,pin,(value > 0));
//						if (value > 0){
//							P1OUT |= bit;}
//						else{
//							P1OUT &= ~bit;}
					}
					if (sf == 2 || sf == 3 || sf == 4){
						if (sf == 3)
							lh = 0;
						if (sf == 4)
							uh = 0;
						if (inport == 0){
							if ((AveTemp <= ut || AveTemp >= ut+lh) && (AveTemp <= ut-uh || AveTemp >= ut)){ // Hysteresis
								if (AveTemp > lt && AveTemp < ut){
									PortPinOutState(1,pin,state);
//									if (state){
//										P1OUT |= bit;}
//									else{
//										P1OUT &= ~bit;}
								}else{
									PortPinOutState(1,pin,~state);
//									if (!state){
//										P1OUT |= bit;}
//									else{
//										P1OUT &= ~bit;}
								}
							}
						}else{
							unsigned int value = (unsigned int) GetAdjustedValue((inport-1)*8+inpin);
							if ((value <= lt || value >= lt+lh) && (value <= ut-uh || value >= ut)){ // Hysteresis
								if (value > lt && value < ut){
									PortPinOutState(1,pin,state);
//									if (state){
//										P1OUT |= bit;}
//									else{
//										P1OUT &= ~bit;}
								}else{
									PortPinOutState(1,pin,~state);
//									if (!state){
//										P1OUT |= bit;}
//									else{
//										P1OUT &= ~bit;}
								}
							}
						}
					}
				}
				if(port==2){
					if (sf == 1){
						unsigned int value = (unsigned int) GetAdjustedValue((inport-1)*8+inpin);
						if (value > 0){
							P2OUT |= bit;}
						else{
							P2OUT &= ~bit;}
					}
					if (sf == 2){
						unsigned int value = (unsigned int) GetAdjustedValue((inport-1)*8+inpin);
						if (value >= lt && value <= ut){
							PortPinOutState(2,pin,state);
//							if (state){
//								P2OUT |= bit;}
//							else{
//								P2OUT &= ~bit;}
						}else{
							PortPinOutState(2,pin,~state);
//							if (!state){
//								P2OUT |= bit;}
//							else{
//								P2OUT &= ~bit;}
						}
					}
					if (sf == 5 || sf == 6){
						unsigned int value = (unsigned int) GetAdjustedValue((inport-1)*8+inpin);
						char bExecute = 0;
						if(value < lt || value > ut){
							if(sf == 6){ // check whether to use memory and only create a pulse one time when reaching a threshold
								if (Port2DOmem & bit == bit){// if a pulse was created last time, then don't create a pulse this time
									bExecute = 0;
								}else{
									Port2DOmem |= bit; // if no pulse last time, then pulse this time and set memory to
									bExecute = 1;
								}
							}else{
								bExecute = 1;} // create a pulse every time
							if(bExecute){
								PortPinOutState(2,pin,~state);
								__delay_cycles (50000);
								PortPinOutState(2,pin,state);
								PinAve[i] = 1; // indicate a pulse was created
							}
						}else{//if we are within the threshold values (and the hysteresis)
							if (sf == 6 && (value > lt+lh || value < ut-uh)) // Hysteresis
								Port2DOmem &= ~bit; // reset the memory bit
						}
					}
				}
				WDTCount[i] = 0;
			}
			else if(*(FlashPortConfig + i)==2){	// A2D Input
				M = (void *)(Flash_ptrC + StructSizeSegC * i);
				p = M->PointsAverage;
				nv = (M->DecimalPlacesVref >> 6) & 0x03;
				pv = (M->DecimalPlacesVref >> 4) & 0x03;
				if(WDTCount[i] < (*(FlashWDTPeriod + i)+(M->ArgPinPort>>5))){
					char pwrport = M->ArgPinPort & 0x03;
					char pwrpin = (M->ArgPinPort >> 2) & 0x07;
					PortPinOutState(pwrport,pwrpin,1);
//					if(pwrport == 1){
//						P1OUT |= 1<<pwrpin;
//					}else{
//						P2OUT |= 1<<pwrpin;
//					}
					WDTCount[i]++;
				}else{
					Single_Measure(pin * 0x1000u, pv);
					unsigned long adcmem = ADC10MEM;
					AdjustPort1Multiplier(pin,&PinAve[i],&adcmem);
					PinAve[i] = MovingAveFilter(PinAve[i],adcmem,p);

					WDTCount[i] = 0;
					if(M->ArgPinPort > 0){
						char pwrport = M->ArgPinPort & 0x03;
						char pwrpin = (M->ArgPinPort >> 2) & 0x07;
						PortPinOutState(pwrport,pwrpin,0);
//						if(pwrport == 1){
//							P1OUT &= ~(1<<pwrpin);
//						}else{
//							P2OUT &= ~(1<<pwrpin);
//						}
					}
				}
			}else if(*(FlashPortConfig + i)==3){	// Capacitive Measurement
				if (CapActiveIndex == 0)
					CapActiveIndex = i;
				if (CapActiveIndex == i){
					M = (void *)(Flash_ptrC + StructSizeSegC * i);
//					char nv = M->DecimalPlacesVref >> 6;
//					char pv = (M->DecimalPlacesVref >> 4) & 0x03;
					chport = M->ArgPinPort & 0x03;
					chpin = (M->ArgPinPort >> 2) & 0x07;
					arg = (M->ArgPinPort >> 5) & 0x01;

					WDTCount[i]++; // the counter is used during the measurement so it should continue incrementing

					if (statCap == 'I' && !bTimerA0inuse){
						bTimerA0inuse = 1;
						//Initialize
							//Setup Timer connect Comparator input to timer, Cap should be charged
							//Start discharge
							InitializeCompTimer(bit,chport,chpin);

						statCap = 'D'; // Discharge
						WDTCount[i] = *(FlashWDTPeriod + i); //reset counter to period and estimate the maximum number of additional periods until an error is declared
						errcntCap = WDTCount[i] + 50;
						uiCapTARRO = 0;
						lastTA0R = 0;
					}
					if (statCap == 'D'){
						//Track running status
						//Check for double trips
						if (TA0CCTL1 & COV){
							//Comparator overflow COV (comparator interrupted twice, which should not happen), if so, discard measurement, set error
							statCap = 'C';
							errCode = 'O';
							TA0CCTL1 &= ~CCIFG;
						}
						//Check for timer overflow, and count as 0xFFFF
						if (TA0CCTL1 & CCI){
							TA0CCTL1 &= ~(CCIFG + COV);
							//Check for comparator trip -- discharge to 0.25Vcc
							//calculate time
							l = uiCapTARRO * 0xFFFF;
							l += TA0CCR1;
							l = l << 1;
							p = M->PointsAverage;
							if(l>0x0FFFFFFF)
								l = 0x0FFFFFFF;//limit so that is doesn't truncate when shifted left 4 places
							AdjustPort1Multiplier(pin,&PinAve[i],&l);
							PinAve[i] = MovingAveFilter(PinAve[i],l,p);

							statCap = 'C'; // Complete
						}else{
							if(TA0R < lastTA0R) // record the number of timer loops
								uiCapTARRO++;
							lastTA0R = TA0R;
						}

						//when measurement complete statHum = 'C';
						//Check for error condition
						if (WDTCount[i] >= errcntCap){
							statCap = 'C';  //move on to the next step because the comparator never tripped
							errCode = 'H';
						}
					}
					if (statCap == 'C'){
						//Count WDT periods to wait for capacitor to recharge before making more measurements
						TA0CTL = MC_0;		// Stop Timer
						CACTL1 &= ~CAON;
						CAPD = 0;
						PortPinOutState(chport,chpin,1);
						if (arg == 1){
							P1DIR |= bit;
							P1OUT |= bit;
						}
						cntCapRecharge = WDTCount[i] + 6;
						statCap = 'R'; // Re-charge
					}
					if (statCap == 'R' && WDTCount[i] >= cntCapRecharge){
						statCap = 'I'; // Initialize
						bTimerA0inuse = 0;
						WDTCount[i] = 0;
						CapActiveIndex = 0;
						TA0CTL = MC_0;
					}
				}
			}else if(*(FlashPortConfig + i)==4){	// 60Hz Wave Measurement
				if (WaveActiveIndex == 0)
					WaveActiveIndex = i;
				if (WaveActiveIndex == i){
					M = (void *)(Flash_ptrC + StructSizeSegC * i);
					struct Measurement *M2 = (void *)(Flash_ptrC + StructSizeSegC * ((M->ArgPinPort>>2) & 7));
//					nv = M->DecimalPlacesVref;
//					pv = M->DecimalPlacesVref;
					if (M->ArgPinPort>>5 == 0){
						if (statWave = 'I' && !bTimerA0inuse){
							bTimerA0inuse = 1;
							bNoComm = 1;
							GetWaveAverage(i,M->ArgPinPort>>2&7,M->PointsAverage,M2->PointsAverage);
							bTimerA0inuse = 0;
							statWave = 'C';
						}
						if (statWave == 'M'){
							if ((TA1CTL & MC_0)){       // Wait for timer to be turned off
								statWave = 'C';
								bTimerA0inuse = 0;
								//Turn off ADC
							}
						}
						if (statWave == 'C'){
							if (bNoComm){
								if(PinAve[i] == 0)
									PinAve[i] = lWaveAve;
								//AdjustPort1Multiplier(pin,&PinAve[i],&lWaveAve);
								PinAve[i] = MovingAveFilter(PinAve[i],lWaveAve,M->PointsAverage);
								char voltpin = (M->ArgPinPort>>2) & 7;
								if(PinAve[voltpin] == 0)
									PinAve[voltpin] = lVoltAve;
								//AdjustPort1Multiplier(voltpin,&PinAve[voltpin],&lVoltAve);
								PinAve[voltpin] = MovingAveFilter(PinAve[voltpin],lVoltAve,M2->PointsAverage);
							}
							WaveActiveIndex = 0;
							WDTCount[i] = 0;
							statWave = 'I';
						}
					}else{WaveActiveIndex = 0;}
				}
			}
    	}
    }

    cntAveTemp++;
	if(cntAveTemp >= *FlashTempPeriod){
		cntAveTemp = 0;
		Single_Measure(INCH_10, 1);
		if (AveTemp == 0)
			AveTemp = ADC10MEM;
		AveTemp = MovingAveFilter(AveTemp,ADC10MEM,*FlashTempPointsAverage);
	}
	if (StartupDelay != 0){
		StartupDelay--;
		if(StartupDelay == 1){
			IE2 |= UCA0RXIE;			// Enable RX Interrupt
		}
	}
	WDTActive = 0;
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
//	ADCValue = ADC10MEM;			// Saves measured value.
//	ADCDone = 1;  			// Sets flag for main loop.
//	__bic_SR_register(SCG0+CPUOFF); //LPM1 + Interrupts
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{}

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
	// Use as Timing Trigger
	if(P2IFG & BIT2){
		P2IE &= !BIT2;					// For de-bounce, disable Interrupts for Trigger Input
		if (iTriggers == 0 && iCycles == 0){
			TA1R = 0;
			TA1CTL &= ~TAIFG;
			TA1CTL |= MC_1 + TAIE;				// Count up to CCR0
			iTriggers += 1;				// Increment the trigger counter
		}else{
//			iTriggers += 1;				// Increment the trigger counter
			iCyclesLast = iCycles;		// The last time there was a real measurement between two triggers
//			if (iCycles > iCyclesMax){iCyclesMax = iCycles;}
//			if (iCycles < iCyclesMin && iCycles != 0){iCyclesMin = iCycles;}
//			iCyclesSum += iCycles;		// Add the count to the sum
			iCycles = 0;				// Re-initialize the Cycle Counter
//			if(iTriggers == 4){			// Average this number of Triggers
				TA1CTL &= ~(MC0 + MC1);				// Stop Counter
				TA1CCTL0 &= ~CCIFG;			// Clear Interrupt Flag
				TA1R = 0;				// Initialize counter
//				iCyclesAve = iCyclesSum >> 2;	// Divide by 4
//				iCyclesSum = 0;
				iTriggers = 0;
//			}
		}
		__delay_cycles (500);			// For de-bounce, wait 0.5ms
		P2IFG &= ~BIT2;					// Clear interrupt flag
		P2IE |= BIT2;					// Re-enable Interrupts for Trigger Input
	}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
	if(bMeasCap){
		if(!bCapOverflow || ucCapOverflow > 7){
			bCapTimer = 0;
			TA0CCTL0 &= ~CCIE;}
		ucCapOverflow++;
		TA0IV=0x00;  // clear interrupt vector flags
	}
	TA0IV=0x00;  // clear interrupt vector flags
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR (void)
{
	if(bMeasCap){
		TA0CCTL1 &= ~CCIE;				// Prevent interrupt from CAON
//		CACTL1 &= ~CAON;
		TA0CTL = MC_0;		// Stop Timer
		TA0IV=0x00;  // clear interrupt vector flags
		uiCompTimer = TA0CCR1;
		bCapTimer = 0;
	}
	TA0IV=0x00;  // clear interrupt vector flags
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR (void)
{}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR (void)
{
	if(bTriggerCounter){
//		if((P1OUT & BIT0) == BIT0){P1OUT &= ~BIT0;}		// For Timer verification use oscilloscope
//		else{P1OUT |= BIT0;}		// For Timer verification is oscilloscope
		CCTL0 &= ~CCIFG;			// Clear Interrupt Flag
		iCycles += 1;
		if(iCycles == 0xA){ 	// Reset after 10 seconds
			TACTL &= ~(MC0 + MC1);		// Stop Counter
			iCycles = 0;		// Reset Cycle counter
			iTriggers = 0; 		// Reset the number of Triggers
//			iCyclesSum = 0;		// Reset the Cycle Average
//			P1OUT &= ~BIT0;		// For Timer verification use oscilloscope
		}
	}
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	IE2 &= ~UCA0RXIE;			// Disable RX Interrupt
	bNoComm = 0;
	CmdBuf[++iCmd] = UCA0RXBUF;
	// Discard characters until our ID found, then position it as the first character
	if (CmdBuf[0] != ID && CmdBuf[0] != '!'){
		iCmd=-1;
	}else{
		//Check for a Carriage Return and run command if found
		if ((CmdBuf[0] == ID || CmdBuf[0] == '!') && (CmdBuf[iCmd] == 0x0D)){
			if (((CmdBuf[3] == ':') || (CmdBuf[4] == ':')) | (iCmd == 3))
				ExecuteCommand();
			iCmd=-1;  //Reset Cmd Buffer index
		}else{
			//Check for overflow and reset Cmd Buffer index
			if (iCmd == 31) iCmd = -1;
		}
	}
	IE2 |= UCA0RXIE;		// Enable RX Interrupt
}

#pragma vector=COMPARATORA_VECTOR
__interrupt void ComparatorA_ISR(void)
{

}

/* Initialize non-used ISR vectors with a trap function */
#pragma vector=NMI_VECTOR,USCIAB0TX_VECTOR
__interrupt void ISR_trap(void)
{
  // the following will cause an access violation which results in a PUC reset
  WDTCTL = 0;
}

void SendOKNO(char PF){
	if(boolResetNow){//reload SendBuf because it is sometimes stepped on by info segment programming
		if (SendBuf[0] != '!'){
			if (*FlashID > 32 && *FlashID < 126){
				SendBuf[0] = *FlashID;
			}else{
				SendBuf[0] = ' ';
			}
			iSend = 0;
		}
	}
	//Pass = 1, Fail = 0
	if(PF){
		SendBuf[++iSend]='O';
		SendBuf[++iSend]='K';
	}else{
		SendBuf[++iSend]='N';
		SendBuf[++iSend]='O';
	}
}

void PortPinOutState(unsigned char Port, unsigned char Pin, char State){
	if (Port == 1){
		if (State)
			P1OUT |= 1 << Pin;
		else
			P1OUT &= ~(1 << Pin);
	}
	if (Port == 2){
		if (State)
			P2OUT |= 1 << Pin;
		else
			P2OUT &= ~(1 << Pin);
	}
}

char ProgramFlashInfoSegment(char *ptrDestSeg,char *ptrDestAddr,char *ptrSource,char NumItems){
	//The NumItems of the array ptrSource will be written starting at ptrDestAddr in the segment ptrDestSeg
		//ptrSource points to an array of length NumItems
		//ptrDestSeg is the address of the information segment to be written to
		//ptrDestAddr is the starting address in the ptrDestSeg segment where the ptrSource array will be written to
	char arrSeg[64];
	char i;
	char n;
	char *d;
	char pf = 1;

	IE1 &= ~WDTIE;

	// Copy Destination Segment to Ram
	d = ptrDestSeg;
	for (i=0;i<64;i++){
		arrSeg[i] = *d++;
	}
	// Handle LOCKA for Information Segment A
//	if(ptrDestSeg == Flash_ptrA){
//		if((FCTL3 & LOCKA) == LOCKA){
//			FCTL3 = FWKEY + LOCKA;
//		}
//	}
	// Erase Segment
	FCTL1 = FWKEY + ERASE;                    // Set Erase bit
	FCTL2 = FWKEY + FSSEL_2 + FN1;
	FCTL3 = FWKEY;                            // Clear Lock bit
	*ptrDestSeg = 0;                           // Dummy write to erase Flash segment

	FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

	// Write Ram back to Destination Segment, but use Source data for NumItems when DestSeg matches DestAddr
	d = ptrDestSeg;
	n=0;
	for (i=0;i<64;i++){
		if (d >= ptrDestAddr && n < NumItems){
			*d++ = *(ptrSource + n++);
		}else{
			*d++ = arrSeg[i];
		}
	}

	FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY + LOCK;                     // Set LOCK bit

// Verify data
	d = ptrDestSeg;
	n=0;
	for (i=0;i<64;i++){
		if (d >= ptrDestAddr && n < NumItems){
			if(*d++ != *(ptrSource + n++))
				pf = 0;
		}else{
			if(*d++ != arrSeg[i])
				pf = 0;
		}
	}

	IE1 |= WDTIE;
	return pf;
}

void MeasCompTimer(unsigned char ucBit)
{
	//  ucBit == BIT0 then P2.0 will charge the cap and P1.5 is configured as CA5 to measure the slope
	//  ucBit == BIT5 then P2.5 will charge the cap and P1.6 is configured as CA6 to measure the slope
	bMeasCap = 1;
//	unsigned char i;
//	unsigned char c;
//	unsigned long l;
// Initialize *****************************************************************
	bCapOverflow = 0;
//	i = 0;
	bCapTimer = 1;
	uiCompTimer = 0;
	TA0CTL = MC_0;		// Stop Timer
	TA0R = 0;			// Set counter to zero
	TA0CCTL1 = CM_2 + CCIS_1 + CAP;	  // Capture Falling Edge, input from CCI1B, Capture mode
	TA0CCTL0 &= ~CCIE;				// Disable interrupts from Timer 0 CC 0 period register

	if(ucBit == BIT0){
		CACTL2 = P2CA3 + P2CA1 + CAF; // Input CA5, Output Filter ON
		CAPD = CAPD5;}				// Disable Comparator buffer CAPD5
	if(ucBit == BIT5){
		CACTL2 = P2CA3 + P2CA2 + CAF; // Input CA6, Output Filter ON
		CAPD = CAPD6;}				// Disable Comparator buffer CAPD6

//	CACTL1 = CAEX + CAREF_2 + CAON;
//	__delay_cycles (30000);

	// Time the capacitor charge to 0.5*VCC
//	TA0CCTL1 |= CCIE;
//    TA0CTL = TASSEL_2 + MC_2;
//    P2OUT |= ucBit;		// Charge Capacitor
    ucCapOverflow = 0;		// reset this counter
    bCapOverflow = 1;	// enable overflow counting
	//while(bCapTimer);
	TA0CCTL1 &= ~CCIE;				// Prevent interrupt from CAON
	TA0CTL = MC_0;		// Stop Timer
	// Wait for capacitor to finish charging to VCC
//	c = 10;
//	if(ucCapOverflow > 0){
//		l = c * ucCapOverflow;
//		l = l >> 4;
//		c = c * ucCapOverflow + l ;
//		uiCompTimer = 0xFFFF;
//	}
//	if(uiCompTimer < 15)
//		uiCompTimer = 15;
//	for(i=0;i<c;i++){
//		bCapTimer = 1;
//		TA0CCR1 = 0;
//		TA0CTL = MC_0;		// Stop Timer
//		TA0R = 0;
//		TA0CCR0 = uiCompTimer;
//		TA0CCTL0 = CM_0 + CCIE;
//		TA0CTL = TASSEL_2 + MC_1;
//		while(bCapTimer);
//	}
//	TA0CCTL1 &= ~CCIE;				// Prevent interrupt from CAON
//	TA0CTL = MC_0;		// Stop Timer
	TA0CCR0 = 0xFFFF;	// initialize compare register to max value
	bCapTimer = 1;	// enable this software function in the timer interrupt
	uiCompTimer = 0;	// initialize
	// Time the discharge to 0.25*VCC
//	TA0R = 0;
	CACTL1 = CAEX + CAREF_1 + CAON;	// Comparator Exchange (exchange inputs, invert output); 0.25*Vcc; turn ON
//	__delay_cycles (30000);
	TA0CCTL1 = CM_1 + CCIS_1 + CAP + CCIE;  // Capture Rising Edge; CCI1B input; Capture Mode; CC Interrupt Enable
    TA0CTL = TASSEL_2 + MC_2;// + TAIE;  SMCLK; Continuous Mode, count up to 0xFFFF
    P2OUT &= ~ucBit;		// Discharge Capacitor
    ucCapOverflow = 0; 		// initialize counter
    bCapOverflow = 1;	// initialize function in timer interrupt
// Check for complete flag(s) and process results ***************************************************
    while(bCapTimer);
    bCapOverflow = 0;	// disable function in TIMER0_A0_ISR timer interrupt
    bMeasCap = 0;		// disable function in TIMER0_A1_ISR timer interrupt
    if(ucBit == BIT0){
    	uiCapacitiveHumidity=uiCompTimer;
    	ucCapacitiveHumidity=ucCapOverflow;
    }
    if(ucBit == BIT5){
    	uiCDS=uiCompTimer;
    	ucCDS=ucCapOverflow;
    }
// Block next measurement until capacitor is charged  **********************************************
    P2OUT |= ucBit;		// Charge Capacitor
}

void GetWaveAverage(unsigned char wavePin, unsigned char voltPin, unsigned char wavePave, unsigned char voltPave)
{
	unsigned long ADCSum;			// Accumulator and result for ADC averaging
//	unsigned char x;
	unsigned char i;
	unsigned long VoltSum;
	unsigned long VoltOffSum = 0;
	unsigned long WaveOffSum = 0;
	char FailedTiming = 0;
	unsigned long calcVoltOff = VoltOff >> 5; // divide by 32
	unsigned long calcWaveOff = WaveOff >> 5; // divide by 32
	//First measure DC Offset on INCH_4
//	ADC10CTL0 &= ~ENC;				// Disable ADC
//	ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON;// + REFON;// + ADC10IE;	// Initialize ADC
//	ADC10CTL1 = ADC10SSEL_3 + INCH_4;				// Set channel, SMCLK
//	unsigned char x;
//	unsigned char i;
//	x = 0;
//	DCOffset = 0;
//	for(x=1;x<=4;x++){
//		ADC10CTL0 |= ENC + ADC10SC;             	// Enable and start conversion
//		i=0;
//		while(!(ADC10CTL0&ADC10IFG) && (i++ < 254)){__delay_cycles (10);}
//		DCOffset = DCOffset + ADC10MEM;
//	}
//	DCOffset = DCOffset >> 2;
//	DCOffset = 512;
	ADC10CTL0 &= ~ENC;				// Disable ADC
	ADC10CTL0 = SREF_0 + ADC10SHT_0 + ADC10ON;// + REFON;// + ADC10IE;	// Initialize ADC
	ADC10CTL1 = wavePin*0x1000u;//INCH_3;// + ADC10SSEL_3;				// Set channel, SMCLK

	//__delay_cycles (10000);					// Delay to allow Ref to settle

	cWave = 0;
	lWaveSum = 0;
	lVoltSum = 0;
	TA1CTL = MC_0;		// Stop Timer
	TA1R = 0;					// Initialize counter
	TA1CCR0 = 521;//microseconds		// Set Count (32 interrupts per 1/60th of a second)
//	TA1CCR0 = 278;//microseconds		// Set Count (60 interrupts per 1/60th of a second)
//	TA1CCR0 = 69;//microseconds		// Set Count (240 interrupts per 1/60th of a second)
	TA1CCTL0 = CM_0;
	TA1CCTL0 &= ~CCIFG;
//	TA1CCTL0 |= CCIE;
	TA1CTL = TASSEL_2 + MC_1;
	TA1CTL &= ~TAIFG;
//	TA1CTL |= TAIE;
//	while(TA1CTL & MC_1)		// Wait for timer to be turned off
//	DebugBufIndex = 0;
	uiWaveMax = 0;
	uiWaveMin = 0xFFFF;
//	P1OUT |= BIT0;
	while(cWave < 32){
		if(TA1CCTL0 && CCIFG){
			TA1CCTL0 &= ~CCIFG;			// Clear Interrupt Flag
			// Take ADC reading

		//	x = 0;
			ADCSum = 0;
			VoltSum = 0;
		//	for(x=1;x<=4;x++){
				ADC10CTL0 |= ENC + ADC10SC;             	// Enable and start conversion
				i=0;
				while(!(ADC10CTL0&ADC10IFG) && (i++ < 254)){__delay_cycles (1);}
				ADCSum += ADC10MEM;

				// configure for voltage channel
				ADC10CTL0 &= ~ENC;				// Disable ADC
				ADC10CTL1 = voltPin*0x1000u;//INCH_4;// + ADC10SSEL_3;				// Set channel, SMCLK
//				ADC10CTL0 = SREF_0 + ADC10SHT_0 + ADC10ON;// + REFON;// + ADC10IE;	// Initialize ADC
				ADC10CTL0 |= ENC + ADC10SC;             	// Enable and start conversion
				i=0;
				while(!(ADC10CTL0&ADC10IFG) && (i++ < 254)){__delay_cycles (1);}
				VoltSum += ADC10MEM;

				//reconfigure for wave channel
				ADC10CTL0 &= ~ENC;				// Disable ADC
				ADC10CTL1 = wavePin*0x1000u;//INCH_3;// + ADC10SSEL_3;				// Set channel, SMCLK
//				ADC10CTL0 = SREF_0 + ADC10SHT_0 + ADC10ON;// + REFON;// + ADC10IE;	// Initialize ADC

		//	}
		//	ADCSum = ADCSum >> 2;
			ADCSum = ADCSum<<4;
			VoltSum = VoltSum<<4;
			VoltOffSum += VoltSum;
			WaveOffSum += ADCSum;
			// Subtract DC offset from waveform and invert (full-wave rectify) the negative portion of the waveform (the values below the DCOffset)
//			if(ADCSum > uiWaveMax)
//				uiWaveMax = ADCSum;
//			if(ADCSum < uiWaveMin)
//				uiWaveMin = ADCSum;
			if(ADCSum > calcWaveOff){
				ADCSum = ADCSum - calcWaveOff;}
			else{
				ADCSum = calcWaveOff - ADCSum;}

			if(VoltSum > calcVoltOff){
				VoltSum = VoltSum - calcVoltOff;}
			else{
				VoltSum = calcVoltOff - VoltSum;}

			lVoltSum += VoltSum;
			lWaveSum += ADCSum;
//			lWaveSum += ADCSum;	//for testing

			if(TA1CCTL0 && CCIFG)
				FailedTiming = 1;

			cWave++;
		}
	}

	if (FailedTiming){
		lVoltSum = 0;
		lWaveSum = 0;
	}
//	P1OUT &= ~BIT0;
	// Calculate the average of the values if there was no serial communication that disrupted the timing of the measurement
	if (bNoComm){
		lWaveAve = lWaveSum >> 5; // divide by 32
		lVoltAve = lVoltSum >> 5; // divide by 32
		VoltOff =  MovingAveFilter(VoltOff,VoltOffSum,voltPave);
		WaveOff = MovingAveFilter(WaveOff,WaveOffSum,wavePave);
	}
}

void Single_Measure(unsigned int chan, unsigned char Reference)
{
	unsigned int i;
	/*Reference: 	3 = 3.3V (VCC)
	 * 				2 = 2.5V
	 * 				1 = 1.5V	*/
	ADC10CTL0 &= ~ENC;				// Disable ADC
	if(Reference == 3)
		ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE + REFON; // keeping the reference on even when it is not used to keep it stabilized
	if(Reference == 2)
		ADC10CTL0 = SREF_1 + ADC10SHT_3 + ADC10ON + ADC10IE + REFON + REF2_5V;
	if(Reference == 1)
		ADC10CTL0 = SREF_1 + ADC10SHT_3 + ADC10ON + ADC10IE + REFON;
	ADC10CTL1 = ADC10SSEL_3 + chan;				// Set 'chan', SMCLK
//	if(Reference < 3)
//		__delay_cycles (30000);					// Delay to allow Ref to settle
	ADC10CTL0 |= ENC + ADC10SC;             	// Enable and start conversion
	i=0;
	while(i != 254){
		if((ADC10CTL0 & ADC10IFG)== ADC10IFG){
			i=253;
		}
		i++;
		__delay_cycles (8);
	}
	// get result from ADC10MEM
	ADC10CTL0 &= ~ENC;				// Disable ADC
	ADC10CTL0 = SREF_1 + ADC10SHT_3 + ADC10ON + REFON;  // try to keep the voltage reference on and stabilized
}

// Disable other trigger sources before enabling this trigger source
void EnableTriggerCounting(void){
	iCycles = 0;				// Reset timer cycle counter
	iTriggers = 0;				// Reset the number of triggers
	TA1CTL = TASSEL_2 + MC_0;	// SMCLK, Turn off Timer
//	TA1R = 0;					// Initialize counter
	TA1CTL &= ~TAIFG;
	TA1CTL = TACLR + TASSEL_2 + MC_2 + ID_3;
//	TA1CCR1 = 1000-20;				// Set Count - 1000 for 1ms resolution (increase or decrease for calibration)
	bTriggerCounter = 1;
	P2SEL2 |= BIT2;				// Interrupt Timer from this IO Port
	P2IFG &= ~BIT2;					// Clear interrupt flag
//	P2IE |= BIT2;				//Enable Interrupts for Trigger Input
	TA1CCR1 = 0xFFFF;
	TA1CCTL1 |= (CM_1 + CCIS_1 + SCS + CAP + OUTMOD_0);			// Set to CCI1B
	TA1CCTL1 &= ~(OUT + COV + CCIFG);			// Clear Interrupt Flag before enabling interrupts
//	TA1CCTL1 |=  CCIE;				// Enable Capture/Compare Interrupt
	// Disable Trigger Counting by sending the following commands
	//  	P2IE &= !BIT2;				// Disable Interrupts for Trigger Input
	//		bTriggerCounter = 0;
}

void TransmitDecimalFromLong(unsigned long value, char DecimalPlaces)
{//Code developed from: http://www.cs.uiowa.edu/~jones/bcd/decimal.html
	unsigned int d9;
	unsigned int d8;
	unsigned int d7;
	unsigned int d6;
	unsigned int d5;
	unsigned int d4;
	unsigned int d3;
	unsigned int d2;
	unsigned int d1;
	unsigned int d0;
	unsigned int q;
	unsigned int d5original;
	unsigned int d6original;

    d0 = value       & 0xF;
    d1 = (value>>4)  & 0xF;
    d2 = (value>>8)  & 0xF;
    d3 = (value>>12)  & 0xF;
    d4 = (value>>16)  & 0xF;
    d5 = (value>>20)  & 0xF;
    d6 = (value>>24)  & 0xF;
    d7 = (value>>28)  & 0xF;
    d5original = d5;
    d6original = d6;

/*		 d  9876543210
 *	2^32 = 	4294967296	8
 * 	2^28 = 	 268435456	7
 * 	2^24 = 	  16777216	6
 * 	2^20 = 	   1048576	5
 * 	2^16 = 	     65536	4
 * 	2^12 = 		  4096	3
 * 	2^8  =		   256	2
 * 	2^4  =			16	1
 * 	2^0  =			 1	0
 */
    d0 = 6*(d7 + d6 + d5 + d4 + d3 + d2 + d1) + d0;
    q = d0 / 10;
    d0 = d0 % 10;

    d1 = q + 5*d7 + 1*d6 + 7*d5 + 3*d4 + 9*d3 + 5*d2 + d1;
    q = d1 / 10;
    d1 = d1 % 10;

    d2 = q + 4*d7 + 2*d6 + 5*d5 + 5*d4 + 2*d2;
    q = d2 / 10;
    d2 = d2 % 10;

    d3 = q + 5*d7 + 7*d6 + 8*d5 + 5*d4 + 4*d3;
    q = d3 / 10;
    d3 = d3 % 10;

    d4 = q + 3*d7 + 7*d6 + 4*d5 + 6*d4;
    q = d4 / 10;
    d4 = d4 % 10;

    d5 = q + 4*d7 + 7*d6;
    q = d5 / 10;
    d5 = d5 % 10;

    d6 = q + 8*d7 + 6*d6 + d5original;
    q = d6 / 10;
    d6 = d6 % 10;

    d7 = q + 6*d7 + d6original;
    q = d7 / 10;
    d7 = d7 % 10;

    d8 = q;
    q = d8 / 10;
    d8 = d8 % 10;

    d9 = q;

    if(DecimalPlaces == 10){
    	SendBuf[++iSend]='0';
    	SendBuf[++iSend]='.';
    }
    if(d9!=0 || DecimalPlaces > 8)
        SendBuf[++iSend]=d9 + '0';								//Billions
    if(DecimalPlaces == 9)
    	SendBuf[++iSend]='.';
    if(d8!=0 || d9!=0 || DecimalPlaces > 7)
        SendBuf[++iSend]=d8 + '0';								//Hundred Millions
    if(DecimalPlaces == 8)
    	SendBuf[++iSend]='.';
    if(d7!=0 || d9!=0 || d8!=0 || DecimalPlaces > 6)
        SendBuf[++iSend]=d7 + '0';								//Ten Millions
    if(DecimalPlaces == 7)
    	SendBuf[++iSend]='.';
    if(d6!=0 || d9!=0 || d8!=0 || d7!=0 || DecimalPlaces > 5)
        SendBuf[++iSend]=d6 + '0';								//Millions
    if(DecimalPlaces == 6)
    	SendBuf[++iSend]='.';
    if(d5!=0 || d9!=0 || d8!=0 || d7!=0 || d6!=0 || DecimalPlaces > 4)
        SendBuf[++iSend]=d5 + '0';								//Hundred Thousands
    if(DecimalPlaces == 5)
    	SendBuf[++iSend]='.';
    if(d4!=0 || d9!=0 || d8!=0 || d7!=0 || d6!=0 || d5!=0 || DecimalPlaces > 3)
    	SendBuf[++iSend]=d4 + '0';								//Ten Thousands
    if(DecimalPlaces == 4)
    	SendBuf[++iSend]='.';
    if(d3!=0 || d9!=0 || d8!=0 || d7!=0 || d6!=0 || d5!=0 || d4!=0 || DecimalPlaces > 2)
    	SendBuf[++iSend]=d3 + '0';								//Thousands
    if(DecimalPlaces == 3)
    	SendBuf[++iSend]='.';
    if(d2!=0 || d9!=0 || d8!=0 || d7!=0 || d6!=0 || d5!=0 || d4!=0 || d3!=0 || DecimalPlaces > 1)
    	SendBuf[++iSend]=d2 + '0';								//Hundreds
    if(DecimalPlaces == 2)
    	SendBuf[++iSend]='.';
    if(d1!=0 || d9!=0 || d8!=0 || d7!=0 || d6!=0 || d5!=0 || d4!=0 || d3!=0 ||d2!=0 || DecimalPlaces > 0)
    	SendBuf[++iSend]=d1 + '0';								//Tens
    if(DecimalPlaces == 1)
    	SendBuf[++iSend]='.';
    SendBuf[++iSend]=d0 + '0';									//Ones
}

void InitializeCompTimer(unsigned char ucBit, unsigned char chport, unsigned char chpin)
{
	unsigned char chbit = 1<<chpin;
	TA0CTL = MC_0;		// Stop Timer
	TA0CCTL1 &= ~CCIE;				// Disable interrupts from Timer 0 CC 0 period register

	// Time the discharge to 0.25*VCC
	CACTL1 = CAREF_1 + CAON;	// 0.25*Vcc; turn ON
	if(ucBit == BIT5){
		P1DIR &= ~BIT5;
		CACTL2 = P2CA3 + P2CA1 + CAF; // Input CA5, Output Filter ON
		CAPD = CAPD5;}				// Disable Comparator buffer CAPD5
	if(ucBit == BIT6){
		P1DIR &= ~BIT6;
		CACTL2 = P2CA3 + P2CA2 + CAF; // Input CA6, Output Filter ON
		CAPD = CAPD6;}				// Disable Comparator buffer CAPD6
//	__delay_cycles (100);

	//CM_1 + CCIS_1 + SCS + CAP + OUTMOD_0
	TA0CCTL1 |= (CM_1 + CCIS_1 + SCS + CAP + OUTMOD_0);  // CCI1B input; Capture Mode
	TA0CCR1 = 0xFFFE;	// initialize compare register to max value minus 1
	TA0CTL &= ~TAIFG;
	TA0CCTL1 &= ~(OUT + COV + CCIFG);
	TA0CTL = TACLR + TASSEL_2 + MC_2 + ID_1; //SMCLK, Continuous Mode, Divide Clk by 2
//	TA0CCR1 = 0xFFFE;	// initialize compare register to max value minus 1
//	TA0CTL = TASSEL_2 + MC_2;// + TAIE;  SMCLK; Continuous Mode, count up to 0xFFFF
	if (chport == 1)
		P1OUT &= ~chbit;		// Discharge Capacitor
	if (chport == 2)
		P2OUT &= ~chbit;		// Discharge Capacitor
}

unsigned long GetAdjustedValue(char pin){
	return PinAve[pin] >> ((Port1Multiplier >> (4*pin)) & 0xF);
}

void AdjustPort1Multiplier(char pin, unsigned long *ave, unsigned long *value){
	unsigned long m = 4;
//	unsigned long m = (Port1Multiplier >> (4*pin)) & 0x0000000F;
//	unsigned char v = 0xFF;
//	unsigned char n = 0xFF;
//
//	if (*value > 0xFFFF){
//		n = 0;
//	}else{
//		if(*value > 0){
//			while(!(*value & (0x8000>>++v))){}  // find max left shift
//		}else{v=m;} // if the value is zero, don't adjust the average
//
//		if(*ave > 0){
//			while(!(*ave & (0x8000>>++n))){}  // find max left shift
//		}else{n=0;}
//	}
//	if(n>m){ // check if the average has not been left shifted as much as possible
//		*ave = *ave<<(n-m);
//		m += (n-m);
//	}
//	if(n<m){ // check if the average needs to be right shifted
//		*ave = *ave>>(m-n);
//		m -= (m-n);
//	}
//
//	if(v<m){	//check if the new value is an order of magnitude larger than the average value
//		*ave = *ave>>(m-v);		// the ave adjustment will track the adjustment of the latest value
//		m -= (m-v);
//	}

	*value = *value << m;  // adjust the value by the multiplier

	//Update the multiplier
//	unsigned long l = ~(0x0000000F << (4*pin));
	unsigned long l = 0xF;
	char shift = 4 * pin;
	l = l<<shift;
	l = ~l;
	Port1Multiplier &= l;
	Port1Multiplier |= (m << shift);
}

unsigned long MovingAveFilter(unsigned long ave, unsigned long value, unsigned char PowerOf2Samples)
{
	unsigned long a;
	// moving average = ave - ave/samples + value/samples
	// Don't average values during the startup delay -- let the sensors settle
	if (PowerOf2Samples > 0 && StartupDelay == 0){
		a = ave;
		a -= ave >> PowerOf2Samples;
		a += value >> PowerOf2Samples;
		if (PowerOf2Samples == 1){ // perform a 2 point average of the variables "ave" and "a"
			a -= a >> PowerOf2Samples;
			a += ave >> PowerOf2Samples;
		}
	}else{
		a = value;
	}
	return a;
}

unsigned long ConvertAdvCmdParameterFloatToHex(char CmdBufOffset, char MultipleOfTen){
	//parameter is between CmdBuf[5] + CmdBuffOffset and cCmd index
	unsigned char d = 0;
	unsigned long l = 0;
	unsigned long m = 0;
	unsigned char arrN[15];
	unsigned char n = 0;

	signed char i=5;
	i += CmdBufOffset;
	while(i<iCmd && (m<MultipleOfTen || MultipleOfTen==0)){
		if (CmdBuf[i] == '.'){
			if(MultipleOfTen==0)
				break;  // if rounding is desired, it could go here (evaluate CmdBuf[i+1] and adjust arrN[n]; but would also need to handle zero minus one and 9 plus one)
			d = i++;
		}else{
			if (d > 0){
				arrN[n++] = CmdBuf[i++]-'0';
				m++;
			}else if (d == 0){
				arrN[n++] = CmdBuf[i++]-'0';
			}
		}
	}
	//pad zeros
	while (m<MultipleOfTen){
		arrN[n++] = 0;
		m++;
	}
	l = arrN[n-1];
	m=1;
	for (i=n-2;i>=0;i--){
		l += arrN[i] * 10 * m;
		m *= 10;
	}
	return l;
}

/* Command List
CMDS
FV			Firmware Version
CS			get flash Configuration Status
ID			get ID from flash

TV			Temperature Value
TC			Temperature a2d Count
TP			Temperature Period
TA			Temperature points Average
TM			Temperature Multiplier
TO			Temperature Offset
PF			Pin Function (lists the function for all pins P1.0-7 and P2.0-7)
RC			Redundant Communication (show the status)
TX			P2.2 Timing Max (read the max count since the last read and reset the max value)
TN			P2.2 Timing Min (read the min count since the last read and reset the min value)
TF			P2.2 Timing Filter Value

PS:PP		Pin Setup (Port Pin)
PM:PP		Pin Multiplier (Port Pin)
PO:PP		Pin Offset (Port Pin)
PV:PP		Pin Value (Port Pin)
PC:PP		Pin Count (Port Pin)
PA:PP		Pin points Average (Port Pin)
UT:PP		Upper Threshold (Port Pin)
LT:PP		Lower Threshold (Port Pin)
SP:PP		Set Pin (Port Pin)
RP:PP		Reset Pin (Port Pin)
TP:PP		Toggle Pin (Port Pin)
BP:PP		Beep (or pulse) pin for 50ms (Port Pin)
CW:PP		Calculate Watts (Port Pin) of current sensor (presently, this is only apparent power, VA)

PRC:x		Program Redundant Communication x: 0=Off, 1=On
SCI:		Set Configuration to Inputs for all pins
PCS:		Program Configuration Status  (I,R,S) (Initialize, Run, Stop)  Any other value will Initialize Pins to Inputs
PID:		Program ID
RST:		ReSeT

PPF:pp,f			Program Pin Function: port pin function (function is enumerated type 0-7)
					(0=DigitalIn,1=DigitalOut,2=AnalogIn,3=Capacitive,4=60HzWave)
CPP:				Configure Port Pin (takes a string of parameters in hexadecimal)
PPP:pp,decimal		Program Pin Period, 0 to 65535
PTM:pp,decimal		Program Temperature Multiplier
PTO:pp,decimal		Program Temperature Offset
PTP:pp,decimal		Program Temperature Period
PTA:pp,decimal		Program Temperature points Average
PM1:pin,decimal		Program Multiplier port 1
PO1:pin,decimal		Program Offset port 1
PA1:pin,decimal		Program points Averaging port 1
PU2:pin,decimal		Program Upper Threshold port 2
PL2:pin,decimal		Program Lower Threshold port 2
PPC:PP,hex			Program Pin Count (Port Pin) and 32 bit hex value (0xFFFFFFFF) - used for testing and debugging
PTF:decimal			Timing Filter for P2.2 Timing Function, 16 bit integer value.  Timing values less than this value will be ignored and will reset the timer.

Presets
PRE:SET0			Configure internal temperature sensor.  This sensor will need it's offset calibrated.  The slope is very repeatable from IC to IC.
PRE:SET1			Capacitive Humidity P1.5,P2.0
PRE:SET2			CDS Light Sensor P1.6,P2.5
PRE:SET3			PIR Motion Sensor P2.1
PRE:SET4			Voltage Humidity Sensor P1.3, power on P2.3
PRE:SET5			Voltage Temperature Sensor P1.4, power on P2.4
PRE:SET6			Voltage Temperature Sensor P1.7, power on P2.7
PRE:SET7			Timing P2.2 (use for anemometer, configure Timing Filter if needed)

Presets configure the following items:
Pin Function 	(PPF:pp,0-7)
Pin Period	(PPP:pp,0-65535)
Parameters specific to Pin Function
	Port 1 only (only port 1 has access to the ADC)
		Analog Measurement parameters	(CPP:<character string of configuration parameters>)
		Multiplier and Offset		(PM1:p,<multiplier>),(PO1:p,<offset>)
		Averaging			(PA1:p,<power of 2 points>)
	Port 1 or 2
		Digital Input parameters	(CPP:<character string of configuration parameters>)
		Digital Output parameters	(CPP:<character string of configuration parameters>)
	Port 2 only (used to control a digital output when tied to an analog input pin)
		Threshold parameters		(PU2:p<upper threshold>,(PL2:p,<lower threshold>)

Configure Internal Temperature Sensor
PTM:0.7428
PTO:465.0
PTP:4
PTA:2 (power of 2 gives 4 (2^2) points of averaging)

Configure LED on P2.1
PPF:21,1
CPP:21000000

Configure Timing on P2.2
PPF:22,0 (this is not needed if the pin was never configured for a different function. The default is zero)
CPP:22110101
PPP:22,10
Optional: Program Timing Filter
	For anemometer: PTF:15


Future Features
	Hold mode for Digital Outputs
		This could be done with the existing subFunction (probably the best solution to limit confusion)
		It could be an override memory based function that would be cleared on reset
		It could be an override function but saved in flash

	Calculate Negative for internal temp sensor
	If first value is zero, then use a positive value so the averaging doesn't get stuck at zero.

 */


/* Unused code


		else if((CmdBuf[1] == 'C') && (CmdBuf[2] == 'C')){  // Calibration Count  	// NOT TESTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// index 4 is the calibration table index (0-9)
			unsigned int i = (CmdBuf[4] - '0');
			unsigned int low = *(FlashCalTable + i);
			unsigned char high = *(FlashCalTable + i) >> 16;
			TransmitExtendedDecimal(high,low,0);
		}

		else if((CmdBuf[1] == 'C') && (CmdBuf[2] == 'C')){  // Calibration Value  	// NOT TESTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// index 4 is the calibration table index (0-9)
			unsigned int i = (CmdBuf[4] - '0');
			unsigned int low = *(FlashCalTable + i) >> 20;
			TransmitExtendedDecimal(0,low,0);
		}

		else if((CmdBuf[1] == 'P') && (CmdBuf[2] == 'C') && ((CmdBuf[3] == 'C') || (CmdBuf[3] == 'V'))){	// Program Calibration Table (Count or Value)
			// NOT TESTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// format PCC:(0-9 table index),Count	(count can be 20 bits)
			// format PCV:(0-9 table index),Value	(value can be 12 bits)
			// data is stored in 32 bit long as (12 bit value || 20 bit count)
			char offset = CmdBuf[5] - '0';
			char PF = 0;
			if (CmdBuf[6] == ','){
				unsigned long cal = *(FlashCalTable + offset);
				unsigned long arg = ConvertAdvCmdParameterFloatToHex(2,0);
				if (CmdBuf[3] == 'C')
					cal = (cal & 0xFFF00000) + arg;
				if (CmdBuf[3] == 'V')
					cal = (cal & 0x000FFFFF) + arg << 20;
				PF = ProgramFlashInfoSegment(Flash_ptrB,(void *)(FlashCalTable + offset),(char *)cal,1);
				boolResetNow = 1;
			}
			SendOKNO(PF);
			boolResetNow = 1;
		}

unsigned long *FlashCalTable = (unsigned long *) 0x10C0;  // table can grow to 12 points

unsigned int InterpolateCalTable(unsigned long count);
unsigned int InterpolateCalTable(unsigned long count){	// NOT TESTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	char i;
	for(i=0;i<9;i++)
		if(((*(FlashCalTable+i) & 0xFFFFF) <= count) && (count <= (*(FlashCalTable+i+1) & 0xFFFFF)))
			break;
	//y = (y2-y1)/(x2-x1)(x-x1)+y1
	unsigned long x1 = *(FlashCalTable+i) & 0xFFFFF;
	unsigned long x2 = *(FlashCalTable+i+1) & 0xFFFFF;
	unsigned long y1 = *(FlashCalTable+i) >> 20;
	unsigned long y2 = *(FlashCalTable+i+1) >> 20;

	if(x2-x1 > 0){
		return (y2-y1)/(x2-x1)*(count-x1)+y1;
	}else{
		return 0xFFFF;}
}

 */
