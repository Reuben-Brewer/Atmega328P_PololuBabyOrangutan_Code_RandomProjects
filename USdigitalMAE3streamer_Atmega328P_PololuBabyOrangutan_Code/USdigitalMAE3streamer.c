#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define timer2_clock_prescalar 1024.0
#define TxRegularUpdatesFlag 1
#define send_every_byte_back_as_received_mode 0
#define echo_Rx_message 0
#define TCNT_ReadingClockCycleOffset 2 //It's takes some number of clock cycles to read and copy TCNT for the PWM measurement, so we're subtracting it ffrom the time stamp.
//////////////////////////////////////////////

//////////////////////////////////////////////
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> //Include this so that we don't get any warnings about the abs() "implicit declaration".
#include <math.h>
#include <ctype.h> //Necessary for isprint(). function of interest for checking if a character is ASCII is isprint()
#include <string.h> //for strlen()
#include "crc16_modified_by_Reuben.h"
//////////////////////////////////////////////

//////////////////////////////////////////////
void TimerOneInitForMAE3(void);

void TimerTwoInitMode4CTC_NoInterrupt(void);
unsigned int CalcTMR2mode4_OCRB_from_freq(float frequency);

void PinInit(void);

void setDebugLED_1(unsigned int value);
void setDebugLED_2(unsigned int value);
void setOScopeDitigalLine(unsigned int value);

void toggleDebugLED_1(void);
void toggleDebugLED_2(void);
void toggleOScopeDO(void);

void blinkDebugLED_2(int numberBlinks, int millisecondsBlink);

void SerialInit(void);
void SendSerialTxUpdate(void);

void externalInterrupt0Init(void);
void DisableINT0(void);
void EnableINT0(void);


//////////////////////////////////////////////

//////////////////////////////////////////////

volatile float main_loop_global_time = 0.0;
volatile float last_loop_time = 0.0;

volatile unsigned int debugState1 = 0;
volatile unsigned int debugState2 = 0;
volatile unsigned int debugStateOScope = 0;

volatile unsigned int TxCounter = 0;
volatile float debug_to_computer;

uint16_t CRC16_checksum_Tx = 0; //DON'T MAKE VOLATILE

volatile int FallingEdgeTimeStamp = 0;
volatile int EndOfCycleTimeStamp = 1;
volatile float ABSencoderFrequency = 0.0;
volatile float ABSencoderDutyCycle = 0.0;
volatile float ABSencoderAngle = 0.0;
volatile float Timer1Prescalar = -1.0;
volatile float x0 = 0; 
volatile float high0 = 0; 
volatile float low0 = 0; 

//////////////////////////////////////////////

//////////////////////////////////////////////
typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void send_serial_byte(unsigned char byte, int UpdateCRCflag, uint16_t *ChecksumPointer)
{
	while ( !( UCSR0A & (1<<UDRE0)) )
	{
	}
	UDR0 = byte;

	if(UpdateCRCflag == 1)
	{
		crc16_update(ChecksumPointer, byte); //To update the running CRC16 calculation
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void send_serial_unsigned_int(unsigned int num, int UpdateCRCflag, uint16_t *ChecksumPointer)
{
	unsigned char lo_byte, hi_byte;

	lo_byte = num;
	hi_byte = num>>8;

	send_serial_byte(lo_byte, UpdateCRCflag, ChecksumPointer);
	send_serial_byte(hi_byte, UpdateCRCflag, ChecksumPointer);
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void send_serial_float(float num, int UpdateCRCflag, uint16_t *ChecksumPointer)
{
	my_union Tx_float_union;
	Tx_float_union.float_num = num;

	send_serial_byte(Tx_float_union.char_num[0], UpdateCRCflag, ChecksumPointer);
	send_serial_byte(Tx_float_union.char_num[1], UpdateCRCflag, ChecksumPointer);
	send_serial_byte(Tx_float_union.char_num[2], UpdateCRCflag, ChecksumPointer);
	send_serial_byte(Tx_float_union.char_num[3], UpdateCRCflag, ChecksumPointer);
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void send_serial_string(char string_input[], int UpdateCRCflag, uint16_t *ChecksumPointer) //PUT THE ARGUMENT IN DOUBLE-QUOTES "INPUT"
{
	int iii = 0;
	while(1)
	{
		if(iii <= strlen(string_input)-1)//(string_input[iii] != '\0')
		{
			send_serial_byte(string_input[iii], UpdateCRCflag, ChecksumPointer);
			iii++;
		}
		else
		{
			break;
		}
	}

}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline unsigned int reconstructSignedIntFromHiByteLoByte(unsigned int HiByte, unsigned int LoByte)
{
	
	unsigned int reconstructed_value = (unsigned int) LoByte | (HiByte<<8);
	
	return reconstructed_value;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void delayms( uint16_t millis ) 
{
	while ( millis ) 
	{
		_delay_ms( 1 );
		millis--;
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void SendSerialTxUpdate() //unicorn
{

	crc16_reset(&CRC16_checksum_Tx); //To reset the CRC16 calculation

	//send_serial_float(TxCounter, 1, &CRC16_checksum_Tx); //byte 0:3
	//send_serial_float(main_loop_global_time, 1, &CRC16_checksum_Tx); //byte 4:7

	send_serial_float(low0, 1, &CRC16_checksum_Tx); //byte 0:3
	send_serial_float(high0, 1, &CRC16_checksum_Tx); //byte 4:7
	send_serial_float(main_loop_global_time, 1, &CRC16_checksum_Tx); //byte 8:11

	send_serial_unsigned_int(CRC16_checksum_Tx, 0, NULL); //bytes -4:-3
	send_serial_byte('\r', 0, NULL);	//byte -2
	send_serial_byte('\n', 0, NULL);	//byte -1

	TxCounter = TxCounter + 1;

}
//////////////////////////////////////////////


//////////////////////////////////////////////
void PinInit(void)
{
	

	///////////////////////Output DO pins
	DDRD |= (1 << DDD4); //PORTD4 set hi to be a digital output for LED for debugging.
	DDRD |= (1 << DDD7); //PORTD7 set hi to be a digital output for LED for debugging. 
	DDRB |= (1 << DDB2); //PORTB2 set hi to be a digital output for O-Scope main loop toggling for debugging.
	///////////////////////
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void setDebugLED_1(unsigned int value)
{
	if(value == 1)
	{
		 PORTD |= (1 << PORTD4); //turn on external LED
		 debugState1 = 1;
	}
	else if(value == 0)
	{
		PORTD &= ~(1 << PORTD4); //turn off external LED
		debugState1 = 0;
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setDebugLED_2(unsigned int value)
{
	if(value == 1)
	{
		 PORTD |= (1 << PORTD7); //turn on external LED
		 debugState2 = 1;
	}
	else if(value == 0)
	{
		PORTD &= ~(1 << PORTD7); //turn off external LED
		debugState2 = 0;
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void blinkDebugLED_2(int numberBlinks, int millisecondsBlink)
{

	setDebugLED_2(0);
	delayms(1);

	for(int blinkCounter = 0; blinkCounter < numberBlinks; blinkCounter++)
	{
		setDebugLED_2(1);
		delayms(millisecondsBlink);
		setDebugLED_2(0);
		delayms(millisecondsBlink);
	}

}
//////////////////////////////////////////////


//////////////////////////////////////////////
void toggleDebugLED_1(void)
{
	if(debugState1 == 0)
	{
		setDebugLED_1(1);
	}
	else if(debugState1 == 1)
	{
		setDebugLED_1(0);
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void toggleDebugLED_2(void)
{
	if(debugState2 == 0)
	{
		setDebugLED_2(1);
	}
	else if(debugState2 == 1)
	{
		setDebugLED_2(0);
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void setOScopeDitigalLine(unsigned int value)
{
	if(value == 1)
	{
		 PORTB |= (1 << PORTB2); //turn on B2 output line for OScope debugging.
		 debugStateOScope = 1;
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB2); //turn off B2 output line for OScope debugging.
		debugStateOScope = 0;
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void toggleOScopeDO(void)
{
	if(debugStateOScope == 0)
	{
		setOScopeDitigalLine(1);
	}
	else if(debugStateOScope == 1)
	{
		setOScopeDitigalLine(0);
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void SerialInit(void)
{
	UCSR0A = (1 << U2X0); //U2X0 = 1 doubles the transmission speed
	UCSR0B = (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0); //enable RX Complete Interrupt, enable Tx, enable Rx
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8n1

	UBRR0H = 0;
	UBRR0L = 4; // UBRR0L = 4 for 0.5Mbs and has 0% serial error

	DDRD &= ~(1 << DDD0); // PORTD0 is set as input for UART RX.
	PORTD |= (1 << DDD0); //Turn on pull-up resistor.
	DDRD |= (1 << DDD1); // PORTD1 is set as output for UART TX.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void TimerOneInitForMAE3(void) 
{

	TCCR1A &= ~(1<<WGM10); //Set WGM10 to 0 for normal mode/mode 0 (TOP = 0xFFFF). PAGE 137
	TCCR1A &= ~(1<<WGM11); //Set WGM11 to 0 for normal mode/mode 0 (TOP = 0xFFFF). PAGE 137
	TCCR1B &= ~(1 << WGM12); //Set WGM12 to 0 for normal mode/mode 0 (TOP = 0xFFFF). PAGE 137
	TCCR1B &= ~(1 << WGM13); //Set WGM13 to 0 for normal mode/mode 0 (TOP = 0xFFFF). PAGE 137

	TCCR1B &= ~(1<<CS10); //Prescaler set to 1/8th. PAGE 134
	TCCR1B |= (1<<CS11); //Prescaler set to 1/8th. PAGE 134
	TCCR1B &= ~(1<<CS12); //Prescaler set to 1/8th. PAGE 134
	Timer1Prescalar = 8.0;

	//TCCR1B |= (1<<CS10); //Prescaler set to 1/1024th. PAGE 134
	//TCCR1B &= ~(1<<CS11); //Prescaler set to 1/1024th. PAGE 134
	//TCCR1B |= (1<<CS12); //Prescaler set to 1/1024th. PAGE 134

	//With prescaler set to 1/1024th and running @ 20MHz, deltaT/click = 0.0000512sec,
	//so 1/244Hz = 0.0041sec
	//so 0.0041sec/0.0000512sec/click = 80 clicks for the full period measurement. At a 1% DC, we'd get 8 clicks.
	//16-bit limit = 32767, so 0.0041sec/32,767 = 7991951.2Hz for a prescalar of 2.5. Use a prescalar of 8 instead.

	//NO INTERRUPT

	TCNT1 = 0; //Reset the clock.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void TimerTwoInitMode4CTC_NoInterrupt(void)
{
	TCCR2A &= ~(1 << WGM20);  //Set for CTC, mode 4 (TOP = OCR2A).PAGE 155
	TCCR2A |= (1 << WGM21);  //Set for CTC, mode 4  (TOP = OCR2A).PAGE 155
	TCCR2B &= ~(1 << WGM22); //Set for CTC, mode 4  (TOP = OCR2A). PAGE 155


	TCCR2B |= (1 << CS20); //Prescaler set to 1/1024th. PAGE 156. DIFFERENT FROM BIT SETTINGS FOR TIMER1!!!
	//TCCR2B &= ~(1 << CS20);
	TCCR2B |= (1 << CS21);  //Prescaler set to 1/1024th. PAGE 156. DIFFERENT FROM BIT SETTINGS FOR TIMER1!!!
	TCCR2B |= (1 << CS22); //Prescaler set to 1/1024th. PAGE 156. DIFFERENT FROM BIT SETTINGS FOR TIMER1!!!

	OCR2A = CalcTMR2mode4_OCRB_from_freq(500); //We're setting to half of the desired 1000Hz because this isn't a typical square wave.
	////////////////////////////TIMSK2 |= (1 << OCIE2A); //Enable the OCR2A interrupt.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned int CalcTMR2mode4_OCRB_from_freq(float frequency)
{
	unsigned int OCR_A_OR_B = round(F_CPU/(2*timer2_clock_prescalar*frequency) - 1); //page 123
	return OCR_A_OR_B;
}
//////////////////////////////////////////////

////////////////////////////////////////////// INTERRUPT RESPONSE
ISR(INT0_vect)
{
	//DisableINT0();

	if((0b00000100 & PIND) != 0) //PD2, INT0
	{
		//Rising Edge
		EndOfCycleTimeStamp = TCNT1;// - TCNT_ReadingClockCycleOffset;

		low0 = EndOfCycleTimeStamp - FallingEdgeTimeStamp;
		high0 = FallingEdgeTimeStamp;

		TCNT1 = 0; //Reset the clock.
		
	}
	else
	{
		FallingEdgeTimeStamp = TCNT1;// - TCNT_ReadingClockCycleOffset;
	}

	//EnableINT0();
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void DisableINT0(void)
{
	EIMSK &= ~(1 << INT0); //Disable interrupt INT0.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void EnableINT0(void)
{
	EIMSK |= (1 << INT0); //Enable interrupt INT0.
	EIFR |= (1 << INT0); //Manually clear INT0 flag.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void externalInterrupt0Init(void)
{
	//External Pin Interrupts detailed on page 73 of the 328p manual.

	DDRD &= ~(1 << DDD2); //PORTD2 set low to be a digital input for external interrupt INT0.
	PORTD |= (1 << PORTD2); //Turn on pull-up resistor.

	//EICRA &= ~(1 << ISC00); //Trigger on falling-edge page 71 of the 328p manual. Using falling edge because the switch pulls it to gnd
	//EICRA |= (1 << ISC01); //Trigger on falling-edge page 71 of the 328p manual. Using falling edge because the switch pulls it to gnd

	//EICRA |= (1 << ISC00); //Trigger on rising-edge page 71 of the 328p manual.
	//EICRA |= (1 << ISC01); //Trigger on rising-edge page 71 of the 328p manual.


	EICRA |= (1 << ISC00); //Trigger on toggle (any logical change). page 71 of the 328p manual.

	EIMSK |= (1 << INT0); //Enable interrupt INT0.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
int main(void) 
{	

	TimerTwoInitMode4CTC_NoInterrupt(); //Initialize the main_loop_global_time using timer 2.
	TimerOneInitForMAE3();
	PinInit();
	externalInterrupt0Init();
	SerialInit();
	sei();
	

	//////////////////////////////////////////////
	//////////////////////////////////////////////
	while(1)
	{	

		//////////////////////////////////////////////
		//////////////////////////////////////////////
		//////////////////////////////////////////////
		if ((TIFR2 & 0b00000010) != 0)
		{

			main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS


			//////////////////////////////////////////////
			//////////////////////////////////////////////
			//////////////////////////////////////////////
			//////////////////////////////////////////////
			if(main_loop_global_time - last_loop_time >= 0.007) //0.007 = 143Hz
			{

				//DisableINT0();
				SendSerialTxUpdate();
				//EnableINT0();
				last_loop_time = main_loop_global_time;
				

			}		
			//////////////////////////////////////////////
			//////////////////////////////////////////////
			//////////////////////////////////////////////
			//////////////////////////////////////////////

			TIFR2 |= (1<<OCF2A); //clear interrupt flag

		}
		//////////////////////////////////////////////
		//////////////////////////////////////////////
		//////////////////////////////////////////////
	}
	//////////////////////////////////////////////
	//////////////////////////////////////////////

	return 0;
}
//////////////////////////////////////////////


///////////////////////////////////////////////////// unused below:
		        /* From USDigital MAE3 Datasheet Page 7:
		        12-bit PWM:
		        x = ((t on * 4098) / (t on+ t off)) -1
		        If x <= 4094, then Position = x
		        If x = 4096 then Position = 4095

		        x0 = ((high0 * 4098)/(high0 + low0)) - 1
		        if x0 <= 4094:
		            tempAngle0 = x0
		        elif x0 == 4096:
		            tempAngle0 = 4095

		        high1 = timer1Value & 0xFFFF
		        low1 = (timer1Value >> 16) & 0xFFFF
		        tempDutyCycle1 = high1 / float(low1 + high1) * 100.0 */
		

				////////////////////////////////////////////// THIS CALCULATION MUST TAKE PLACE OUTSIDE OF THE INTERRUPT BECAUSE IT'S SLOW.
				

				//low0 = EndOfCycleTimeStamp - FallingEdgeTimeStamp;
				/*
				if(low0 > 1025)
				{
					low0 = 1025;
				}
				else if(low0 < 1)
				{
					low0 = 1;
				}
				*/


				//high0 = FallingEdgeTimeStamp;
				/*
				if(high0 > 1025)
				{
					high0 = 1025;
				}
				else if(high0 < 1)
				{
					high0 = 1;
				}
				*/


				/*
		        x0 = ((high0 * 4098.0)/(high0 + low0)) - 1.0;

				if(x0 <= 0)
				{
					ABSencoderAngle = 0.0;
				}
		        else if(x0 <= 4095)
				{
		            ABSencoderAngle  = x0;
				}
		        else if(x0 >= 4096)
				{
		            ABSencoderAngle  = 4095;
				}
				else
				{
					ABSencoderAngle = -11111;
				}


				ABSencoderFrequency = (float) F_CPU/(Timer1Prescalar*EndOfCycleTimeStamp);
				*/ 
				//////////////////////////////////////////////




