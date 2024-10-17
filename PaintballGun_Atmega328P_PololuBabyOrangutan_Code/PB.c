#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L
#define timer1_clock_prescalar 1.0
#define timer2_clock_prescalar 1024.0
#define TxRegularUpdatesFlag 1
#define send_every_byte_back_as_received_mode 0
#define dead_man_time 10.0 //5000mS
#define dead_man_enable_flag_define 0 //Sets whether or not the board should use deadman timer.
#define cameraTriggerHiPulseTime 0.009 //unicorn Set the trigger pulse time seconds, so this is 10mS
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
void Motor0Init(void);
void Motor2Init(void);
void setMotor_0_char(unsigned char value);
void setMotor_2_char(unsigned char value);

void TimerOneInit(void);
void turnFPSpinOn(void);
void turnFPSpinOff(void);
void toggleCameraFPSpin(void);
void setCameraFPSpin(unsigned int value);

void PinInit(void);
void readDigitalInputPins(void);

void ADCInit(void);
void ADCSetChannel(unsigned char channel);

void setSpeaker(unsigned int value);
void setDebugLED_1(unsigned int value);
void setDebugLED_2(unsigned int value);
void setOScopeDitigalLine(unsigned int value);

void toggleDebugLED_1(void);
void toggleDebugLED_2(void);
void toggleOScopeDO(void);

void blinkDebugLED_2(int numberBlinks, int millisecondsBlink);

void SerialInit(void);
void SendSerialTxUpdate(void);

void SPI_init(void);
void SPI_WriteByte(unsigned char);
unsigned char SPI_WriteByte_AndGrabResponse(unsigned char);
unsigned char SPI_ReadByte_AndGrabResponse(unsigned char);
void set_SPI_SS_pin(unsigned int);
//////////////////////////////////////////////

//////////////////////////////////////////////

volatile float main_loop_global_time = 0.0;
volatile float last_loop_time = 0.0;

volatile unsigned int need_to_respond_to_interrupt0_flag = 0;
volatile unsigned int need_to_respond_to_interrupt1_flag = 0;

volatile unsigned int debugState0 = 0;
volatile unsigned int debugState1 = 0;
volatile unsigned int debugState2 = 0;
volatile unsigned int debugState3 = 0;
volatile unsigned int debugStateOScope = 0;

volatile signed int motor_0_velocity = 0;
volatile signed int motor_2_velocity = 0;

volatile unsigned int TxCounter = 0;
volatile unsigned int TxTempCounter = 0;
volatile unsigned int Rx_mutex_flag;
volatile float last_Rx_time = 0.0;
volatile float debug_to_computer;
volatile unsigned int counterRx = 0;
volatile unsigned int message_counter = 0;
volatile unsigned int message_length = 0;
volatile unsigned char RxMessage[20];
volatile int message_being_processed = 0;

uint16_t CRC16_checksum_Tx = 0; //DON'T MAKE VOLATILE
uint16_t CRC16_checksum_Rx = 0; //DON'T MAKE VOLATILE
volatile unsigned int ChecksumRxComputed;
volatile unsigned int ChecksumRxReconstructedFromTwoBytes;

volatile unsigned int ResponseReadyToBeTransmittedFlag = 0;
volatile unsigned char ResponseReadyToBeTransmittedString[20];
volatile unsigned int ResponseRepeatCounter = 0;

volatile unsigned int magnet_state = 0;
volatile unsigned int magnet_state_BEF_DUR_PWR_RELAY = 0;
volatile unsigned int magnet_state_needs_to_be_set_flag = 0;
volatile signed int polarity_record = 0;


volatile unsigned int cameraFPSpinState = 0;
volatile unsigned int isFPSpinOn = 0;
volatile unsigned int cameraFPSblockSignal = 0;
volatile unsigned char cameraFPS = 1; //DEFAULT VALUE ON POWER UP
volatile unsigned char last_cameraFPS = 0;
volatile unsigned int FPS_register_value;
volatile unsigned int FPS_register_value_OCR1A;
volatile unsigned int FPS_register_value_OCR1B;
volatile unsigned int TimerOnePrescalerN;

volatile unsigned char TriggerNeedsToFire = 0;
volatile unsigned char last_TriggerNeedsToFire = 0;
volatile unsigned char NumberShotsInBurst = 0;
volatile unsigned char LaserDutyCycle = 0;
volatile unsigned char last_LaserDutyCycle = 0;
volatile unsigned char JigglerDutyCycle = 0;
volatile unsigned char last_JigglerDutyCycle = 0;

volatile signed int ShotsLeftToFireInThisBurst = 0;
volatile signed int last_ShotsLeftToFireInThisBurst = 0;

volatile unsigned char ADCloByte;
volatile unsigned char ADChiByte;
volatile unsigned int ADC0_value;
volatile float TemperatureDegC_0;
volatile float TemperatureDegC_0_last;

volatile unsigned char SpeakerState = 0;
volatile float SpeakerStartTime = 0.0;
volatile float SpeakerDurationToSound = 5.0;

volatile unsigned char SPI_data_low_byte, SPI_data_medium_byte, SPI_data_high_byte;

//////////////////////////////////////////////

//////////////////////////////////////////////
typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void ADCReadAndRespond(unsigned char channel)
{
	ADCloByte = ADCL;
	ADChiByte	= ADCH;
	ADC0_value = (ADCloByte |(ADChiByte<<8));
	
	float lambda = 0.001;
	
	TemperatureDegC_0 = lambda*TemperatureDegC_0_last + (1.0-lambda)*((5.0*(float)ADC0_value/1024) - 1.25)/0.005;
	TemperatureDegC_0_last = TemperatureDegC_0;

	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}
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
ISR(SPI_STC_vect) //// SPI Transmission/reception complete ISR
{
	
	toggleDebugLED_2();
	
	//SPSR |= (1<<SPIF); //Clear SPI interrupt flag
}
//////////////////////////////////////////////

//////////////////////////////////////////////
ISR(USART_RX_vect)
{
	if(send_every_byte_back_as_received_mode == 1)
	{
		send_serial_byte(UDR0, 0, NULL);
	}
	

	if(message_being_processed == 0) //Waiting to receive the start of a new serial packet
	{
		RxMessage[0] = UDR0; //Grab new serial data.

		if(RxMessage[0] == '$') //First character in a new serial packet
		{
			message_being_processed = 1;
			message_counter = 0;
			
		}

	}
	else if(message_being_processed == 1)//If we're midway through receiving a new serial packet
	{

		message_counter = message_counter + 1;
		RxMessage[message_counter] = UDR0; //Grab new serial data.

		if(RxMessage[message_counter] == '\n' && RxMessage[message_counter - 1] == '\r') //Last two bytes of a message are '\r\n'
		{		
			message_being_processed = 0;
			last_Rx_time = main_loop_global_time;
				
			//////////////////////////////////////////////// PARSE THE RECEIVED MESSAGE
			Rx_mutex_flag = 1;
			/////////////////////////

			if(message_counter == 2) //A trigger-only message. MESSAGE_COUNTER IS THE ACTUAL INDEX NOT THE LENGTH OF THE MESSAGE (WHICH IS MESSAGE_COUNTER + 1)
			{
				//toggleDebugLED_2();
				TriggerNeedsToFire = 1;
				counterRx++;
			}
			else //A parameter-setting message
			{
				ChecksumRxReconstructedFromTwoBytes = reconstructSignedIntFromHiByteLoByte(RxMessage[message_counter-2], RxMessage[message_counter-3]); //Last two bytes are '\r\n'

				crc16_reset(&CRC16_checksum_Rx); //To reset the CRC16 calculation
				for(int tempCounter2 = 0; tempCounter2 < message_counter-3; tempCounter2++) //The '-3' takes off the 2 bytes of received checksum and the '\r\n'.
				{
					crc16_update(&CRC16_checksum_Rx, RxMessage[tempCounter2]); //To update the running CRC16 calculation
				}
				ChecksumRxComputed = CRC16_checksum_Rx; //Copy the value from the pointer that's used by the ccrc16_update function to another variable.

				if(ChecksumRxComputed == ChecksumRxReconstructedFromTwoBytes) //Check that the computed and issued/received checksums are the same
				{
					toggleDebugLED_2();
					
					//unsigned char DollarSign = RxMessage[0]; //unicorn
					TriggerNeedsToFire = RxMessage[1];
					NumberShotsInBurst = RxMessage[2];
					cameraFPS = RxMessage[3];
					LaserDutyCycle = RxMessage[4];
					JigglerDutyCycle = RxMessage[5];
					//reconstructSignedIntRxSerial(laser0SpeedHiByte, laser0SpeedLoByte);

					counterRx++;
					
				}			
			}

			/////////////////////////
			Rx_mutex_flag = 0;
			//////////////////////////////////////////////// PARSE THE RECEIVED MESSAGE
		}
	}
}
//////////////////////////////////////////////

////////////////////////////////////////////// unicorn
void SendSerialTxUpdate()
{

	crc16_reset(&CRC16_checksum_Tx); //To reset the CRC16 calculation

	send_serial_float(main_loop_global_time, 1, &CRC16_checksum_Tx); //bytes 0:4
	
	send_serial_unsigned_int(ADC0_value, 1, &CRC16_checksum_Tx); //bytes 5:6
		
	debug_to_computer = SPI_data_high_byte;//TemperatureDegC_0;//ShotsLeftToFireInThisBurst;
	send_serial_float(debug_to_computer, 1, &CRC16_checksum_Tx); //bytes 7:10 debug message		
		
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
	
	DDRB |= (1 << DDB1); //PORTB1 set hi to be a digital output for camera FPS.
	isFPSpinOn = 1;

	DDRC |= (1 << DDC1); //PORTC1set hi to be a digital output for controlling the speaker relay. 
	///////////////////////
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setSpeaker(unsigned int value)
{
	if(value == 1)
	{
		PORTC |= (1 << PORTC1); //turn on the speaker
		SpeakerState = 1;
		SpeakerStartTime = main_loop_global_time;
	}
	else if(value == 0)
	{
		PORTC &= ~(1 << PORTC1); //turn off the speaker
		SpeakerState = 0;
	}
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


////////////////////////////////////////////// Set up Trigger pulse timer
void TimerOneInit(void)
{

	TIMSK1 |= (1<<OCIE1A); //Enable OCR1A interrupts.
	TIMSK1 |= (1<<OCIE1B); //Enable OCR1B interrupts.

	TCCR1A &= ~(1<<COM1B0); //Normal operation, OCR1B Pin disconnected (page 135).
	TCCR1A &= ~(1<<COM1B1); //Normal operation, OCR1B Pin disconnected (page 135).

	
	TCCR1A &= ~(1<<WGM10); //WGM10 to 0 , mode 0 (TOP = FFFF). PAGE 137
	TCCR1A &= ~(1<<WGM11); //WGM11 to 0 , mode 0 (TOP = FFFF). PAGE 137
	TCCR1B &= ~(1 << WGM12); //WGM12 to 0 , mode 0 (TOP = FFFF). PAGE 137
	TCCR1B &= ~(1 << WGM13); //WGM12 to 0 , mode 0 (TOP = FFFF). PAGE 137

	TimerOnePrescalerN = 1024;
	TCCR1B |=  (1<<CS10); //1    Pre-scaler set to 1/1024th page 138
	TCCR1B &= ~(1<<CS11); //0    Pre-scaler set to 1/1024th page 138
	TCCR1B |=  (1<<CS12); //1    Pre-scaler set to 1/1024th page 138

	//	OCR1A = 97;
	//	OCR1B = 651;
}
//////////////////////////////////////////////


//////////////////////////////////////////////
inline void turnFPSpinOn(void)
{
	cameraFPSblockSignal = 0;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void turnFPSpinOff(void)
{
	cameraFPSblockSignal = 1;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setCameraFPSpin(unsigned int value)
{
	if(value == 1)
	{
		PORTB |= (1 << PORTB1); //turn on camera FPS pin.
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB1); //turn off camera FPS pin.
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void set_SPI_SS_pin(unsigned int value)
{
	if(value == 1)
	{
		PORTB |= (1 << PORTB0);
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB0);
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void toggleCameraFPSpin(void)
{
	if(cameraFPSpinState == 0)
	{
		cameraFPSpinState = 1;
		setCameraFPSpin(1);
	}
	else if(cameraFPSpinState == 1)
	{
		cameraFPSpinState = 0;
		setCameraFPSpin(0);
	}
}
//////////////////////////////////////////////


//////////////////////////////////////////////
inline  void CalcAndSetTimer1registerFPS(void)
{
	FPS_register_value_OCR1A =  (float) cameraTriggerHiPulseTime*20000000.0/TimerOnePrescalerN;
	FPS_register_value_OCR1B =  (float) 20000000.0/(cameraFPS*TimerOnePrescalerN);


	//Have to disable and clear interrupts and TCNT1 so that we don't get delay in the new FPS values taking effect.
	//Otherwise, we could set an OCR1A lower than the current TCNT1.
	TIMSK1 &= ~(1<<OCIE1A); //Disable OCR1A interrupts.
	TIMSK1 &= ~(1<<OCIE1B); //Disable OCR1B interrupts.
	TIFR1 |= (1<<OCF1A); //Clear OCR1A interrupt flag
	TIFR1 |= (1<<OCF1B); //Clear OCR1B interrupt flag

	OCR1A = FPS_register_value_OCR1A;
	OCR1B = FPS_register_value_OCR1B;
	
	TCNT1 = 0; //Clear Timer One.
	TIMSK1 |= (1<<OCIE1A); //Enable OCR1A interrupts.
	TIMSK1 |= (1<<OCIE1B); //Enable OCR1B interrupts.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
ISR(TIMER1_COMPA_vect) //Called to turn off the camera pulse
{
	if(cameraFPSblockSignal == 0 && cameraFPS > 0) ///only pulses if the camera is supposed to be on and is non-zero FPS
	{		
		cameraFPSpinState = 0;
		setCameraFPSpin(0);
	}

	TIFR1 |= (1<<OCF1A); //Clear OCR1A interrupt flag
}
//////////////////////////////////////////////

//////////////////////////////////////////////
ISR(TIMER1_COMPB_vect) //Called at cameraFPS frequency
{
	//if(ShotsLeftToFireInThisBurst > 0)
	//{
		if(cameraFPSblockSignal == 0 && cameraFPS > 0) ///only pulses if the camera is supposed to be on and is non-zero FPS
		{
			cameraFPSpinState = 1;
			setCameraFPSpin(1);
			
			
		}

	//}
	//else
	//{
	//	TriggerNeedsToFire = 0;
	//	last_TriggerNeedsToFire = 0;
	//}

	//if(ShotsLeftToFireInThisBurst > 0)
	//{
		ShotsLeftToFireInThisBurst = ShotsLeftToFireInThisBurst - 1;
	//}
	
	
	TCNT1 = 0;
	TIFR1 |= (1<<OCF1B); //Clear OCR1B interrupt flag
}
//////////////////////////////////////////////


//////////////////////////////////////////////
void Motor0Init(void)
{
	TCCR0A |= (1 << WGM01);  //Set for fast PWM, mode 3 (TOP = 0xFF).PAGE 161
	TCCR0A |= (1 << WGM00);  //Set for fast PWM, mode (TOP = 0xFF).PAGE 161

	TCCR0A |= (1 << COM0A1);//For pin OC0A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0A0);//For pin OC0A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0B1);//For pin OC0B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR0A |= (1 << COM0B0);//For pin OC0B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	
	TCCR0B &= ~(1 << WGM02); //Set WGM2 to 0 for fast PWM, MODE 3 (TOP = 0xFF). PAGE 161

	TCCR0B &= ~(1 << CS00); // 1/8 PAGE 163
	TCCR0B |= (1 << CS01);  // 1/8 PAGE 163
	TCCR0B &= ~(1 << CS02); // 1/8 PAGE 163

	OCR0A = 0;
	OCR0B = 0;

	DDRD |= 1 << DDD5; //Motor 0 control line, Timer0 PWM output B (OC0B)
	DDRD |= 1 << DDD6; //Motor 0 control line, Timer0 PWM output A (OC0A)
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void Motor2Init(void)
{
	TCCR2A |= (1 << WGM21);  //Set for fast PWM, mode 3 (TOP = 0xFF).PAGE 161
	TCCR2A |= (1 << WGM20);  //Set for fast PWM, mode (TOP = 0xFF).PAGE 161

	TCCR2A |= (1 << COM2A1);//For pin OC1A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2A0);//For pin OC1A to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2B1);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	TCCR2A |= (1 << COM2B0);//For pin OC1B to be set on compare match, FAST-PWM, inverting mode.PAGE 160
	
	TCCR2B &= ~(1 << WGM22); //Set WGM2 to 0 for fast PWM, MODE 3 (TOP = 0xFF). PAGE 161

	TCCR2B &= ~(1 << CS20); // 1/8 PAGE 163
	TCCR2B |= (1 << CS21);  // 1/8 PAGE 163
	TCCR2B &= ~(1 << CS22); // 1/8 PAGE 163

	OCR2A = 0;
	OCR2B = 0;

	DDRD |= 1 << DDD3; //Motor 2 control line, Timer2 PWM output B (OC2B)	
	DDRB |= 1 << DDB3; //Motor 2 control line, Timer2 PWM output A (OC2A)
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setMotor_0_char(unsigned char value)
{
		OCR0A = 0;
		OCR0B = value; //Lights LED with positive terminal on Motor B terminal
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setMotor_2_char(unsigned char value)
{
		OCR2A = 0;
		OCR2B = value; //Lights LED with positive terminal on Motor B terminal
}		
//////////////////////////////////////////////


//////////////////////////////////////////////
void ADCInit(void)
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1)|(1 << ADPS0); //Sets the AD pre-scaler to 1/128 or 156.25kHz (page 265)
	ADCSRA |= (1 << ADEN); //Enables ADC.

	ADCSetChannel(0); //must jump start the conversion so that we have a flag to look for in the main() loop

	DDRC &= ~(1 << DDC0); // PORTC0/ADC0 set low to be an analog input.

	ADCSRA |= (1 << ADIF); //Clear the A to D conversion complete flag (done by setting it to 1)
	ADCSRA |= (1 << ADSC); //Initialize the next A to D conversion
}
//////////////////////////////////////////////

//////////////////////////////////////////////
//This function sets the proper ADC channel to pay attention to then starts the conversion
void ADCSetChannel(unsigned char channel)
{
	if(channel == 0)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4 to select ADC0. Page 291
	}
	else if(channel == 1)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC1. Page 291;
		ADMUX |= (1 << MUX0); //Page 291

	}
	else if(channel == 2)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC2. Page 291;
		ADMUX |= (1 << MUX1); //Page 291
	}
	else if(channel == 3)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC3. Page 291;
		ADMUX |= (1 << MUX1) | (1 << MUX0); //Page 291
	}
	else if(channel == 4)
	{
		ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); //Clear bits 0:4, set bit 1 high to select ADC6. Page 291;
		ADMUX |= (1 << MUX2) | (1 << MUX1); //Page 291
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void SPI_WriteByte(unsigned char ByteToSend)
{
	set_SPI_SS_pin(0); //Low active
	
	SPDR = 0x80; //Tells the slave that we're going to make a WRITE operation to the Configuration Register 0
	while(!(SPSR & (1<<SPIF) ));   
	
	SPDR = ByteToSend; // Load data into the buffer
	while(!(SPSR & (1<<SPIF)));   
	
	set_SPI_SS_pin(1); //Pull back high
	
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned char SPI_WriteByte_AndGrabResponse(unsigned char ByteToSend)
{
	set_SPI_SS_pin(0); //Low active
	
	SPDR = 0x80; //Tells the slave that we're going to make a WRITE operation to the Configuration Register 0
	while(!(SPSR & (1<<SPIF) ));
	
	SPDR = ByteToSend; // Load data into the buffer
	while(!(SPSR & (1<<SPIF)));
	
	unsigned char ValueToReturn;
	ValueToReturn = SPDR;
	
	set_SPI_SS_pin(1); //Pull back high
	
	toggleDebugLED_2();
	
	return ValueToReturn;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned char SPI_ReadByte_AndGrabResponse(unsigned char ByteToSend)
{
	set_SPI_SS_pin(0); //Low active
		
	SPDR = ByteToSend; // Load data into the buffer
	while(!(SPSR & (1<<SPIF)));
	
	unsigned char ValueToReturn;
	ValueToReturn = SPDR;
	
	set_SPI_SS_pin(1); //Pull back high
	
	toggleDebugLED_2();
	
	return ValueToReturn;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void SPI_init(void)
{
	//http://maxembedded.com/2013/11/the-spi-of-the-avr/
	
	DDRB |= (1 << DDB5); //Set SPI clock line SCK as Output
	DDRB &= ~(1 << DDB4); //Set SPI Master-In-Slave-Out MISO as Output
	DDRB |= (1 << DDB3); //Set SPI Master-Out-Slave-In MOSI as Output
	DDRB |= (1 << DDB2); //Set SPI SS as Output
	PORTB |= (1 << PORTB2); //Set SPI SS pin high for master
	
	DDRB |= (1 << DDB0); //Set PinB0 as output pin that connects to thermocouple's SS. 
	
	SPCR |= (1 << SPE); //Enable SPI
	SPCR |= (1 << MSTR); //Set this Atmega328P as the SPI master
	
	SPCR |= (1 << SPR0); //Set the SPI Pre-scaler: Fosc/16
	SPCR &= ~(1 << SPR1); //Set the SPI Pre-scaler: Fosc/16
	SPSR &= ~(1 << SPI2X); //Don't double the SPI speed
	
	//Address and data bytes are shifted MSB-first into the serial-data input (SDI) and out of the serial-data output (SDO)."
	SPCR |= (1 << DORD); //Send MSB first (required by the thermocouple chip adafruit-max31856-thermocouple-amplifier) 
	
	SPCR &= ~(1 << CPOL); //Sets clock polarity to leading edge RISING trailing edge FALLING, thermocouple chip doesn't care
	
	//SPCR |= (1 << SPIE); //Enable the SPI interrupt
	
	
	
	//Configuration Register 0: Address for reading is 00h. Address for writing is 80h (page 18 of the chip manual).
	
	//SPI_WriteByte(0b10000000); //Bit 7, CMODE = 1 = Automatic Conversion mode. Conversions occur continuously every 100ms (nominal).
	
	
}

//////////////////////////////////////////////


//////////////////////////////////////////////
int main(void) 
{	
	TimerOneInit();
	Motor0Init();
	Motor2Init(); //Conflicts with the SPI in PinB3 MOSI
	PinInit();
	ADCInit();
	SerialInit();
	//SPI_init();
	sei();
	

	turnFPSpinOff();

	//////////////////////////////////////////////
	//////////////////////////////////////////////
	while(1)
	{	
		delayms(1);
		main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS


		//////////////////////////////////////////////
		//////////////////////////////////////////////
		////////////////////////////////////////////// unicorn
		if(main_loop_global_time - last_loop_time >= 0.002)
		{

			if(TriggerNeedsToFire == 1 && TriggerNeedsToFire != last_TriggerNeedsToFire) //Only update if the value is a rising edge
			{
				ShotsLeftToFireInThisBurst = NumberShotsInBurst;
				turnFPSpinOn();
				last_TriggerNeedsToFire = TriggerNeedsToFire;
				setSpeaker(1);
			}

			if(cameraFPS != last_cameraFPS) //Only update the FPS-controlling OCR1A and OCR1B registers if we need to because the commanded FPS changed.
			{
				CalcAndSetTimer1registerFPS();
				last_cameraFPS = cameraFPS;
			}

			if(LaserDutyCycle != last_LaserDutyCycle) //Only update if the value has changed
			{
				setMotor_0_char(LaserDutyCycle);
				last_LaserDutyCycle = LaserDutyCycle;
			}

			if(JigglerDutyCycle != last_JigglerDutyCycle) //Only update if the value has changed
			{
				setMotor_2_char(JigglerDutyCycle);
				last_JigglerDutyCycle = JigglerDutyCycle;
			}			

			////////////////////////////////////////////// Read analog in
			if ((ADCSRA & 0b00010000) != 0) //When the ADIF flag is set meaning that an ADC conversion has completed
			{
				ADCReadAndRespond(0);
			}
			//////////////////////////////////////////////

			if(main_loop_global_time > (SpeakerStartTime + SpeakerDurationToSound) && SpeakerState == 1)
			{
				setSpeaker(0);
			}
			
			
			
			//if(ShotsLeftToFireInThisBurst == 0 && last_ShotsLeftToFireInThisBurst == 1)
			if(ShotsLeftToFireInThisBurst <= 0)
			{
				setCameraFPSpin(0);
				cameraFPSpinState = 0;
				turnFPSpinOff();
				TriggerNeedsToFire = 0;
				last_TriggerNeedsToFire = 0;

			}
			
			last_ShotsLeftToFireInThisBurst = ShotsLeftToFireInThisBurst;

			//toggleDebugLED_1();

			
			TxTempCounter++;
			if(TxTempCounter == 15)
			{
				SendSerialTxUpdate();
				TxTempCounter = 0;
				
				//SPI_WriteByte(0b01000000); //Bit 6, get a new query  
				
				//This is the high byte of the 19-bit register that contains the linearized and cold-junction-compensated thermocouple temperature value.
				//SPI_data_high_byte = SPI_ReadByte_AndGrabResponse(0x0C); // 0Ch, 0Dh, and 0Eh. //We're receiving 15
				//SPI_data_medium_byte = SPI_ReadByte_AndGrabResponse(0x0D); // 0Ch, 0Dh, and 0Eh. we're receiving 13
				//SPI_data_low_byte = SPI_ReadByte_AndGrabResponse(0x0E); // 0Ch, 0Dh, and 0Eh. we're receiving 14
				
			}
			
			
			last_loop_time = main_loop_global_time;

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












