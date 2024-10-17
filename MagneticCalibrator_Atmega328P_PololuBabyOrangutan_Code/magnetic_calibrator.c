#define F_CPU 20000000UL	// Baby Orangutan frequency (20MHz), // F_CPU tells util/delay.h our clock frequency
#define SCL_CLOCK  100000L
#define timer1_clock_prescalar 1.0
#define timer2_clock_prescalar 1024.0
#define buzzer1_frequency_max 9000.0 //9765.625
#define buzzer1_frequency_min 1000.0 //1000
#define interrupt_response_pulse_counter_LIMIT 4
#define TxRegularUpdatesFlag 1
#define send_every_byte_back_as_received_mode 0
#define echo_Rx_message 0
#define dead_man_time 10.0 //5000mS
#define dead_man_enable_flag_define 1 //Sets whether or not the board should use deadman timer.
//////////////////////////////////////////////

//////////////////////////////////////////////
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> //Include this so that we don't get any warnings about the abs() "implicit declaration".
#include <math.h>
#include <ctype.h> //Necessary for isprint(). function of interest for checking if a character is ASCII is isprint()
#include <string.h> //for strlen()
#include "crc16_Juicero_then_modified_by_Reuben.h"
#include "RD_Midi_in_C.h"
#include "RU_Midi_in_C.h"
#include "SMB_Midi_in_C.h"
#include "ALARM_Midi_in_C.h"
#include "LC_Midi_in_C.h"
//////////////////////////////////////////////

//////////////////////////////////////////////
void PinInit(void);
void readDigitalInputPins(void);

void externalInterrupt0Init(void);
void DisableINT0(void);
void EnableINT0(void);


void Motor0Init(void);
void setMotor_0_signed_int(signed int value); //Accepts range [-255, 255]
void toggleMotor_0_signed_int();

void Motor2Init(void);
void TimerTwoInitMode4CTC(void);
unsigned int CalcTMR2mode4_OCRB_from_freq(float frequency);
unsigned int CalcTMR2mode4_OCRB_from_dT(float deltaT);

void TimerOneInitMode4CTC(void);
unsigned int CalcTMR1mode4_OCRB_from_freq(float frequency);
unsigned int CalcTMR1mode4_OCRB_from_dT(float deltaT);
void setBuzzerFrequency(float frequency);
void temporarilyTurnBuzzerOn(void);
void temporarilyTurnBuzzerOff(void);

void setDebugLED_0(unsigned int value);
void setDebugLED_1(unsigned int value);
void setDebugLED_2(unsigned int value);
void setDebugLED_3(unsigned int value);
void setOScopeDitigalLine(unsigned int value);
void toggleDebugLED_0(void);
void toggleDebugLED_1(void);
void toggleDebugLED_2(void);
void toggleDebugLED_3(void);
void toggleOScopeDO(void);
void blinkDebugLED_3(int numberBlinks, int millisecondsBlink);

void blinkBuzzer(int numberBlinks, int msDurationBlink, float frequency);
void BuzzerPlayRampUpNoise(void);
void BuzzerPlayRampDownNoise(void);
void BuzzerPlayPWRrelayTripperNoise(void);

void SerialInit(void);
void SendSerialTxUpdate(int TxOverrideFlag);

void setJuicerPowerRelay(unsigned int value);
void updateJuicerPowerRelay(void);

void SetupNewSongToPlay(int SongNumberToPlay);
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
volatile unsigned int JuicerPowerRelayState = 0;

volatile unsigned int SSI_alarm1_actual_last = 1;
volatile unsigned int SSI_alarm1_actual = 1;
volatile unsigned int SSI_alarm1_board_button_last = 1;
volatile unsigned int SSI_alarm1_board_button = 1;
volatile unsigned int SSI_alarm2_actual_last = 1;
volatile unsigned int SSI_alarm2_actual = 1;
volatile unsigned int SSI_alarm2_board_button_last = 1;
volatile unsigned int SSI_alarm2_board_button = 1;

volatile signed int motor_0_velocity = 0;
volatile signed int motor_2_velocity = 0;

volatile unsigned int TxCounter = 0;
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

volatile float buzzer1_current_frequency = 0;
volatile unsigned int buzzer1_current_OCR1A = 0;
volatile float BuzzerLastTurnOnTime = 0.0;

volatile unsigned int playMidiFileFlag = 0;
volatile unsigned int playMidiFileFlag_last = 0;
volatile unsigned int playMidiFileFlag_BEF_DUR_PWR_RELAY = 0;
volatile unsigned int MidiNoteCounter = 0;
volatile float MidiSongStartedPlayingTime = 0;
volatile float LastMidiNoteStartedPlayingTime = 0;
volatile unsigned int IsMidiNotePlayingFlag = 0;
volatile float transpose_by_N_octaves = 4.0;
volatile float MidiTimeScaler = 1.0;
volatile float BuzzerFrequencyToSetMidiNumber = 0.0;
volatile float BuzzerFrequencyToSetHz = 0.0;
volatile float BuzzerToStartNextAtTime = 0.0;
volatile unsigned int MidiSongMaxNumberNotes = 10;
volatile unsigned int MidiSongSelection = 0;
volatile unsigned int MidiSongSelection_last = 0;
volatile unsigned int MidiSongSelection_BEF_DUR_PWR_RELAY = 0;

volatile unsigned int noteList_startTime_GENERAL_ARRAY[75];
volatile char noteList_pitch_GENERAL_ARRAY[75];

volatile char DMT_expired_state = 0;
volatile char DMT_expired_state_last = 0;
volatile float DMT_TimeTowardsExpiration = 0.0;
//////////////////////////////////////////////

//////////////////////////////////////////////
typedef union //c-syntax
{
	float float_num;
	char char_num[4];
} my_union; //c-syntax
//////////////////////////////////////////////

//////////////////////////////////////////////
void SetupNewSongToPlay(int SongNumberToPlay)
{

	TIMSK2 &= ~(1 << OCIE2A); //Disable the OCR2A Timer 2 interrupt.

	playMidiFileFlag = 0;

		if(SongNumberToPlay == 1)
		{
			MidiSongMaxNumberNotes = numberMidiNotes_RU;
			MidiTimeScaler = 0.25; //THIS IS SOMETHING THAT I'M HARDCODING BASED ON HOW I LIKE THE SOUND ONCE PLAYING ON THE MICRO
			memcpy((char*) noteList_pitch_GENERAL_ARRAY, (char*) noteList_pitchMidiNumber_RU, numberMidiNotes_RU*sizeof(unsigned int));
			memcpy((char*) noteList_startTime_GENERAL_ARRAY, (char*) noteList_startTime_MidiTicks_RU, numberMidiNotes_RU*sizeof(unsigned int));
			MidiSongSelection = 1;
		}
		else if(SongNumberToPlay == 2)
		{
			MidiSongMaxNumberNotes = numberMidiNotes_RD;
			MidiTimeScaler = 0.25; //THIS IS SOMETHING THAT I'M HARDCODING BASED ON HOW I LIKE THE SOUND ONCE PLAYING ON THE MICRO
			memcpy((char*) noteList_pitch_GENERAL_ARRAY, (char*) noteList_pitchMidiNumber_RD, numberMidiNotes_RD*sizeof(unsigned int));
			memcpy((char*) noteList_startTime_GENERAL_ARRAY, (char*) noteList_startTime_MidiTicks_RD, numberMidiNotes_RD*sizeof(unsigned int));
			MidiSongSelection = 2;
		}
		else if(SongNumberToPlay == 3)
		{
			MidiSongMaxNumberNotes = numberMidiNotes_LC;
			MidiTimeScaler = 0.35; //THIS IS SOMETHING THAT I'M HARDCODING BASED ON HOW I LIKE THE SOUND ONCE PLAYING ON THE MICRO
			memcpy((char*) noteList_pitch_GENERAL_ARRAY, (char*) noteList_pitchMidiNumber_LC, numberMidiNotes_LC*sizeof(unsigned int));
			memcpy((char*) noteList_startTime_GENERAL_ARRAY, (char*) noteList_startTime_MidiTicks_LC, numberMidiNotes_LC*sizeof(unsigned int));
			MidiSongSelection = 3;
		}
		else if(SongNumberToPlay == 4)
		{
			MidiSongMaxNumberNotes = numberMidiNotes_ALARM;
			MidiTimeScaler = 0.75; //THIS IS SOMETHING THAT I'M HARDCODING BASED ON HOW I LIKE THE SOUND ONCE PLAYING ON THE MICRO
			memcpy((char*) noteList_pitch_GENERAL_ARRAY, (char*) noteList_pitchMidiNumber_ALARM, numberMidiNotes_ALARM*sizeof(unsigned int));
			memcpy((char*) noteList_startTime_GENERAL_ARRAY, (char*) noteList_startTime_MidiTicks_ALARM, numberMidiNotes_ALARM*sizeof(unsigned int));
			MidiSongSelection = 4;
		}
		else if(SongNumberToPlay == 5)
		{
			MidiSongMaxNumberNotes = numberMidiNotes_SMB;
			MidiTimeScaler = 0.8; //THIS IS SOMETHING THAT I'M HARDCODING BASED ON HOW I LIKE THE SOUND ONCE PLAYING ON THE MICRO
			memcpy((char*) noteList_pitch_GENERAL_ARRAY, (char*) noteList_pitchMidiNumber_SMB, numberMidiNotes_SMB*sizeof(unsigned int));
			memcpy((char*) noteList_startTime_GENERAL_ARRAY, (char*) noteList_startTime_MidiTicks_SMB, numberMidiNotes_SMB*sizeof(unsigned int));
			MidiSongSelection = 5;
		}
		else
		{
			MidiSongSelection = 0;
			TIMSK2 |= (1 << OCIE2A); //Enable the OCR2A Timer 2 interrupt.
			TIFR2 |= (1<<OCF2A); //clear Timer2 OCRA2 interrupt flag
			return 0;
		}

	MidiNoteCounter = 0;
	MidiSongStartedPlayingTime = main_loop_global_time;
	temporarilyTurnBuzzerOn();
	playMidiFileFlag = 1;

	TIMSK2 |= (1 << OCIE2A); //Enable the OCR2A Timer 2 interrupt.
	TIFR2 |= (1<<OCF2A); //clear Timer2 OCRA2 interrupt flag

	return 1;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline void check_deadman_expired(void)
{

	DMT_expired_state_last = DMT_expired_state;

	DMT_TimeTowardsExpiration = main_loop_global_time - last_Rx_time;

	if(DMT_TimeTowardsExpiration <= dead_man_time)
	{
		DMT_expired_state = 0; //DMT not expired
	}
	else
	{
		DMT_expired_state = 1; //DMT expired
	}
	
}
//////////////////////////////////////////////

//////////////////////////////////////////////
inline int check_deadman_enabled(void)
{
	if(dead_man_enable_flag_define == 1)
	{
		return 1; //DMT enabled
	}
	else
	{
		return 0; //DMT disabled
	}		
}
//////////////////////////////////////////////

//////////////////////////////////////////////
ISR(TIMER2_COMPA_vect)
{
	main_loop_global_time = main_loop_global_time + 0.001; //+ 1 mS
	toggleOScopeDO();

	//////////////////////////////////////////////
	if(playMidiFileFlag == 0 && playMidiFileFlag_last == 1) //Stopping song after music's been playing.	
	{
		temporarilyTurnBuzzerOff();
	}
	else if(playMidiFileFlag == 1 && MidiSongSelection != 0) //Continuing to play music
	{	
						
		if(MidiNoteCounter == MidiSongMaxNumberNotes) //Loop the song!!!
		{
			if(MidiSongSelection == 1 || MidiSongSelection == 2 || MidiSongSelection == 4) //RU, RD, or ALARM LOOP
			{
				MidiNoteCounter = 0;
				MidiSongStartedPlayingTime = main_loop_global_time;
			}
			else
			{
				MidiSongSelection = 0;
				playMidiFileFlag = 0; //STOP PLAYING MUSIC ON NON-LOOPING SONGS
			}
		}

		if(IsMidiNotePlayingFlag == 0)
		{

			BuzzerToStartNextAtTime = noteList_startTime_GENERAL_ARRAY[MidiNoteCounter];

			if(main_loop_global_time - MidiSongStartedPlayingTime >= MidiTimeScaler*BuzzerToStartNextAtTime*(3500000.0/(571428.0*1000.0)) && MidiNoteCounter <= MidiSongMaxNumberNotes - 1)
			{
				BuzzerFrequencyToSetMidiNumber = noteList_pitch_GENERAL_ARRAY[MidiNoteCounter];
				BuzzerFrequencyToSetHz = transpose_by_N_octaves*pow(2.0, (BuzzerFrequencyToSetMidiNumber - 69.0)/12.0)*440.0;
				setBuzzerFrequency(BuzzerFrequencyToSetHz);
				temporarilyTurnBuzzerOn();
				IsMidiNotePlayingFlag = 1;
				LastMidiNoteStartedPlayingTime = main_loop_global_time;
				MidiSongSelection_last = MidiSongSelection;
			}
		}
		else
		{
			if(main_loop_global_time - LastMidiNoteStartedPlayingTime >= 0.100) //USE A SET BEEP LENGTH FOR ALL BEEPS
			{
				temporarilyTurnBuzzerOff();
				MidiNoteCounter = MidiNoteCounter + 1;
				IsMidiNotePlayingFlag = 0;
			}

		}		
	}


	playMidiFileFlag_last = playMidiFileFlag; //Update the history
	//////////////////////////////////////////////


	TIFR2 |= (1<<OCF2A); //clear Timer2 OCRA2 interrupt flag
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
		//send_serial_byte(UDR0);
		//send_serial_byte(RxMessage[message_counter]);

		if(RxMessage[message_counter] == '\n' || RxMessage[message_counter] == '\r') //Last character (newline) in a new serial packet
		{		
			message_being_processed = 0;
			last_Rx_time = main_loop_global_time;
			counterRx++;		
			

			if(echo_Rx_message == 1)
			{
				for(int tempCounter = 0; tempCounter <= message_counter; tempCounter++)
				{
					send_serial_byte(RxMessage[tempCounter], 0, NULL);	
				}
			}

			//////////////////////////////////////////////// PARSE THE RECEIVED MESSAGE
			Rx_mutex_flag = 1;
			/////////////////////////

			//MESSAGE_COUNTER IS THE ACTUAL INDEX NOT THE LENGTH OF THE MESSAGE (WHICH IS MESSAGE_COUNTER + 1)

			ChecksumRxReconstructedFromTwoBytes = reconstructSignedIntFromHiByteLoByte(RxMessage[message_counter-1], RxMessage[message_counter-2]);

			crc16_reset(&CRC16_checksum_Rx); //To reset the CRC16 calculation
			for(int tempCounter2 = 0; tempCounter2 < message_counter-2; tempCounter2++) //The '-2' takes off the 2 bytes of received checksum and the '\n'.
			{
				crc16_update(&CRC16_checksum_Rx, RxMessage[tempCounter2]); //To update the running CRC16 calculation
			}
			ChecksumRxComputed = CRC16_checksum_Rx; //Copy the value from the pointer that's used by the ccrc16_update function to another variable.

			if(ChecksumRxComputed == ChecksumRxReconstructedFromTwoBytes) //Check that the computed and issued/received checksums are the same
			{
				
				char string_for_comparison_OFF[] = "$magnet=off";
				char string_for_comparison_SOUTH[] = "$magnet=south";
				char string_for_comparison_NORTH[] = "$magnet=north";
				char string_for_magnet_command_response[] = "$magnet|Ok";
				
				if(strncmp(RxMessage,string_for_comparison_OFF, strlen(string_for_comparison_OFF)) == 0)
				{
					blinkDebugLED_3(1, 100);
					strcpy(ResponseReadyToBeTransmittedString, string_for_magnet_command_response);
					magnet_state = 0;
					magnet_state_needs_to_be_set_flag = 1;
					ResponseReadyToBeTransmittedFlag = 1;	

					MidiSongSelection = 0;
					playMidiFileFlag = 0;
				}	
				if(strncmp(RxMessage,string_for_comparison_SOUTH, strlen(string_for_comparison_SOUTH)) == 0)
				{
					blinkDebugLED_3(2, 100);
					strcpy(ResponseReadyToBeTransmittedString, string_for_magnet_command_response);
					magnet_state = -255;
					magnet_state_needs_to_be_set_flag = 1;
					ResponseReadyToBeTransmittedFlag = 1;
					SetupNewSongToPlay(2);

					/*
					if(MidiSongSelection != 3) //WE DON'T WANT TO CHANGE THE NOISE IF THE PWR RELAY ALARM IS GOING OFF
					{
						SetupNewSongToPlay(2);
					}
					else
					{
						MidiSongSelection_BEF_DUR_PWR_RELAY = 2;
						playMidiFileFlag_BEF_DUR_PWR_RELAY = 1;
					}
					*/

				}
				if(strncmp(RxMessage,string_for_comparison_NORTH, strlen(string_for_comparison_NORTH)) == 0)
				{
					blinkDebugLED_3(3, 100);
					strcpy(ResponseReadyToBeTransmittedString, string_for_magnet_command_response);
					magnet_state = 255;
					magnet_state_needs_to_be_set_flag = 1;
					ResponseReadyToBeTransmittedFlag = 1;
					SetupNewSongToPlay(1);

					/*
					if(MidiSongSelection != 3) //WE DON'T WANT TO CHANGE THE NOISE IF THE PWR RELAY ALARM IS GOING OFF
					{
						SetupNewSongToPlay(1);
					}
					else
					{
						MidiSongSelection_BEF_DUR_PWR_RELAY = 1;
						playMidiFileFlag_BEF_DUR_PWR_RELAY = 1;
					}	
					*/					
				}



				char StringFC_polarityrec_SOUTH[] = "$polarityrec=south";
				char StringFC_polarityrec_NORTH[] = "$polarityrec=north";
				char String_polarityrec_CMDresponse[] = "$polarityrec|Ok";

				if(strncmp(RxMessage, StringFC_polarityrec_SOUTH, strlen(StringFC_polarityrec_SOUTH)) == 0)
				{
					blinkDebugLED_3(4, 100);
					strcpy(ResponseReadyToBeTransmittedString, String_polarityrec_CMDresponse);
					polarity_record = -1;
					ResponseReadyToBeTransmittedFlag = 1;	
				}
				if(strncmp(RxMessage, StringFC_polarityrec_NORTH, strlen(StringFC_polarityrec_NORTH)) == 0)
				{
					blinkDebugLED_3(5, 100);
					strcpy(ResponseReadyToBeTransmittedString, String_polarityrec_CMDresponse);
					polarity_record = 1;
					ResponseReadyToBeTransmittedFlag = 1;	
				}


				char string_for_comparison_EASTEREGG_OFF[] = "$easteregg=0";
				char string_for_comparison_EASTEREGG_ON[] = "$easteregg=1";
				char string_for_easteregg_command_response_Ok[] = "$easteregg|Ok";
				char string_for_easteregg_command_response_No[] = "$easteregg|No";
				
				if(strncmp(RxMessage,string_for_comparison_EASTEREGG_OFF, strlen(string_for_comparison_EASTEREGG_OFF)) == 0)
				{
					blinkDebugLED_3(6, 100);
					strcpy(ResponseReadyToBeTransmittedString, string_for_easteregg_command_response_Ok);
					playMidiFileFlag = 0;
					temporarilyTurnBuzzerOff(); //Can't count on the interrupt loop to do this.
					ResponseReadyToBeTransmittedFlag = 1;	
				}
				if(strncmp(RxMessage,string_for_comparison_EASTEREGG_ON, strlen(string_for_comparison_EASTEREGG_ON)) == 0)
				{
					blinkDebugLED_3(7, 100);

					
					if(MidiSongSelection == 0) //WE DON'T WANT TO CHANGE THE NOISE IF THE PWR RELAY ALARM IS GOING OFF
					{
						strcpy(ResponseReadyToBeTransmittedString, string_for_easteregg_command_response_Ok);
						ResponseReadyToBeTransmittedFlag = 1;

						SetupNewSongToPlay(5);
					}
					else
					{
						strcpy(ResponseReadyToBeTransmittedString, string_for_easteregg_command_response_No);
						ResponseReadyToBeTransmittedFlag = 1;

						MidiSongSelection_BEF_DUR_PWR_RELAY = 5;
						playMidiFileFlag_BEF_DUR_PWR_RELAY = 1;
					}
				}
			}


			/////////////////////////
			Rx_mutex_flag = 0;
			//////////////////////////////////////////////// PARSE THE RECEIVED MESSAGE
		}
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void SendSerialTxUpdate(int TxOverrideFlag)
{
	TxCounter = TxCounter + 1;

	if(ResponseReadyToBeTransmittedFlag == 1 && TxOverrideFlag == 0)
	{
		crc16_reset(&CRC16_checksum_Tx); //To reset the CRC16 calculation

		//send_serial_byte('$', 1, &CRC16_checksum_Tx); 
		send_serial_string(ResponseReadyToBeTransmittedString, 1, &CRC16_checksum_Tx); 
		//send_serial_string("$magnet|Ok", 1, &CRC16_checksum_Tx);
		send_serial_unsigned_int(CRC16_checksum_Tx, 0, NULL);
		send_serial_byte('\n', 0, NULL);
		
		/*
		ResponseRepeatCounter = ResponseRepeatCounter + 1;

		if(ResponseRepeatCounter == 2) //WE'RE GOING TO TRANSMIT EACH RESPONSE 3 TIMES IN CASE ONE OF THE ATTEMPTS FAILS.
		{
			ResponseRepeatCounter = 0;
			ResponseReadyToBeTransmittedFlag = 0;
		}
		*/

		ResponseReadyToBeTransmittedFlag = 0;
	}

	if(TxCounter == 50 && ResponseReadyToBeTransmittedFlag == 0 && TxOverrideFlag == 0) //every 500ms
	{
		if(TxRegularUpdatesFlag == 1)
		{

			crc16_reset(&CRC16_checksum_Tx); //To reset the CRC16 calculation

			send_serial_byte('$', 1, &CRC16_checksum_Tx); //byte 0
			send_serial_string("status|", 1, &CRC16_checksum_Tx); //bytes 1:7		
			send_serial_float(main_loop_global_time, 1, &CRC16_checksum_Tx); //byte 8:11
			
			debug_to_computer = DMT_TimeTowardsExpiration; //polarity_record; 
			send_serial_float(debug_to_computer, 1, &CRC16_checksum_Tx); //bytes 12:15 debug message

			
			if(magnet_state == 255)
			{
				send_serial_string("magnet is north", 1, &CRC16_checksum_Tx);
			}
			else if(magnet_state == -255)
			{
				send_serial_string("magnet is south", 1, &CRC16_checksum_Tx);
			}
			else if(magnet_state == 0)
			{
				send_serial_string("magnet is off", 1, &CRC16_checksum_Tx);
			}

			if(DMT_expired_state == 0)
			{
				send_serial_string("|DMT=0", 1, &CRC16_checksum_Tx);
			}
			else
			{
				send_serial_string("|DMT=1", 1, &CRC16_checksum_Tx);
			}

			send_serial_unsigned_int(CRC16_checksum_Tx, 0, NULL); //bytes -3:-2
			send_serial_byte('\n', 0, NULL);	//byte -1
		}

		TxCounter = 0;
	}

	if(TxOverrideFlag == 1)
	{

		crc16_reset(&CRC16_checksum_Tx); //To reset the CRC16 calculation

		send_serial_byte('$', 1, &CRC16_checksum_Tx); //byte 0
		send_serial_string("status|", 1, &CRC16_checksum_Tx); //bytes 1:7		
		//send_serial_float(main_loop_global_time, 1, &CRC16_checksum_Tx); //byte 8:11
		send_serial_float(buzzer1_current_OCR1A, 1, &CRC16_checksum_Tx); //byte 8:11
		
		debug_to_computer = buzzer1_current_frequency; // unicorn
		send_serial_float(debug_to_computer, 1, &CRC16_checksum_Tx); //bytes 12:15 debug message

		send_serial_string("BuzzerFrequencyToSetHz", 1, &CRC16_checksum_Tx);

		send_serial_unsigned_int(CRC16_checksum_Tx, 0, NULL); //bytes -3:-2
		send_serial_byte('\n', 0, NULL);	//byte -1

	}

}
//////////////////////////////////////////////


////////////////////////////////////////////// INTERRUPT RESPONSE
ISR(INT0_vect)
{
	DisableINT0();
	

	if(need_to_respond_to_interrupt0_flag == 0) //ONLY RESPOND IF WE'VE ALREADY FINISHED PROCESSING THE CURRENT INTERRUPT.
	{

		blinkDebugLED_3(1, 100);
		magnet_state = 0;
		setMotor_0_signed_int(magnet_state); // A constant, non-toggling or THE SSI ALARM TURNING ON
		magnet_state_needs_to_be_set_flag = 0;
		need_to_respond_to_interrupt0_flag = 1;

	}


	EnableINT0();
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

	EICRA &= ~(1 << ISC00); //Trigger on falling-edge page 71 of the 328p manual. Using falling edge because the switch pulls it to gnd
	EICRA |= (1 << ISC01); //Trigger on falling-edge page 71 of the 328p manual. Using falling edge because the switch pulls it to gnd

	//EICRA |= (1 << ISC00); //Trigger on rising-edge page 71 of the 328p manual.
	//EICRA |= (1 << ISC01); //Trigger on rising-edge page 71 of the 328p manual.


	//EICRA |= (1 << ISC00); //Trigger on toggle (any logical change). page 71 of the 328p manual.

	EIMSK |= (1 << INT0); //Enable interrupt INT0.
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
void setMotor_0_signed_int(signed int value) //Accepts range [-255, 255]
{

	if(value > 255)
	{
		value = 255;
	}
	else if(value < -255)
	{
		value = -255;
	}

	motor_0_velocity = value;

	if(motor_0_velocity >= 0)
	{
		OCR0A = 0;
		OCR0B = abs(motor_0_velocity); //Lights LED with positive terminal on Motor B terminal
	}
	else
	{
		OCR0A = abs(motor_0_velocity);
		OCR0B = 0; //Lights LED with positive terminal on Motor B terminal
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void toggleMotor_0_signed_int()
{

	setMotor_0_signed_int(-1*motor_0_velocity);

}
//////////////////////////////////////////////


//////////////////////////////////////////////
void PinInit(void)
{
	
	///////////////////////Input DI pins
	//DDRC &= ~(1 << DDC0); // PORTC0 set low to be a digital input for .
	//PORTC |= (1 << PORTC0); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC1); // PORTC1 set low to be a digital input for .
	PORTC |= (1 << PORTC1); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC2); // PORTC2 set low to be a digital input for .
	PORTC |= (1 << PORTC2); //Turn on pull-up resistor.

	DDRC &= ~(1 << DDC3); // PORTC3 set low to be a digital input for .
	PORTC |= (1 << PORTC3); //Turn on pull-up resistor.
	///////////////////////

	///////////////////////Output DO pins
	//DDRD |= (1 << DDD1); //PORTD1 set hi to be a digital output for LED for debugging. CAN'T USE BECAUSE WE'RE CONNECTING TO UART RXD
	DDRD |= (1 << DDD4); //PORTD4 set hi to be a digital output for LED for debugging.
	DDRD |= (1 << DDD7); //PORTD7 set hi to be a digital output for LED for debugging. 
	DDRB |= (1 << DDB5); //PORTB5 set hi to be a digital output for LED for debugging.

	DDRB |= (1 << DDB0); //PORTB0 set hi to be a digital output controlling the Juicer's main PWR-OFF safety relay.

	DDRB |= (1 << DDB2); //PORTB2 set hi to be a digital output for O-Scope main loop toggling for debugging.
	///////////////////////
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void readDigitalInputPins(void)
{

	//SSI_alarm1_actual_last = SSI_alarm1_actual;
	SSI_alarm1_board_button_last = SSI_alarm1_board_button;
	SSI_alarm2_actual_last = SSI_alarm2_actual;
	SSI_alarm2_board_button_last = SSI_alarm2_board_button;

	/*
	if((0b00000001 & PINC) != 0) //PC0
	{
		SSI_alarm1_actual = 0; //0 when nothing external is happening to pin and it's pulled to 5V by pull-up resistor
	}
	else
	{
		SSI_alarm1_actual = 1; //1 when pin is forced to GND
	}	
	*/

	if((0b00000010 & PINC) != 0) //PC1
	{
		SSI_alarm1_board_button = 0; //0 when nothing external is happening to pin and it's pulled to 5V by pull-up resistor
	}
	else
	{
		SSI_alarm1_board_button = 1; //1 when pin is forced to GND
	}

	if((0b00001000 & PINC) != 0) //PC3
	{
		SSI_alarm2_board_button = 0; //0 when nothing external is happening to pin and it's pulled to 5V by pull-up resistor
	}
	else
	{
		SSI_alarm2_board_button = 1; //1 when pin is forced to GND
	}

	
	if((0b00000100 & PINC) != 0) //PC2
	{
		SSI_alarm2_actual = 0; //0 when nothing external is happening to pin and it's pulled to 5V by pull-up resistor
	}
	else
	{
		SSI_alarm2_actual = 1; //1 when pin is forced to GND
	}
	

}
//////////////////////////////////////////////


//////////////////////////////////////////////
void setDebugLED_0(unsigned int value)
{
	if(value == 1)
	{
		 //PORTD |= (1 << PORTD1); //turn on user LED
		 debugState0 = 1;
	}
	else if(value == 0)
	{
		//PORTD &= ~(1 << PORTD1); //turn off user LED
		debugState0 = 0;
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
void setDebugLED_3(unsigned int value)
{
	if(value == 1)
	{
		 PORTB |= (1 << PORTB5); //turn on external LED
		 debugState3 = 1;
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB5); //turn off external LED
		debugState3 = 0;
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void blinkDebugLED_3(int numberBlinks, int millisecondsBlink)
{

	setDebugLED_3(0);
	delayms(1);

	for(int blinkCounter = 0; blinkCounter < numberBlinks; blinkCounter++)
	{
		setDebugLED_3(1);
		delayms(millisecondsBlink);
		setDebugLED_3(0);
		delayms(millisecondsBlink);
	}

}
//////////////////////////////////////////////

//////////////////////////////////////////////
void toggleDebugLED_0(void)
{
	if(debugState0 == 0)
	{
		setDebugLED_0(1);
	}
	else if(debugState0 == 1)
	{
		setDebugLED_0(0);
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
void toggleDebugLED_3(void)
{
	if(debugState3 == 0)
	{
		setDebugLED_3(1);
	}
	else if(debugState3 == 1)
	{
		setDebugLED_3(0);
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
void TimerOneInitMode4CTC(void) 
{
	
	// IMPORTANT NOTE: TOP is always set by OCR1A in mode 4/CTC mode, so OCR1B is stuck at that same frequency.
	TCCR1A &= ~(1<<WGM10); //Set WGM0 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 132
	TCCR1A &= ~(1<<WGM11); //Set WGM1 to 0 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 132
	TCCR1B |= (1 << WGM12); //Set WGM2 to 1 for CTC (clear timer on compare), mode 4 (TOP = OCR1A). PAGE 132

	TCCR1B |= (1<<CS10); //Prescaler set to 1/1. DIFFERENT FROM BIT SETTINGS FOR TIMERS0/2!!!
	TCCR1B &= ~(1<<CS11); //Prescaler set to 1/1. DIFFERENT FROM BIT SETTINGS FOR TIMERS0/2!!!
	TCCR1B &= ~(1<<CS12); //Prescaler set to 1/1. DIFFERENT FROM BIT SETTINGS FOR TIMERS0/2!!! 

	//TCCR1B |= (1<<CS10); //Prescaler set to 1/1024th. DIFFERENT FROM BIT SETTINGS FOR TIMERS0/2!!!
	//TCCR1B &= ~(1<<CS11); //Prescaler set to 1/1024th. DIFFERENT FROM BIT SETTINGS FOR TIMERS0/2!!!
	//TCCR1B |= (1<<CS12); //Prescaler set to 1/1024th. DIFFERENT FROM BIT SETTINGS FOR TIMERS0/2!!! 

	
	////////////////////// Below is an experiment with the audio buzzer on 5/31/16 to put a buzzer on OCR1A
	DDRB |= 1 << DDB1; //Motor 2 control line, Timer1 PWM output B (OCB1)
	TCCR1A |= (1<<COM1A0); //Set COM1B0 to 1 to toggle OC1B pin on output compare PAGE 131
	TCCR1A &= ~(1<<COM1A1); //Set COM1B1 to 0 to toggle OC1B pin on output compare PAGE 131
	////////////////////// Below is an experiment with the audio buzzer on 5/31/16 to put a buzzer on OCR1A
	

	/*
	////////////////////// Below is an experiment with the audio buzzer on 5/31/16 to put a buzzer on OCR1B
	DDRB |= 1 << DDB2; //Motor 2 control line, Timer1 PWM output B (OCB1)
	TCCR1A |= (1<<COM1B0); //Set COM1B0 to 1 to toggle OC1B pin on output compare PAGE 131
	TCCR1A &= ~(1<<COM1B1); //Set COM1B1 to 0 to toggle OC1B pin on output compare PAGE 131
	////////////////////// Below is an experiment with the audio buzzer on 5/31/16 to put a buzzer on OCR1B
	*/
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned int CalcTMR1mode4_OCRB_from_dT(float deltaT)
{
	unsigned int OCR_A_OR_B = round(deltaT*F_CPU/(2*timer1_clock_prescalar) - 1); //page 123
	return OCR_A_OR_B;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned int CalcTMR1mode4_OCRB_from_freq(float frequency)
{
	unsigned int OCR_A_OR_B = round(F_CPU/(2.0*timer1_clock_prescalar*frequency) - 1); //page 123
	return OCR_A_OR_B;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned int CalcTMR2mode4_OCRB_from_dT(float deltaT)
{
	unsigned int OCR_A_OR_B = round(deltaT*F_CPU/(2*timer2_clock_prescalar) - 1); //page 123
	return OCR_A_OR_B;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
unsigned int CalcTMR2mode4_OCRB_from_freq(float frequency)
{
	unsigned int OCR_A_OR_B = round(F_CPU/(2*timer2_clock_prescalar*frequency) - 1); //page 123
	return OCR_A_OR_B;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void TimerTwoInitMode4CTC(void)
{
	TCCR2A &= ~(1 << WGM20);  //Set for CTC, mode 4 (TOP = OCR2A).PAGE 155
	TCCR2A |= (1 << WGM21);  //Set for CTC, mode 4  (TOP = OCR2A).PAGE 155
	TCCR2B &= ~(1 << WGM22); //Set for CTC, mode 4  (TOP = OCR2A). PAGE 155


	TCCR2B |= (1 << CS20); //Prescaler set to 1/1024th. PAGE 156. DIFFERENT FROM BIT SETTINGS FOR TIMER1!!!
	//TCCR2B &= ~(1 << CS20);
	TCCR2B |= (1 << CS21);  //Prescaler set to 1/1024th. PAGE 156. DIFFERENT FROM BIT SETTINGS FOR TIMER1!!!
	TCCR2B |= (1 << CS22); //Prescaler set to 1/1024th. PAGE 156. DIFFERENT FROM BIT SETTINGS FOR TIMER1!!!

	OCR2A = CalcTMR2mode4_OCRB_from_freq(500); //We're setting to half of the desired 1000Hz because this isn't a typical square wave.
	TIMSK2 |= (1 << OCIE2A); //Enable the OCR2A interrupt.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setBuzzerFrequency(float frequency)
{
	float frequency_limited = frequency;

	
	if(frequency <= 0)
	{
		temporarilyTurnBuzzerOff();
		return;
	}
	else if(frequency > buzzer1_frequency_max)
	{
		frequency_limited = buzzer1_frequency_max;
	}
	else if(frequency < buzzer1_frequency_min)
	{
		frequency_limited = buzzer1_frequency_min;
	}


	buzzer1_current_frequency = frequency_limited;
	unsigned int buzzer1_current_OCR1A_temp = CalcTMR1mode4_OCRB_from_freq(frequency_limited);
	buzzer1_current_OCR1A = buzzer1_current_OCR1A_temp;
	OCR1A = buzzer1_current_OCR1A_temp;
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void temporarilyTurnBuzzerOn(void)
{
	DDRB |= 1 << DDB1; //Motor 2 control line, Timer1 PWM output B (OCA1)
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void temporarilyTurnBuzzerOff(void)
{
	DDRB &= ~(1 << DDB1); //Motor 2 control line, Timer1 PWM output B (OCA1)
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void blinkBuzzer(int numberBlinks, int msDurationBlink, float frequency)
{
	
	temporarilyTurnBuzzerOff();
	setBuzzerFrequency(frequency);
	delayms(20);

	for(int blinkCounter = 0; blinkCounter < numberBlinks; blinkCounter++)
	{
		temporarilyTurnBuzzerOn();
		delayms(msDurationBlink);
		temporarilyTurnBuzzerOff();
		delayms(msDurationBlink);
	}
	
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void BuzzerPlayRampUpNoise(void)
{
	temporarilyTurnBuzzerOn();

	BuzzerFrequencyToSetHz = 1000;
	for(int yyy = 1; yyy <= 10; yyy++)
	{
		setBuzzerFrequency(BuzzerFrequencyToSetHz);
		delayms(10);
		BuzzerFrequencyToSetHz = BuzzerFrequencyToSetHz + 500.0;
	}
	
	temporarilyTurnBuzzerOff();
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void BuzzerPlayRampDownNoise(void)
{
	temporarilyTurnBuzzerOn();

	BuzzerFrequencyToSetHz = 6000;
	for(int yyy = 1; yyy <= 10; yyy++)
	{
		setBuzzerFrequency(BuzzerFrequencyToSetHz);
		delayms(10);
		BuzzerFrequencyToSetHz = BuzzerFrequencyToSetHz - 500.0;
	}
	
	temporarilyTurnBuzzerOff();
}	
//////////////////////////////////////////////

//////////////////////////////////////////////
void BuzzerPlayPWRrelayTripperNoise(void)
{
	temporarilyTurnBuzzerOn();

	for(int yyy = 1; yyy <= 10; yyy++)
	{
		setBuzzerFrequency(2000.0);
		delayms(10);
		setBuzzerFrequency(1000.0);
		delayms(10);
	}
	
	temporarilyTurnBuzzerOff();
}	
//////////////////////////////////////////////

//////////////////////////////////////////////
void SerialInit(void)
{
	UCSR0A = (1 << U2X0); //double the transmission speed
	UCSR0B = (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0); //enable RX Complete Interrupt, enable Tx, enable Rx
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8n1
	UBRR0H = 0;
	UBRR0L = 21; // FOR U2X0 = 1 (doubling Tx speed), UBRR0L = 4 for 0.5Mbs, UBRR0L = 21 for 115k2 page 180, page 199 for baud rate example table. MATLAB SCRIPT FOR CALCULATING BAUD RATE

	DDRD &= ~(1 << DDD0); // PORTD0 is set as input for UART RX.
	PORTD |= (1 << DDD0); //Turn on pull-up resistor.
	DDRD |= (1 << DDD1); // PORTD1 is set as output for UART TX.
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void setJuicerPowerRelay(unsigned int value)
{
	if(value == 1)
	{
		 PORTB |= (1 << PORTB0); //turn on Juicer Power Relay
		 JuicerPowerRelayState = 1;
	}
	else if(value == 0)
	{
		PORTB &= ~(1 << PORTB0); //turn off Juicer Power Relay
		JuicerPowerRelayState = 0;
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
void updateJuicerPowerRelay(void)
{
	if(SSI_alarm2_actual == 1)
	{
		setJuicerPowerRelay(1); //TURN OFF THE JUICER BY ENERGIZING THE RELAY

		if(SSI_alarm2_actual_last == 0) //Rising Edge only
		{
			DisableINT0();

			MidiSongSelection_BEF_DUR_PWR_RELAY = MidiSongSelection;
			playMidiFileFlag_BEF_DUR_PWR_RELAY = playMidiFileFlag;
			SetupNewSongToPlay(4);

			magnet_state_BEF_DUR_PWR_RELAY = magnet_state;
			magnet_state = 0; //TURN OFF MAGNET WHEN THE RELAY TRIPS
			magnet_state_needs_to_be_set_flag = 1;
		}
	}
	else if(SSI_alarm2_actual == 0)
	{
		setJuicerPowerRelay(0); //TURN ON THE JUICER BY NO LONGER ENERGIZING THE RELAY 

		if(SSI_alarm2_actual_last == 1) //Falling Edge only
		{
			temporarilyTurnBuzzerOff();
			SetupNewSongToPlay(MidiSongSelection_BEF_DUR_PWR_RELAY);
			playMidiFileFlag = playMidiFileFlag_BEF_DUR_PWR_RELAY;

			magnet_state = magnet_state_BEF_DUR_PWR_RELAY; //TURN THE MAGNET BACK TO WHATEVER STATE IT WAS IN PRIOR TO THE RELAY TRIPPING
			magnet_state_needs_to_be_set_flag = 1;

			if(playMidiFileFlag == 0)
			{
				temporarilyTurnBuzzerOff();
			}

			EnableINT0();
			
		}
	}
}
//////////////////////////////////////////////

//////////////////////////////////////////////
int main(void) 
{	
	TimerTwoInitMode4CTC(); //Initialize the main_loop_global_time using timer 2.
	Motor0Init(); //Initialize motor 0 for driving the electromagnet using timer 0.
	TimerOneInitMode4CTC(); //Initialize timer 1 for driving the buzzer.
	temporarilyTurnBuzzerOff();
	PinInit();
	externalInterrupt0Init();
	SerialInit();
	sei();
	
	setMotor_0_signed_int(1); //DEFAULT TO THE DOOR APPEARING TO BE CLOSED


	//////////////////////////////////////////////
	//////////////////////////////////////////////
	while(1)
	{	

			////////////////////////////////////////////// THIS SECTION HANDLES EXTERNAL INTERRUPRT RESPONSES
			//////////////////////////////////////////////
			//////////////////////////////////////////////
			if(need_to_respond_to_interrupt0_flag == 1)	
			{
				setDebugLED_1(0);
				setDebugLED_2(0);
				SetupNewSongToPlay(3);			

				need_to_respond_to_interrupt0_flag = 0;
			}

			//////////////////////////////////////////////
			//////////////////////////////////////////////
			//////////////////////////////////////////////

			//////////////////////////////////////////////
			//////////////////////////////////////////////
			//////////////////////////////////////////////
			else
			{		
				////////////////////////////////////////////// THIS SECTION HANDLES REGULARLY-SCHEDULED, NON-INTERRUPT LOOP FUNCTIONS
				//////////////////////////////////////////////
				//////////////////////////////////////////////
				//////////////////////////////////////////////
				if(main_loop_global_time - last_loop_time >= 0.010)
				{

					readDigitalInputPins();
					updateJuicerPowerRelay();

					if(check_deadman_enabled() == 1)
					{
						check_deadman_expired();
						if(DMT_expired_state == 1 && DMT_expired_state_last == 0) //if DTM is enabled and expired, RISING EDGE ONLY
						{
							magnet_state = 0;
							magnet_state_needs_to_be_set_flag = 1;
						}
					}


					if(magnet_state_needs_to_be_set_flag == 1)
					{
						setMotor_0_signed_int(magnet_state); // A constant, non-toggling

						if(magnet_state == 255)
						{
							setDebugLED_1(1);
							setDebugLED_2(0);
							playMidiFileFlag = 1;
						}
						else if(magnet_state == -255)
						{
							setDebugLED_1(0);
							setDebugLED_2(1);
							playMidiFileFlag = 1;
						}
						else if(magnet_state == 0)
						{
							setDebugLED_1(0);
							setDebugLED_2(0);
							playMidiFileFlag = 0;
						}

						magnet_state_needs_to_be_set_flag = 0;
					}
				
			
					SendSerialTxUpdate(0); //the '(0)' means that we're not using the TxOverrideFlag option.
					last_loop_time = main_loop_global_time;

				}		
				//////////////////////////////////////////////
				//////////////////////////////////////////////
				//////////////////////////////////////////////
				//////////////////////////////////////////////

			}
			//////////////////////////////////////////////
			//////////////////////////////////////////////
			//////////////////////////////////////////////
		//}

	}
	//////////////////////////////////////////////
	//////////////////////////////////////////////

	return 0;
}
//////////////////////////////////////////////




