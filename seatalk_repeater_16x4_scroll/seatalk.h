#ifndef _SEATALK_H
#define _SEATALK_H

#define SEATALK_DATA 			PORTBbits.RB5
#define SEATALK_TRISB_READ_VAL 	0b00100000
#define SEATALK_ANSEL_B_VAL 	0b11011111

#define M_PI 					3.1416f
#define M_PI_2		 			1.5708f
#define DEGREES_TO_RADIANS 		57.259f

typedef void(*message_handler)(unsigned char);

// these are the states the receiver goes through for the process of receiving one byte 
enum receiver_state_t 
{
	RS_HAVE_NOTHING,
	RS_WAIT_HALF_A_BIT,
	RS_HAVE_STARTBIT,
	RS_PARITY=RS_HAVE_STARTBIT+8,
	RS_WAIT_FOR_STOP,
	RS_HAVE_BYTE
};	

enum message_state_t
{
	MS_READY,
	MS_READING,
	MS_DONE,
};	

enum bus_state_t
{
	BS_WAITING_FOR_COMMAND,
	BS_WAITING_FOR_SIZE,
	BS_RECEIVING_MESSAGE
};
	
void init_seatalk(message_handler callback);
unsigned char correct_for_daylight_saving(void);
void parse_next_seatalk_message(void);

#define NUMBER_OF_MESSAGES 			10
#define MAX_MESSAGE_SIZE 			20

// these are Seatalk message command identifiers
#define DEPTH 						0x00
#define APPARENT_WIND_ANGLE 		0x10
#define APPARENT_WIND_SPEED 		0x11
#define HEADING1					0x84
#define HEADING2					0x9c
#define DEPTH						0x00
#define TEMPERATURE					0x27
#define BOATSPEED					0x20
#define TRIPLOG						0x25
#define VARIATION					0x99
#define SOG							0x52
#define COG							0x53
#define LAMPS1      				0x30
#define LAMPS2      				0x80
#define NAV_TO_WAYPOINT				0x85 

// these are user defined
#define USER_DEFINED_MESSAGE_MIN	0xc0
#define TRUE_WIND					USER_DEFINED_MESSAGE_MIN
#define	TIME_TO_GO					(USER_DEFINED_MESSAGE_MIN+1)

#ifndef TRUE
#define TRUE	1
#endif
#ifndef FALSE
#define FALSE 	0
#endif

#define do_seatalk_read(); \
	if(PIE5bits.TMR4IE && PIR5bits.TMR4IF)\
	{\
		static unsigned char receive_tick_count=0;\
		static unsigned char last_message_character_position_written=0;\
		static unsigned char wait_for_command=TRUE;\
		static unsigned char parity;\
		static unsigned char receive_state=RS_HAVE_NOTHING;\
		static unsigned char byte_to_receive;\
		static unsigned char next_message=0;\
		static unsigned char bus_state=BS_WAITING_FOR_COMMAND;\
\
		PIR5bits.TMR4IF=0;\
\
		/* this is the receive stuff*/\
		switch(receive_state)\
		{\
			case RS_HAVE_NOTHING:\
				/* Check for start bit of a received char.*/\
				if(SEATALK_DATA==0)\
				{\
					receive_state=RS_WAIT_HALF_A_BIT;\
					receive_tick_count=0;\
				}\
				break;\
\
			case RS_WAIT_HALF_A_BIT:\
				receive_tick_count++;\
				if(receive_tick_count==4)\
				{\
					if(SEATALK_DATA==0)\
					{\
						receive_tick_count=0;\
						receive_state=RS_HAVE_STARTBIT;\
						byte_to_receive=0;\
					}\
					else\
					{\
						receive_state=RS_HAVE_NOTHING;\
					}		\
				}\
				break;\
\
			default:\
				receive_tick_count++;\
				if(receive_tick_count==8)\
				{\
					receive_tick_count=0;\
					receive_state++;\
					byte_to_receive>>=1;\
					if(SEATALK_DATA)\
					{\
						byte_to_receive|=0x80;\
					}\
				}\
				break;\
\
			case RS_PARITY:\
				receive_tick_count++;\
				if(receive_tick_count==8)\
				{\
					receive_tick_count=0;\
					receive_state++;\
					parity=SEATALK_DATA;\
				}\
				break;\
\
			case RS_WAIT_FOR_STOP:\
				receive_tick_count++;\
				if(receive_tick_count==8)\
				{\
					switch(bus_state)\
					{\
						case BS_WAITING_FOR_COMMAND:\
							if(parity)\
							{\
								/* move on to next message*/\
								next_message++;\
								if(next_message==NUMBER_OF_MESSAGES)\
								{\
									next_message=0;\
								}\
								\
								/* if next message is being read, move on again	*/\
								if(messages[next_message][0]==MS_READING)\
								{\
									next_message++;\
									if(next_message==NUMBER_OF_MESSAGES)\
									{\
										next_message=0;\
									}\
								}\
								messages[next_message][1]=byte_to_receive;\
								bus_state=BS_WAITING_FOR_SIZE;\
							}\
							break;\
\
						case BS_WAITING_FOR_SIZE:\
							if(parity)\
							{\
								/* got an unexpected command message so must be a collision, abandon this message*/\
								bus_state=BS_WAITING_FOR_COMMAND;\
							}\
							else\
							{\
								messages[next_message][2]=byte_to_receive;\
								last_message_character_position_written=2;\
								bus_state=BS_RECEIVING_MESSAGE;\
							}\
							break;\
\
						case BS_RECEIVING_MESSAGE:\
							if(parity)\
							{\
								/* got an unexpected command message so must be a collision, abandon this message*/\
								bus_state=BS_WAITING_FOR_COMMAND;\
							}\
							else\
							{\
								last_message_character_position_written++;\
								if(last_message_character_position_written==MAX_MESSAGE_SIZE)\
								{\
									bus_state=BS_WAITING_FOR_COMMAND;\
								}\
								else\
								{\
									messages[next_message][last_message_character_position_written]=byte_to_receive;\
									if(last_message_character_position_written==((messages[next_message][2])&0x0f)+3)\
									{\
										/* mark previous message as ready */\
										messages[next_message][0]=MS_READY;\
										bus_state=BS_WAITING_FOR_COMMAND;\
									}\
								}\
							}\
							break;\
					}\
					receive_state=RS_HAVE_NOTHING;\
				}\
				break;\
		}\
	}

#endif