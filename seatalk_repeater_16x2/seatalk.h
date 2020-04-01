#ifndef _SEATALK_H
#define _SEATALK_H

#define SEATALK_DATA_READ		PORTBbits.RB6
#define SEATALK_TRISB_READ_VAL 	0b01000000

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
void seatalk_process_next_message(void);

#define SEATALK_NUMBER_OF_MESSAGES_IN		10
#define SEATALK_MAX_MESSAGE_SIZE 			20

// these are Seatalk message command identifiers
#define SEATALK_ID_DEPTH 						0x00
#define SEATALK_ID_BOATSPEED					0x20
#define SEATALK_ID_COMP_RUDD_AUTO				0x84
#define SEATALK_ID_COMP_RUDD					0x9c
#define SEATALK_ID_VARIATION					0x99
#define SEATALK_ID_TEMPERATURE					0x27
#define SEATALK_ID_TRIP							0x21
#define SEATALK_ID_LOG							0x22
#define SEATALK_ID_TRIPLOG						0x25
#define SEATALK_ID_APPARENT_WIND_ANGLE 			0x10
#define SEATALK_ID_APPARENT_WIND_SPEED 			0x11
#define SEATALK_ID_SOG							0x52
#define SEATALK_ID_COG							0x53
#define SEATALK_ID_LATITUDE						0x50
#define SEATALK_ID_LONGITUDE					0x51
#define SEATALK_ID_GMT							0x54
#define	SEATALK_ID_DATE							0x56
#define SEATALK_ID_NAV_TO_WAYPOINT				0x85 

#define SEATALK_ID_UNKNOWN						0xff

// these are user defined
#define SEATALK_ID_USER_DEFINED_MESSAGE_MIN		0xc0
#define SEATALK_ID_TRUE_WIND					SEATALK_ID_USER_DEFINED_MESSAGE_MIN
#define	SEATALK_ID_HEADING_MAGNETIC				(SEATALK_ID_USER_DEFINED_MESSAGE_MIN+1)
#define	SEATALK_ID_RUDDER						(SEATALK_ID_USER_DEFINED_MESSAGE_MIN+2)

#ifndef TRUE
#define TRUE	1
#endif
#ifndef FALSE
#define FALSE 	0
#endif

typedef struct
{
	unsigned char year;
	unsigned char month;
	unsigned char date;
} date_t;	

typedef struct
{
	unsigned char hour;
	unsigned char minute;
 	unsigned char second;
} time_t;

#define do_seatalk_read(); \
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
	/* this is the receive stuff*/\
	switch(receive_state)\
	{\
		case RS_HAVE_NOTHING:\
			/* Check for start bit of a received char.*/\
			if(SEATALK_DATA_READ==0)\
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
				if(SEATALK_DATA_READ==0)\
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
				if(SEATALK_DATA_READ)\
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
				parity=SEATALK_DATA_READ;\
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
							if(next_message==SEATALK_NUMBER_OF_MESSAGES_IN)\
							{\
								next_message=0;\
							}\
							\
							/* if next message is being read, move on again	*/\
							if(seatalk_messages_in[next_message][0]==MS_READING)\
							{\
								next_message++;\
								if(next_message==SEATALK_NUMBER_OF_MESSAGES_IN)\
								{\
									next_message=0;\
								}\
							}\
							seatalk_messages_in[next_message][1]=byte_to_receive;\
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
							seatalk_messages_in[next_message][2]=byte_to_receive;\
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
							if(last_message_character_position_written==SEATALK_MAX_MESSAGE_SIZE)\
							{\
								bus_state=BS_WAITING_FOR_COMMAND;\
							}\
							else\
							{\
								seatalk_messages_in[next_message][last_message_character_position_written]=byte_to_receive;\
								if(last_message_character_position_written==((seatalk_messages_in[next_message][2])&0x0f)+3)\
								{\
									/* mark previous message as ready */\
									seatalk_messages_in[next_message][0]=MS_READY;\
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