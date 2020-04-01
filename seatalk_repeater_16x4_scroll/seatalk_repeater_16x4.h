#ifndef _WIND_LOGGER_H
#define _WIND_LOGGER_H

// lazy kipper wants ttg
//#define LAZY_KIPPER

// slipstream wants no scrolling
//#define SLIPSTREAM

// cuan wants speed, depth and logs
//#define CUAN

void init_app(void);
void update_display(void);
void seatalk_message_handler(unsigned char message_type);

#endif