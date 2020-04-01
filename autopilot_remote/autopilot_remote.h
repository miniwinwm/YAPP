#ifndef _WIND_LOGGER_H
#define _WIND_LOGGER_H

void init_app(void);
void update_display(void);
void update_stats_background(void);
void update_stats(unsigned char message_type);
void display_time_and_date(void);
void seatalk_message_handler(unsigned char message_type);

#endif