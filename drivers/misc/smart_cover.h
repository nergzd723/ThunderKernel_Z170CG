#ifndef _SMART_COVER_H_
#define _SMART_COVER_H_

enum smart_cover_type {
	POWER_BANK = 1,
	AUDIO_SLEEVE
};

void cover_detect_pin_register(char *, void(*)(bool));
void cover_detect_pin_unregister(char *);
void cover_battery_status_register(char *, void(*)(bool));
void cover_battery_status_unregister(char *);
bool is_cover_attached(void);
bool is_cover_battery_low(void);
int get_cover_type(void);

#endif
