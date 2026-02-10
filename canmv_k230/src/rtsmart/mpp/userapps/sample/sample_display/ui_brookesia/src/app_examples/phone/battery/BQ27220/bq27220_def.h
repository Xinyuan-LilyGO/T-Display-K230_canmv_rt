#ifndef BQ27220_H
#define BQ27220_H
#ifdef __cplusplus
extern "C" {
#endif
void _init();
uint32_t tca8418_get_key();
void control_keyboard_backlight(int freq,int duty);
void control_keyboard_indicator_led_caps_lock(int is_open);
#ifdef __cplusplus
}
#endif

#endif
