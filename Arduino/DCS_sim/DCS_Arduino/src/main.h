#include <Arduino.h>

float phase;
bool LeftLL;
bool LeftUL;
bool RightLL;
bool RightUL;
bool man_right;
bool man_left;
bool run_demo = 0;
bool auto_mode;
bool man_mode;
bool air_on;
bool mode_up;
bool mode_down;
bool left__PB;
bool enable_auto_motion = 0;
bool home_in_progress = 0;
bool data_is_changing;

int man_speed;
int air_speed;
int motion_amplitude_scale;
int left__pos;
int right_pos;
int man_pos, air_PWM;
int demo_left__wpos;
int demo_right_wpos;
int in_home_counter = 0;
int left__percent_power = 0;
int right_percent_power = 0;

int target_left = 0;
int target_right = 0;
int prev_target_left = 0;
int prev_target_right = 0;
int no_change_counter = 0;

int BufferEnd[2] = {-1};           // Rx Buffer end index for each of the two comm ports
unsigned int RxByte[2] = {0};      // Current byte received from each of the two comm ports
unsigned int RxBuffer[5][2] = {0}; // 5 byte Rx Command Buffer for each of the two comm ports
unsigned int CommsTimeout = 0;     // used to reduce motor power if there has been no comms for a while
byte errorcount = 0;               // serial receive error detected by invalid packet start/end bytes

unsigned long last_sent_tele;
unsigned long last_run;
unsigned long time_turn_on;
unsigned long time_turn_off;
unsigned long time_started_homing;
unsigned long millis_on = 200;
unsigned long millis_off = 800;

bool led_on;
bool green_LED_on = 0;
bool blue_LED_on = 1;
bool red_LED_on = 0;