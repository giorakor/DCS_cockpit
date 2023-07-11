#include <Arduino.h>
#include <Servo.h>
#include <defines.h>

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
int target_speed = 0;
int prev_target_left = 0;
int prev_target_right = 0;
int prev_target_speed = 0;
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

Servo left__motor;
Servo right_motor;
Servo air_motor;

int limit(int val, int limits)
{
  if (val > limits)
    val = limits;
  if (val < -limits)
    val = -limits;
  return val;
}

int range(int val, int lower, int upper)
{
  if (val > upper)
    val = upper;
  if (val < lower)
    val = lower;
  return val;
}

int sign(int val)
{
  if (val > 0)
    return 1;
  if (val < 0)
    return -1;
  return 0;
}

int dead_band(int val, int db)
{
  if (val > db)
    val -= db;
  else if (val < -db)
    val += db;
  else
    val = 0;
  return val;
}

void LED_set_color(bool r, bool g, bool b)
{
  green_LED_on = g;
  blue_LED_on = b;
  red_LED_on = r;
}

void LED_set_timing(int time_on, int time_off)
{
  millis_on = time_on;
  millis_off = time_off;
}

void ParseCommand(int ComPort)
{
  CommsTimeout = 0; // reset the comms timeout counter to indicate we are getting packets
  switch (RxBuffer[0][ComPort])
  {
  case 'A':
    prev_target_left = target_left;
    target_left = int(RxBuffer[1][ComPort] * 256 + RxBuffer[2][ComPort]); // range 0...1012
    target_left = (target_left - 512) * 3 / 5;                            //-300...300
    target_left = range((target_left * motion_amplitude_scale / 20) + pos_ofset, left__min_pos, left__max_pos);
    break;
  case 'B':
    prev_target_right = target_right;
    target_right = int(RxBuffer[1][ComPort] * 256 + RxBuffer[2][ComPort]);
    target_right = (target_right - 512) * 3 / 5;
    target_right = range((target_right * motion_amplitude_scale / 20) + pos_ofset, right_min_pos, right_max_pos);
    break;
  case 'C':
    prev_target_speed = target_speed;
    target_speed = int(RxBuffer[1][ComPort] * 256 + RxBuffer[2][ComPort]);
    target_speed = (target_speed - 512) / 7; // 0....70
    target_speed = range((target_speed * air_speed / 100) + pos_ofset, 1, 100);
    break;
  case 'S':
    enable_auto_motion = 1;
    break;
  case 'E':
    enable_auto_motion = 0;
    home_in_progress = 1;
    break;
  }
}

void read_data_from_serial()
{
  while (Serial.available())
  {
    if (BufferEnd[COM0] == -1)
    {
      RxByte[COM0] = Serial.read();
      if (RxByte[COM0] != START_BYTE)
      {
        BufferEnd[COM0] = -1;
        errorcount++;
      }
      else
      {
        BufferEnd[COM0] = 0;
      }
    }
    else
    {
      RxByte[COM0] = Serial.read();
      RxBuffer[BufferEnd[COM0]][COM0] = RxByte[COM0];
      BufferEnd[COM0]++;
      if (BufferEnd[COM0] > 3)
      {
        if (RxBuffer[3][COM0] == END_BYTE)
        {
          ParseCommand(COM0);
        }
        else
        {
          errorcount++;
        }
        BufferEnd[COM0] = -1;
      }
    }
  }
}

void send_tele()
{
  if (millis() - last_sent_tele > 50)
  {
    Serial.print(" LP: ");
    Serial.print(left__pos);
    Serial.print(" RP: ");
    Serial.print(right_pos);
    Serial.print(" AirP: ");
    Serial.print(air_PWM);
    Serial.print(" L%: ");
    Serial.print(left__percent_power);
    Serial.print(" R%: ");
    Serial.print(right_percent_power);
    // Serial.print(" spd: ");
    //  Serial.print(man_speed);
    //  Serial.print(" air: ");
    //  Serial.print(air_speed);
    //  Serial.print(" motion_amplitude_scale: ");
    //  Serial.print(motion_amplitude_scale);
    //  Serial.print(" lft, rgt: ");
    //  Serial.print(man_left);
    //  Serial.print(man_right);
    Serial.print(" lft LL UL, rgt LL UL: ");
    Serial.print(LeftLL);
    Serial.print(LeftUL);
    Serial.print(RightLL);
    Serial.println(RightUL);
    last_sent_tele = millis();
  }
  return;
}

void read_IO()
{
  // digitals
  LeftLL = 1 - digitalRead(LeftLL_pin);
  LeftUL = 1 - digitalRead(LeftUL_pin);
  man_left = 1 - digitalRead(man_left__pin);
  RightLL = 1 - digitalRead(RightLL_pin);
  RightUL = 1 - digitalRead(RightUL_pin);
  man_right = 1 - digitalRead(man_right_pin);
  auto_mode = 1 - digitalRead(auto_pin);
  man_mode = 1 - digitalRead(man_pin);
  air_on = 1 - digitalRead(air_on_pin);
  mode_up = 1 - digitalRead(mode_up_pin);
  mode_down = 1 - digitalRead(mode_down_pin);
  left__PB = 1 - digitalRead(left__PB_pin);
  // analogs
  left__pos = analogRead(left__pos_pin) - left__pos_0;
  right_pos = 1023 - analogRead(right_pos_pin) - right_pos_0;
  man_speed = limit((analogRead(man_speed_pin) - 465) / 5, 100);     // -100 ... 100
  man_pos = limit((analogRead(man_speed_pin) - 465), 400);           // -400 ... 400
  air_speed = range((analogRead(air_speed_pin) - 23) / 10, 0, 100);  // 0 ... 100
  motion_amplitude_scale = range(analogRead(scale_pin) / 50, 0, 20); // 0 ... 20
  return;
}

void operate_motors(int left__percent, int right_percent)
{
  int left__PWM, right_PWM;
  right_percent = limit(right_percent, max_pwr);
  left__percent = limit(left__percent, max_pwr);

  if (RightUL && right_percent > 0)
    right_percent = 0;
  if (RightLL && right_percent < 0)
    right_percent = 0;
  if (LeftUL && left__percent > 0)
    left__percent = 0;
  if (LeftLL && left__percent < 0)
    left__percent = 0;

  left__PWM = (left__percent * PWM_range_per_side) / 100 + PWM_zero;
  right_PWM = (right_percent * PWM_range_per_side) / 100 + PWM_zero;

  left__motor.write(left__PWM);
  right_motor.write(right_PWM);
}

void calc_motors_pwr_to_pos(int left__W, int right_W)
{
  int left__err = range(left__W, left__min_pos, left__max_pos) - left__pos;
  int right_err = range(right_W, right_min_pos, right_max_pos) - right_pos;
  if (abs(left__err) > DB)
    left__percent_power = left__err * KP / 10 + KS * sign(left__err);
  if (abs(right_err) > DB)
    right_percent_power = right_err * KP / 10 + KS * sign(right_err);
}

void set_LEDs(bool r, bool g, bool b)
{
  digitalWrite(LED_grn, 1 - g);
  digitalWrite(LED_blu, 1 - b);
  digitalWrite(LED_red, 1 - r);
}

void operate_LEDs()
{
  digitalWrite(LED_man_pin, man_mode);
  digitalWrite(LED_auto_pin, auto_mode);
  if (led_on == 1)
  {
    if (millis() - time_turn_on > millis_on)
    {
      set_LEDs(0, 0, 0);
      time_turn_off = millis();
      led_on = 0;
    }
  }
  else
  {
    if (millis() - time_turn_off > millis_off)
    {
      set_LEDs(red_LED_on, green_LED_on, blue_LED_on);
      time_turn_on = millis();
      led_on = 1;
    }
  }
}

void operate_air()
{
  if (!air_on || millis() < 2000)
    air_speed = 0;
  if (target_speed > 0 && enable_auto_motion)
  {
    air_speed = target_speed;
  }
  air_PWM = 10 + air_speed;
  air_motor.write(air_PWM);
}

void operate_manual_mode()
{
  LED_set_color(1, 0, 1);
  int speed = dead_band(man_speed, 20);
  if (mode_up)
  {
    LED_set_timing(200, 300);
    if (man_left)
      calc_motors_pwr_to_pos(man_pos, man_pos);
    if (man_right)
      calc_motors_pwr_to_pos(man_pos, -man_pos);
  }
  else if (mode_down)
  {
    LED_set_timing(200, 1300);
    if (man_left)
      left__percent_power = speed;
    if (man_right)
      right_percent_power = speed;
  }
  else
  {
    LED_set_timing(200, 800);
    if (man_left)
    {
      left__percent_power = speed;
      right_percent_power = speed;
    }
    if (man_right)
    {
      left__percent_power = speed;
      right_percent_power = -speed;
    }
  }
  if (left__PB) // home
  {
    calc_motors_pwr_to_pos(0, 0);
  }
  operate_air();
  enable_auto_motion = 0;
  target_speed = 0;
}

void operate_demo_mode()
{
  LED_set_color(0, 1, 0);
  if (man_left)
    run_demo = 1;
  if (man_right)
    run_demo = 0;
  if (run_demo)
  {
    LED_set_timing(200, 300);
    demo_left__wpos = int(sin(phase) * motion_amplitude_scale * 13);
    demo_right_wpos = int(sin(phase * 1.2) * motion_amplitude_scale * 13);
    phase += 0.00002 * (man_speed + 100);
    calc_motors_pwr_to_pos(demo_left__wpos, demo_right_wpos);
    air_PWM = 5 + (sin(phase / 4) + 1.0) * 20;
    air_motor.write(air_PWM);
  }
  else // home
  {
    LED_set_timing(300, 700);
    calc_motors_pwr_to_pos(0, 0);
    air_motor.write(10);
  }
  enable_auto_motion = 0;
  target_speed = 0;
}

void operate_auto_mode()
{
  LED_set_color(1 - data_is_changing, home_in_progress, 1);
  if (enable_auto_motion)
    LED_set_timing(100, 200);
  else
    LED_set_timing(300, 1500);

  read_data_from_serial(); // fills target_left and target_right
  if (target_left == prev_target_left && target_right == prev_target_right)
    no_change_counter++;
  else
    no_change_counter = 0;

  if (no_change_counter > 500)
    data_is_changing = 0;
  else
    data_is_changing = 1;

  if (enable_auto_motion && data_is_changing)
  {
    calc_motors_pwr_to_pos(target_left, target_right);
  }
  else if (home_in_progress)
  {
    calc_motors_pwr_to_pos(0, 0);
    if (abs(left__pos) < 25 && abs(right_pos) < 25)
      in_home_counter++;
    else
      in_home_counter = 0;
    if (in_home_counter > 500 || millis() - time_started_homing > 3000)
    {
      home_in_progress = 0;
      enable_auto_motion = 0;
    }
  }
  else
    time_started_homing = millis();
  air_speed /= 3;
  operate_air();
}

void calc_motors_power()
{
  left__percent_power = 0;
  right_percent_power = 0;
  if (man_mode)
    operate_manual_mode();
  else if (auto_mode) // auto mode
    operate_auto_mode();
  else
    operate_demo_mode();
}

void setup()
{
  Serial.begin(baud_rate);
  for (int i = 0; i < 38; i++)
    pinMode(i, INPUT_PULLUP);

  pinMode(LeftPWM_pin, OUTPUT);
  pinMode(RightPWM_pin, OUTPUT);
  pinMode(air_PWM_pin, OUTPUT);
  pinMode(LED_auto_pin, OUTPUT);
  pinMode(LED_man_pin, OUTPUT);
  pinMode(LED_red, OUTPUT);
  pinMode(LED_blu, OUTPUT);
  pinMode(LED_grn, OUTPUT);

  digitalWrite(LeftPWM_pin, LOW);
  digitalWrite(RightPWM_pin, LOW);
  digitalWrite(air_PWM_pin, LOW);
  digitalWrite(LED_auto_pin, LOW);
  digitalWrite(LED_man_pin, LOW);
  digitalWrite(LED_grn, HIGH);
  digitalWrite(LED_blu, HIGH);
  digitalWrite(LED_red, HIGH);

  left__motor.attach(LeftPWM_pin);
  right_motor.attach(RightPWM_pin);
  air_motor.attach(air_PWM_pin);
  air_motor.write(10);
}

void loop()
{
  while ((millis() - last_run) < 1)
    ;
  last_run = millis();
  read_IO();
  calc_motors_power();
  operate_motors(left__percent_power, right_percent_power);
  operate_LEDs();
  send_tele();
}