#include <Arduino.h>
#include <Servo.h>
// PWM pins
#define LeftPWM_pin 3
#define RightPWM_pin 6
#define air_PWM_pin 11
// Digital pins
#define LeftLL_pin 4
#define LeftUL_pin 5
#define RightLL_pin 8
#define RightUL_pin 7
#define man_right_pin 10
#define man_left_pin 9
#define LED_auto_pin 22
#define LED_man_pin 24
#define auto_pin 26
#define man_pin 28
#define air_on_pin 30
#define mode_up_pin 34
#define mode_down_pin 32
#define left_PB_pin 36
#define LED_red 44
#define LED_blu 46
#define LED_grn 45
// Analog pins
#define left_pos_pin 0
#define right_pos_pin 1
#define man_speed_pin 2
#define air_speed_pin 3
#define scale_pin 4
// calibrations
#define left_pos_0 497
#define right_pos_0 557
#define left_min_pos -280
#define left_max_pos 315
#define right_min_pos -280
#define right_max_pos 315
#define KS 2 // 10
#define KP 5 // X10  5 means 0.5
#define PWM_zero 90
#define max_pwr 50 // in %

#define baud_rate 500000
#define COM0 0         // hardware Serial Port
#define START_BYTE '[' // Start Byte for serial commands
#define END_BYTE ']'   // End Byte for serial commands
Servo left_motor;
Servo right_motor;
Servo air_motor;

float phase;

bool LeftLL, LeftUL, RightLL, RightUL, man_right, man_left, run_demo = 0;
bool auto_mode, man_mode, air_on, mode_up, mode_down, left_PB, enable_motion = 0, home_in_progress = 0;
int man_speed, air_speed, scale, left_pos, right_pos, man_pos, air_PWM, demo_left_wpos, demo_right_wpos;
long last_sent_tele, last_run;
int in_home_counter = 0;
int left_percent_power = 0;
int right_percent_power = 0;

int Target_left = 0;
int Target_right = 0;

unsigned int RxByte[2] = {0};      // Current byte received from each of the two comm ports
int BufferEnd[2] = {-1};           // Rx Buffer end index for each of the two comm ports
unsigned int RxBuffer[5][2] = {0}; // 5 byte Rx Command Buffer for each of the two comm ports
byte errorcount = 0;               // serial receive error detected by invalid packet start/end bytes
unsigned int CommsTimeout = 0;     // used to reduce motor power if there has been no comms for a while

unsigned long time_turn_on, time_turn_off, time_started_homing;
bool led_on;
unsigned long millis_on = 200, millis_off = 800;
bool green_LED_on = 0, blue_LED_on = 1, red_LED_on = 0;

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
    Target_left = range((RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort] - 512, left_min_pos, left_max_pos);
    break;
  case 'B':
    Target_right = range((RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort] - 512, right_min_pos, right_max_pos);
    break;
  case 'S':
    enable_motion = 1;
    LED_set_timing(100, 300);
    break;
  case 'E':
    enable_motion = 0;
    home_in_progress = 1;
    LED_set_timing(300, 1200);
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
    Serial.print(left_pos);
    Serial.print(" RP: ");
    Serial.print(right_pos);
    Serial.print(" AirP: ");
    Serial.print(air_PWM);
    Serial.print(" L%: ");
    Serial.print(left_percent_power);
    Serial.print(" R%: ");
    Serial.print(right_percent_power);
    // Serial.print(" spd: ");
    //  Serial.print(man_speed);
    //  Serial.print(" air: ");
    //  Serial.print(air_speed);
    //  Serial.print(" scale: ");
    //  Serial.print(scale);
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
  LeftLL = 1 - digitalRead(LeftLL_pin);
  LeftUL = 1 - digitalRead(LeftUL_pin);
  man_left = 1 - digitalRead(man_left_pin);
  RightLL = 1 - digitalRead(RightLL_pin);
  RightUL = 1 - digitalRead(RightUL_pin);
  man_right = 1 - digitalRead(man_right_pin);
  auto_mode = 1 - digitalRead(auto_pin);
  man_mode = 1 - digitalRead(man_pin);
  air_on = 1 - digitalRead(air_on_pin);
  mode_up = 1 - digitalRead(mode_up_pin);
  mode_down = 1 - digitalRead(mode_down_pin);
  left_PB = 1 - digitalRead(left_PB_pin);

  left_pos = analogRead(left_pos_pin) - left_pos_0;
  right_pos = 1023 - analogRead(right_pos_pin) - right_pos_0;
  man_speed = limit((analogRead(man_speed_pin) - 465) / 5, 100); // -100 to 100
  man_pos = limit((analogRead(man_speed_pin) - 465), 400);
  air_speed = range((analogRead(air_speed_pin) - 23) / 10, 0, 100); // 0.... 100
  scale = range(analogRead(scale_pin) / 50, 0, 20);                 // 0 ...20
  return;
}

void operate_motors(int left_percent, int right_percent)
{
  int left_PWM, right_PWM;
  right_percent = limit(right_percent, max_pwr);
  left_percent = limit(left_percent, max_pwr);

  if (right_percent > 0 && RightUL)
    right_percent = 0;
  if (right_percent < 0 && RightLL)
    right_percent = 0;
  if (left_percent > 0 && LeftUL)
    left_percent = 0;
  if (left_percent < 0 && LeftLL)
    left_percent = 0;

  left_PWM = PWM_zero + (left_percent * 60) / 100;
  right_PWM = PWM_zero + (right_percent * 60) / 100;

  left_motor.write(left_PWM);
  right_motor.write(right_PWM);
}

void send_motors_to_pos(int left_W, int right_W)

{
  int left_err = range(left_W, left_min_pos, left_max_pos) - left_pos;
  int right_err = range(right_W, right_min_pos, right_max_pos) - right_pos;
  left_percent_power = left_err * KP / 10 + KS * sign(left_err);
  right_percent_power = right_err * KP / 10 + KS * sign(right_err);
}

void operate_LEDs()
{
  digitalWrite(LED_man_pin, man_mode);
  digitalWrite(LED_auto_pin, auto_mode);
  if (led_on == 1)
  {
    if (millis() - time_turn_on > millis_on)
    {
      digitalWrite(LED_grn, HIGH);
      digitalWrite(LED_blu, HIGH);
      digitalWrite(LED_red, HIGH);
      time_turn_off = millis();
      led_on = 0;
    }
  }
  if (led_on == 0)
  {
    if (millis() - time_turn_off > millis_off)
    {
      digitalWrite(LED_grn, 1 - green_LED_on);
      digitalWrite(LED_blu, 1 - blue_LED_on);
      digitalWrite(LED_red, 1 - red_LED_on);
      time_turn_on = millis();
      led_on = 1;
    }
  }
}

void operate_air()
{
  if (!air_on || millis() < 2000)
    air_speed = 0;
  air_PWM = 50 + air_speed;
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
      send_motors_to_pos(man_pos, man_pos);
    if (man_right)
      send_motors_to_pos(man_pos, -man_pos);
  }
  else if (mode_down)
  {
    LED_set_timing(200, 1300);
    if (man_left)
      left_percent_power = speed;
    if (man_right)
      right_percent_power = speed;
  }
  else
  {
    LED_set_timing(200, 800);
    if (man_left)
    {
      left_percent_power = speed;
      right_percent_power = speed;
    }
    if (man_right)
    {
      left_percent_power = speed;
      right_percent_power = -speed;
    }
  }
  if (left_PB) // home
  {
    left_percent_power = limit(-left_pos * KP / 10 - KS * sign(left_pos), 40);
    right_percent_power = limit(-right_pos * KP / 10 - KS * sign(right_pos), 40);
  }
  operate_air();
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
    demo_left_wpos = int(sin(phase) * scale * 13);
    demo_right_wpos = int(sin(phase * 1.2) * scale * 13);
    phase += 0.00002 * (man_speed + 100);
    send_motors_to_pos(demo_left_wpos, demo_right_wpos);
    air_PWM = 10 + (sin(phase / 4) + 1.0) * 20;
    air_motor.write(air_PWM);
  }
  else // home
  {
    LED_set_timing(300, 700);
    send_motors_to_pos(0, 0);
    air_motor.write(10);
  }
}

void operate_auto_mode()
{
  LED_set_color(0, 0, 1);
  read_data_from_serial(); // fills target_left and target_right
  if (enable_motion)
  {
    send_motors_to_pos(Target_left * scale / 20, Target_right * scale / 20);
    time_started_homing = millis();
  }
  else if (home_in_progress)
  {
    send_motors_to_pos(0, 0);
    if (abs(left_pos) < 25 && abs(right_pos) < 25)
      in_home_counter++;
    else
      in_home_counter = 0;
    if (in_home_counter > 500 || millis() - time_started_homing > 5000)
    {
      home_in_progress = 0;
      enable_motion = 0;
    }
  }
  air_speed /= 4;
  operate_air();
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

  left_motor.attach(LeftPWM_pin);
  right_motor.attach(RightPWM_pin);
  air_motor.attach(air_PWM_pin);
  air_motor.write(10);
}

void loop()
{
  while ((millis() - last_run) < 1)
    ;
  last_run = millis();
  left_percent_power = 0;
  right_percent_power = 0;
  read_IO();
  if (man_mode)
    operate_manual_mode();
  else if (auto_mode) // auto mode
    operate_auto_mode();
  else
    operate_demo_mode();
  operate_motors(left_percent_power, right_percent_power);
  operate_LEDs();
  send_tele();
}