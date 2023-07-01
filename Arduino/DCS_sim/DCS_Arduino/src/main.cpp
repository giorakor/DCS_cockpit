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
#define KS 5
#define KP 5 // X10  5 means 0.5
#define PWM_zero 90
#define max_pwr 50 // in %

#define COM0 0         // hardware Serial Port
#define START_BYTE '[' // Start Byte for serial commands
#define END_BYTE ']'   // End Byte for serial commands
Servo left_motor;
Servo right_motor;
Servo air_motor;

bool LeftLL, LeftUL, RightLL, RightUL, man_right, man_left;
bool auto_mode, man_mode, air_on, mode_up, mode_down, left_PB;
int man_speed, air_speed, scale, left_pos, right_pos, man_pos, air_PWM;
long last_sent_tele, last_run;

int left_pct = 0;
int right_pct = 0;

int Target_left = 0;
int Target_right = 0;

unsigned int RxByte[2] = {0};      // Current byte received from each of the two comm ports
int BufferEnd[2] = {-1};           // Rx Buffer end index for each of the two comm ports
unsigned int RxBuffer[5][2] = {0}; // 5 byte Rx Command Buffer for each of the two comm ports
byte errorcount = 0;               // serial receive error detected by invalid packet start/end bytes
unsigned int CommsTimeout = 0;     // used to reduce motor power if there has been no comms for a while

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
  }
}

void CheckSerial0()
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
  scale = analogRead(scale_pin);
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
  left_pct = left_err * KP / 10 + KS * sign(left_err);
  right_pct = right_err * KP / 10 + KS * sign(right_err);
}

void operate_LEDs()
{
  digitalWrite(LED_man_pin, man_mode);
  digitalWrite(LED_auto_pin, auto_mode);
}

void manual_mode()
{
  int speed = dead_band(man_speed, 20);

  if (mode_up)
  {
    if (man_left)
      send_motors_to_pos(man_pos, man_pos);
    if (man_right)
      send_motors_to_pos(man_pos, -man_pos);
  }
  else if (mode_down)
  {
    if (man_left)
      left_pct = speed;
    if (man_right)
      right_pct = speed;
  }
  else
  {
    if (man_left)
    {
      left_pct = speed;
      right_pct = speed;
    }
    if (man_right)
    {
      left_pct = speed;
      right_pct = -speed;
    }
  }
  if (left_PB) // home
  {
    left_pct = limit(-left_pos * KP / 10 + KS * sign(left_pos), 40);
    right_pct = limit(-right_pos * KP / 10 + KS * sign(right_pos), 40);
  }
  if (!air_on || millis() < 1000)
    air_speed = 0;
  air_PWM = 10 + air_speed * 12 / 10;
  air_motor.write(air_PWM);
}

void setup()
{
  Serial.begin(500000); // 115200
  for (int i = 0; i < 38; i++)
    pinMode(i, INPUT_PULLUP);
  pinMode(LeftPWM_pin, OUTPUT);
  pinMode(RightPWM_pin, OUTPUT);
  pinMode(air_PWM_pin, OUTPUT);
  pinMode(LED_auto_pin, OUTPUT);
  pinMode(LED_man_pin, OUTPUT);

  digitalWrite(LeftPWM_pin, LOW);
  digitalWrite(RightPWM_pin, LOW);
  digitalWrite(air_PWM_pin, LOW);
  digitalWrite(LED_auto_pin, LOW);
  digitalWrite(LED_man_pin, LOW);

  left_motor.attach(LeftPWM_pin);
  right_motor.attach(RightPWM_pin);
  air_motor.attach(air_PWM_pin);
  air_motor.write(10);
}

void loop()
{
  while ((millis() - last_run) < 1)
  {
    ;
  }
  last_run = millis();

  left_pct = 0;
  right_pct = 0;
  read_IO();
  if (man_mode)
    manual_mode();
  else if (auto_mode) // auto mode
  {
    CheckSerial0();
    send_motors_to_pos(Target_left, Target_right);
  }
  operate_motors(left_pct, right_pct);
  operate_LEDs();
  send_tele();
  delay(1);
}