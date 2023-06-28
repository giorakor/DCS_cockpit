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
#define PWM_zero 92
#define max_pwr 100 // in %

Servo left_motor;
Servo right_motor;

bool LeftLL, LeftUL, RightLL, RightUL, man_right, man_left;
bool auto_mode, man_mode, air_on, mode_up, mode_down, left_PB;
int man_speed, air_speed, scale, left_pos, right_pos, man_pos;
long last_sent_tele;

int left_pct = 0;
int right_pct = 0;

void setup()
{
  Serial.begin(115200);
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
}

void send_tele()
{
  if (millis() - last_sent_tele > 50)
  {
    Serial.print(" LPB: ");
    Serial.print(left_PB);
    Serial.print(" LP: ");
    Serial.print(left_pos);
    Serial.print(" RP: ");
    Serial.print(right_pos);
    Serial.print(" spd: ");
    Serial.print(man_speed);
    Serial.print(" air: ");
    Serial.print(air_speed);
    Serial.print(" scale: ");
    Serial.print(scale);
    Serial.print(" lft, rgt: ");
    Serial.print(man_left);
    Serial.print(man_right);
    Serial.print(" lft LL UL, rgt LL UL: ");
    Serial.print(LeftLL);
    Serial.print(LeftUL);
    Serial.print(RightLL);
    Serial.println(RightUL);
    last_sent_tele = millis();
  }
  return;
}

int limit(int val, int limits)
{
  if (val > limits)
    val = limits;
  if (val < -limits)
    val = -limits;
  return val;
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
  man_pos = limit((analogRead(man_speed_pin) - 465), 250);
  air_speed = analogRead(air_speed_pin);
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

  left_PWM = PWM_zero + (left_percent * 70) / 100;
  right_PWM = PWM_zero + (right_percent * 70) / 100;

  left_motor.write(left_PWM);
  right_motor.write(right_PWM);
}

void operate_LEDs()
{
  digitalWrite(LED_man_pin, man_mode);
  digitalWrite(LED_auto_pin, auto_mode);
}

void loop()
{
  read_IO();
  if (man_mode)
  {
    left_pct = 0;
    right_pct = 0;
    if (man_speed > 20)
      man_speed -= 20;
    else if (man_speed < -20)
      man_speed += 20;
    else
      man_speed = 0;

    if (mode_up)
    {
      if (man_left)
      {
        left_pct = man_speed;
        right_pct = man_speed;
      }
      if (man_right)
      {
        left_pct = man_speed;
        right_pct = -man_speed;
      }
    }
    else if (mode_down)
    {
      if (man_left)
      {
        left_pct = limit((man_pos - left_pos) / 2, 60);
        right_pct = limit((man_pos - right_pos) / 2, 60);
      }
      if (man_right)
      {
        left_pct = limit((man_pos - left_pos) / 2, 60);
        right_pct = limit((-man_pos - right_pos) / 2, 60);
      }
    }
    else
    {
      if (man_left)
        left_pct = man_speed;
      if (man_right)
        right_pct = man_speed;
    }
    if (left_PB)
    {
      left_pct = limit(-left_pos, 40);
      right_pct = limit(-right_pos, 40);
    }
  }
  else
  {
    left_pct = 0;
    right_pct = 0;
  }
  operate_motors(left_pct, right_pct);
  operate_LEDs();
  send_tele();
  delay(1);
}