
#include <Servo.h>

#define LeftPWM_pin 3
#define RightPWM_pin 6
#define air_PWM_pin 11

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

#define man_speed_pin 2
#define air_speed_pin 3
#define scale_pin 4

Servo left_motor;
Servo right_motor;

bool LeftLL, LeftUL, RightLL, RightUL, man_right, man_left;
bool auto_mode, man_mode, air_on, mode_up, mode_down, left_PB;
int man_speed, air_speed, scale;
long last_sent_tele;

int left_PWM = 92;
int right_PWM = 92;
int stp = 92;
int pwm_speed;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 37; i++) pinMode(i, INPUT_PULLUP);
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

void send_tele() {
  if (millis() - last_sent_tele > 50) {
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

void read_IO() {
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

  man_speed = analogRead(man_speed_pin) - 465;
  air_speed = analogRead(air_speed_pin);
  scale = analogRead(scale_pin);
  return;
}

void operate_motors() {
  if (right_PWM > stp && RightUL) right_PWM = stp;
  if (right_PWM < stp && RightLL) right_PWM = stp;
  if (left_PWM > stp && LeftUL) left_PWM = stp;
  if (left_PWM < stp && LeftLL) left_PWM = stp;

  left_motor.write(left_PWM);
  right_motor.write(right_PWM);
}

void operate_LEDs()
{
  digitalWrite(LED_man_pin, man_mode);
  digitalWrite(LED_auto_pin, auto_mode);
}

void loop() {
  read_IO();
  if (man_mode) {
    if (abs(man_speed) < 100) {
      pwm_speed = stp;
    } else {
      if (man_speed > 0) pwm_speed = stp + abs((man_speed - 100) / 10);
      else pwm_speed = stp - abs((man_speed + 100) / 10);
    }
    if (mode_up) {
      left_PWM = stp;
      right_PWM = stp;
      if (man_left) {
        left_PWM = pwm_speed;
        right_PWM = pwm_speed;
      }
      if (man_right) {
        left_PWM = pwm_speed;
        right_PWM = stp - (pwm_speed - stp);
      }
    } else {
      if (man_left) left_PWM = pwm_speed;
      else left_PWM = stp;
      if (man_right) right_PWM = pwm_speed;
      else right_PWM = stp;
    }
  } else {
    left_PWM = stp;
    right_PWM = stp;
  }
  operate_motors();
  operate_LEDs();
  send_tele();
  delay(1);
}
