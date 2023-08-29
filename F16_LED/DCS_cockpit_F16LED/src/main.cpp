#include <Arduino.h>
#define RED 10
#define GRN 9
#define BLU 11

int Hue;

void change_led(int pin_number, int value) {
      digitalWrite(pin_number, value);          
}

void RGB_LED(int r, int g, int b)
{
  analogWrite(RED, 255 - r);
  analogWrite(GRN, 255 - g);
  analogWrite(BLU, 255 - b);
}

void HSV_LED(int H, int S, int V) //  0..360, 0..100, 0..100
{
  float s = float(S) / 100;
  float v = float(V) / 200;
  float C = s * v;
  float X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
  float m = v - C;
  float r, g, b;
  if (H >= 0 && H < 60)
  {
    r = C, g = X, b = 0;
  }
  else if (H >= 60 && H < 120)
  {
    r = X, g = C, b = 0;
  }
  else if (H >= 120 && H < 180)
  {
    r = 0, g = C, b = X;
  }
  else if (H >= 180 && H < 240)
  {
    r = 0, g = X, b = C;
  }
  else if (H >= 240 && H < 300)
  {
    r = X, g = 0, b = C;
  }
  else
  {
    r = C, g = 0, b = X;
  }
  int R = (r + m) * 255;
  int G = (g + m) * 255;
  int B = (b + m) * 255;
  RGB_LED(R, G, B);
}

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(GRN, OUTPUT);
  pinMode(BLU, OUTPUT);

  change_led(RED, LOW);  
  change_led(GRN, LOW);  
  change_led(BLU, LOW);  
}

void loop() {
  Hue++;
  if (Hue>359) Hue = 0;
  HSV_LED (Hue,100,100);
  delay(50);
  // put your main code here, to run repeatedly:
}
