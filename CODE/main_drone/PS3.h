#include <Ps3Controller.h>

#define m1 32
#define m2 23
#define m3 4
#define m4 27
#define red 13
#define green 2

#define freq  5000
#define resolution   8
#define ledChannel1  0
#define ledChannel2  1
#define ledChannel3  2
#define ledChannel4  3


void PS3_SETUP()
{
  Ps3.begin("01:00:01:01:09:09");
  
  Serial.println("PS3 STARTED");


  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);

  ledcAttachPin(m1, ledChannel1);
  ledcAttachPin(m2, ledChannel2);
  ledcAttachPin(m3, ledChannel3);
  ledcAttachPin(m4, ledChannel4);

  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
}

void lights(int a) {
  digitalWrite(red, a);
  digitalWrite(green, a);
}
