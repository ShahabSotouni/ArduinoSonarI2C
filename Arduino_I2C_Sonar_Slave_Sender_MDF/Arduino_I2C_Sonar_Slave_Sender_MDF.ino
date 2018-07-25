// Composed by Shahab : Shahab.Sotouni@gmail.com
// AUTMAV
// Amirkabir University of Tech. Micro Air Vehicles Lab.
// Credits to I2C and SRF05 Arduino example providers


#include <Wire.h>
#include "MedianFilter.h"

#define ECHO_PIN 4
#define TRIG_PIN  5
#define PULSE_TIMEOUT 200000

MedianFilter MDF;
uint16_t tempread, reading_cm;
byte flag = 0;

//Blink LED
void blinkled()
{
  if(flag)
  {
    flag = 0;
    digitalWrite(13, LOW);
  }
  else
  {
    flag = 1;
    digitalWrite(13, HIGH);
  }
}


void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);          // LED PIN
  pinMode(ECHO_PIN, INPUT);     // ECHO PIN
    pinMode(TRIG_PIN,OUTPUT);
  Wire.begin(0x38);             // i2c in slave mode address is 0x38(in ppz should be 0x70)
  Wire.onRequest(requestEvent); // register event
  MDF.SetWindowSize(41);
}


void loop()
{
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempread = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);
  MDF.PushMedian(tempread);
  reading_cm=MDF.GetMedian()/29.1/2;

   Serial.println(reading_cm);

}


void requestEvent()
{
  byte sendhi = 0, sendli = 0;
  byte sendbyte[3];
  uint16_t tempreading_cm;
  tempreading_cm=reading_cm;
  sendhi=tempreading_cm>>8;
  sendli=tempreading_cm&0xff;
  sendbyte[0]=sendhi;
  sendbyte[1]=sendli;
  sendbyte[2]=(sendhi+sendli)&0xff;
  Wire.write(sendbyte,3);
  blinkled();
}
