// Composed by Shahab : Shahab.Sotouni@gmail.com
// AUTMAV
// Amirkabir University of Tech. Micro Air Vehicles Lab.
// Credits to I2C and SRF05 Arduino example providers
//***************************************************

///*********BIG WARNING****************
//!!! Disconnect TF01 from Arduino RX when uploading code and reconnect after upload !
//!!! Never connect TF01 to Arduino TX!


#include <Wire.h>
#include "RMSFilter.h"
#include "MedianFilter.h"


#define ECHO_PIN 4
#define TRIG_PIN  5
#define PULSE_TIMEOUT 200000

RMSFilter RMSF_Sonar;
MedianFilter MDF_TF01;
MedianFilter MDF_Fused;

uint16_t tempread_Sonar, reading_cm_Sonar,fused_reading_cm;
unsigned int height_TF01=0,height_TF01_lastval=0;
bool ledflag = false;

//Blink LED
void blinkled()
{
  if(ledflag)
  {
    ledflag = false;
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    ledflag = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void requestEvent();

void readTF01();

void readSRF05();



void setup()
{
    Serial.begin(115200);

  pinMode(13, OUTPUT);          // LED PIN
  pinMode(ECHO_PIN, INPUT);     // ECHO PIN
  pinMode(TRIG_PIN,OUTPUT);
  Wire.begin(0x38);             // i2c in slave mode address is 0x38(in ppz should be 0x70)
  Wire.onRequest(requestEvent); // register event
  RMSF_Sonar.SetWindowSize(12);
  MDF_TF01.SetWindowSize(9);
  MDF_Fused.SetWindowSize(9);
}







void loop()
{
  readTF01();


  if(height_TF01>50){
       fused_reading_cm=height_TF01;
  }else {
    readSRF05();
    if(reading_cm_Sonar>30){
  
  
    double int_gain=(((double)reading_cm_Sonar-30.0)/20.0);
    
      fused_reading_cm=(1-int_gain)*(double)reading_cm_Sonar
                        +int_gain*(double)height_TF01;
  }else{
      fused_reading_cm=reading_cm_Sonar;
  } 
  }



/*MDF_Fused.PushMedian(fused_reading_cm);
fused_reading_cm=MDF_Fused.GetMedian();
*/
  Serial.print("Sonar:\t");
  Serial.print(reading_cm_Sonar);
  Serial.print("\t");


  Serial.print("Laser:\t");
  Serial.print(height_TF01);
  Serial.print("\t");
  

  
Serial.print("Fused:\t");
  Serial.println(fused_reading_cm);


  
}






void readTF01(){
 
    while(Serial.available()>=9)
    {
        if((0x59 == Serial.read()) && (0x59 == Serial.read())) //Byte1 & Byte2
        {
            unsigned int t1 = Serial.read(); //Byte3
            unsigned int t2 = Serial.read(); //Byte4
            t2 <<= 8;
            t2 += t1;
            height_TF01=t2;
            t1 = Serial.read(); //Byte5
            t2 = Serial.read(); //Byte6

            t2 <<= 8;
            t2 += t1;

            if (t2>1000 && height_TF01>100 )
              height_TF01-=30;

             MDF_TF01.PushMedian(height_TF01);
             height_TF01=MDF_TF01.GetMedian();
             if (height_TF01>300)
                 height_TF01=height_TF01_lastval;
             
             height_TF01_lastval=height_TF01;
            //Serial.println(t2);

         }
        }
    }





void readSRF05(){
  
  delay(53);
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempread_Sonar = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);
  RMSF_Sonar.PushRMS(tempread_Sonar);
  reading_cm_Sonar=RMSF_Sonar.GetRMS()/29.1/2; 
  
  float tmp=tempread_Sonar/29.1/2;
//  Serial.print(tmp);
//  Serial.print("\t");

  
}





void requestEvent()
{
  byte sendhi = 0, sendli = 0;
  byte sendbyte[3];
  uint16_t tempreading_cm;
  tempreading_cm=fused_reading_cm;
  sendhi=tempreading_cm>>8;
  sendli=tempreading_cm&0xff;
  sendbyte[0]=sendhi;
  sendbyte[1]=sendli;
  sendbyte[2]=(sendhi+sendli)&0xff;
  Wire.write(sendbyte,3); 
  blinkled(); 
}
