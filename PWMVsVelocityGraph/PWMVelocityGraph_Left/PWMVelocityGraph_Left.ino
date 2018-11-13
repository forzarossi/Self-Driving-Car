
#include <Encoder.h>

#include "DualMC33926MotorShield.h"

# define r1 2
# define r2 11
# define l1 3
# define l2 13

Encoder leftEnc(l1,l2);
Encoder rightEnc(r1,r2);
DualMC33926MotorShield md;

void setup() {

  Serial.begin(9600);
  md.init();
  Serial.println("Basic Encoder Test:");

}

long oldLeftPos  = -999;
long oldRightPos  = -999;

int i=100;
int time=0;
int h=0;
int bound=200;
int LowerLimit=80;

const int ArraySize=(200-80/10) + 1;
int StartingTimeArray[ArraySize];
int EndingTimeArray[ArraySize];

int counter=0;
int limit=96;
int StartingTick=0;

void loop() {
  if (h==0){
    //Serial.print("Start time is \t");
    time=millis();
    StartingTimeArray[counter]=time;
    //Serial.println(time);
  }
  md.setM2Speed(bound);
  md.setM1Speed(bound);
  
  long newLeftPos = -leftEnc.read();
  long newRightPos = rightEnc.read(); 
  
  if (newLeftPos != oldLeftPos || newRightPos != oldRightPos){

    oldLeftPos = newLeftPos;
    oldRightPos = newRightPos;

    if (h==0&&newLeftPos!=0)
      limit=newLeftPos+96;
      
    Serial.print(bound);
    Serial.print(' ');
    
    Serial.print(newLeftPos);
    Serial.print(' ');

    Serial.print(newRightPos);
    Serial.print("\t\t");
    Serial.print(h);
    Serial.print("\t\t");
    Serial.println(counter);
   
    h++;
  }
  
  if (newLeftPos>=limit){
    //Serial.print("End time is \t");
    time=millis();
    EndingTimeArray[counter]=time;
    //Serial.println(time);
    Serial.flush();
    
    //limit=limit+96;
    bound=bound-10;
    counter=counter+1;
    h=0;
  }
  
  if (bound<=LowerLimit-10){
    while(1){
      md.setM2Speed(0);
      md.setM1Speed(0);
      bound =200;
      for (i=0;bound>LowerLimit-10;i++){
        Serial.print(bound);
        Serial.print("\t");
        bound=bound-10;
      }
      Serial.println();
      bound =200;
      for (i=0;bound>LowerLimit-10;i++){
        Serial.print(EndingTimeArray[i]-StartingTimeArray[i]);
        Serial.print("\t");
        bound=bound-10;
      }
      Serial.println();
      Serial.println();      
      /*
      Serial.println(StartingTimeArray[1]);
      Serial.println(EndingTimeArray[1]);
      Serial.println();
      */
    }
  }
}
