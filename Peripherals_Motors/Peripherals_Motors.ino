#include "DualMC33926MotorShield.h"

/* To use the motorshield with motors
 1. Declare the motorshield object outside the setup() and loop() functions (something like, " DualMC33926MotorShield "ObjectName"; " )
 2. Initialize the object in the setup, (something like " ObjectName.init(); " )
 3. To set Motor 1 use, " speed ObjectName.setM1speed(int value)" where value goes from -400 to 400; 0 is no speed, -400 is max speed anticlockwise, 400 is max speed clockwise
 4. To set Motor 2 use, " speed ObjectName.setM2speed(int value)" where value goes from -400 to 400; 0 is no speed, -400 is max speed anticlockwise, 400 is max speed clockwise
 5. To get current readigs from motor1 use "ObjectName.getM1CurrentMilliamps() "
 6. To get current readigs from motor2 use "ObjectName.getM2CurrentMilliamps() " 
 "stopIfFault()" function is used to see if there is something is wrong with the motor (and stop it), doesn't matter if you use it or not
 
 */

DualMC33926MotorShield md;

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

void setup()  
{
  Serial.begin(115200);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();
}

void loop()
{
    int i=200;
    md.setM2Speed(i);
    md.setM1Speed(i);
}
