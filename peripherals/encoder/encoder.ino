/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <PinChangeInt.h>

# define r1 2
# define r2 3
# define l1 9
# define l2 10
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder leftEnc(l1,l2);
Encoder rightEnc(r1,r2);
//   avoid using pins with LEDs attached


void setup() {
//  pinMode(p1,INPUT);
//  pinMode(p2,INPUT);
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldLeftPos  = -999;
long oldRightPos  = -999;

void loop() {
  long newLeftPos = leftEnc.read();
  long newRightPos = rightEnc.read();
  if (newLeftPos != oldLeftPos || newRightPos != oldRightPos){
    oldLeftPos = newLeftPos;
    oldRightPos = newRightPos;
    Serial.print(newLeftPos);
    Serial.print(' ');
    Serial.println(newRightPos);
  }
}
