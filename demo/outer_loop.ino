#include "DualMC33926MotorShield.h"
#include <math.h>
#include <Encoder.h>


#define circumference 8.625
#define n_sector 24
#define wheel_base 6.5


#define r1 2

#define r2 11

#define l1 3

#define l2 13



#define K 30

#define B 5



Encoder leftEnc(l1,l2);

Encoder rightEnc(r1,r2);



long oldLeftPos  = -999;

long oldRightPos  = -999;

long newLeftPos  = -999;

long newRightPos  = -999;



int PWM_L=200;

int PWM_R=210;


double error = 0;

double delta_v = -0;

double oldError = 0;

double PWM_Correction_Right=0;

double PWM_Correction_Left=0;

double oldVelocity_Left = PWM_L*0.0074-0.7058;

double oldVelocity_Right = PWM_R*0.0073-0.7642;

long time;



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


double p=1;
double c=1;
int j=1;

double x=0;
double y=0;

void SetMotor1speed(int x){

  md.setM1Speed(x);

  stopIfFault();

}





void SetMotor2speed(int y){

  md.setM2Speed(y);

  stopIfFault();

}



/*

y-velocity, x=PWM



y = 0.0074x - 0.7642 //RIGHT

x=(y+0.7642)/(0.0074)



y = 0.0073x - 0.7058 //left

x=(y+0.7058)/(0.0073)







*/

void setup()  

{

  Serial.begin(9600);

  Serial.println("Dual MC33926 Motor Shield");

  md.init();

  time = millis();

}



int VelocityToPWM_Right(double velocity){

  return int((velocity+0.7642)/(0.0074));

}



int VelocityToPWM_Left(double velocity){

  return int((velocity+0.7058)/(0.0073));

}



void loop()

{
  /*
  if (j++==1)
    delta_v=0;
  
  */
  newLeftPos = -leftEnc.read();

  newRightPos = rightEnc.read(); 

  double delta_left = newLeftPos-oldLeftPos;

  double delta_right = newRightPos-oldRightPos;
    
  //Serial.print(delta_left);
  //Serial.print('\t');
  //Serial.print(delta_right);
  //Serial.print('\t\t');

  if (newLeftPos != oldLeftPos || newRightPos != oldRightPos){

    double d_l = wheel_distance(newLeftPos);

    double d_r = wheel_distance(newRightPos);

    double theta = heading_change(d_l,d_r);
    
    double displacement = robot_distance(d_l, d_r);
    //cumulative_displacement(theta,x,y,d_l,d_r);

    if (displacement>=24 && j==1){
      //START TURNING TILL THE HEADING REACHES 0.95
      
      theta=0;
      p=2.15;  
      j++;
      //go_straight(0,0);
      //delay(1000000);
    }
    
    if (j!=1){
      if (abs(theta)>=0.98){
        //STOP TURNING
        p=1.0;
        PWM_R=206;
        oldVelocity_Right = PWM_R*0.0073-0.7642;
        //j=1;
        //displacement=0;
      }
    }
    if(displacement>=87){
      go_straight(0,0);
      delay(100000);
    }

    Serial.print(displacement); 
    Serial.print(' ');
    //Serial.print(j); 
    Serial.print(' ');
    Serial.println(theta);
    oldLeftPos = newLeftPos;

    oldRightPos = newRightPos;
/*
    Serial.print(newLeftPos);

    Serial.print(' ');

    Serial.print(newRightPos);  
  
    Serial.print(' ');
    
    Serial.print(d_l);

    Serial.print(' ');

    Serial.print(d_r);

    Serial.print(' ');

    Serial.println(theta);
*/
  }

  //go_straight(200,210);

  double cur_time = millis();

  long delta_t = cur_time - time;

  double v_left = delta_left/delta_t;

  double v_right = delta_right/delta_t;

  error = p*v_left - c*v_right;
/*
  Serial.print(delta_left);
  Serial.print('\t');
  Serial.println(delta_right);
  */
  

  //error = delta_left-c*delta_right;

  delta_v = -K*(error)-B*(error-oldError);

  time = cur_time;

  oldError = error;

  

  //COMPSATE DELTA V ON RIGHT, 

  //IF DELTA_V -> +VE => RIGHT IS GOING FAST,

  //

  

  double newVelocity_Right=oldVelocity_Right-delta_v/2;
  double newVelocity_Left=oldVelocity_Left+delta_v/2;
  
  
  int temp1=VelocityToPWM_Right(newVelocity_Right);
  int temp2=VelocityToPWM_Left(newVelocity_Left);
  /*
  Serial.print(delta_v/2);
  Serial.print('\t');
  Serial.println(temp2);
  Serial.flush();
  */
 // Serial.println(newVelocity_Right);


  /*

  Serial.println(PWM_Correction_Right);

  Serial.print('\t');

  

  Serial.print(PWM_L);

  Serial.print('\t\t');

  Serial.print(delta_v);

  Serial.print('\t');

  Serial.println(oldError);

  //Serial.println('\t');

  */

  go_straight(temp2,temp1);
  //oldVelocity_Left = newVelocity_Left*0.0074-0.7058;

  //oldVelocity_Right = newVelocity_Right*0.0073-0.7642;
  

  delay(100);  

}



void go_straight(int m_l, int m_r){

  SetMotor1speed(m_l);

  SetMotor2speed(m_r);

}



double wheel_distance(int counter){

  /*

  The distance traveled by a wheel, i.e. delta_s(i).



  Args:

    counter: the counter from quadrature encoder, integer.

    circumference: the circumference of the wheel, positive real number.

    n_sector: the number of sectors on the disk, positive integer.

  Returns:

    A float representing the distance traveled by this wheel, in inches.

  */

  return (counter%(n_sector*2)/double(n_sector*2))*circumference

      +int(counter/(n_sector*2))*circumference;

}



double robot_distance(double s_left, double s_right){

  /*

  The distance traveled by the robot, i.e. delta_x.



  Args:

    s_left: the distance traveled by the left wheel.

    s_right: the distance traveled by the right wheel.



  Returns:

    The distance trabeled by this robot.

  */

  return (s_left+s_right)/2;

}



double heading_change(double s_left, double s_right){

  /*

  The heading change of the robot, i.e. delta_theta.



  Args:

    s_left: the distance traveled by the left wheel.

    s_right: the distance traveled by the right wheel.

    wheel_base: wheel base of the robot.



  Returns:

    The heading change in radians.

  */

  return atan2((s_left-s_right)/2, wheel_base/2);

}



double* cumulative_displacement(double theta, double x, double y, double s_left, double s_right){

  /*

  The new culmulative displacement given previous culmulative displacement

  theta, x, and y.



  Args:

    theta: the latest theta, in radians.

    x: the latest x.

    y: the latest y.

    s_left: the distance traveled by the left wheel.

    s_right: the distance traveled by the right wheel.



  Returns:

    A tuple of the new theta, x, and y, i.e. (new_theta, new_x, new_y).

  */

  static double values[3];

  double delta_theta = heading_change(s_left,s_right);

  double delta_x = robot_distance(s_left,s_right);

  double new_theta = theta + delta_theta;

  values[0] = new_theta;

  values[1] = x+delta_x*cos(new_theta);

  values[2] = y+delta_x*sin(new_theta);

  return values;

}
