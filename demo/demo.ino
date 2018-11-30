#include "DualMC33926MotorShield.h"
#include <math.h>
#include <Encoder.h>

// constants for odometry functions
#define circumference 8.625
#define n_sector 24
#define wheel_base 6.5

// pins for encoder
#define r1 2
#define r2 11
#define l1 3
#define l2 13

// initialization of encoder
Encoder leftEnc(l1,l2);
Encoder rightEnc(r1,r2);
long oldLeftPos  = -999;
long oldRightPos  = -999;
long newLeftPos  = -999;
long newRightPos  = -999;

// v_ref, unit in PWM
int PWM_L=200;
int PWM_R=215;

// parameters of PD controller
#define K 30
#define B 5

// initialization of PD controller 
double error = 0;
double delta_v = 0;
double oldError = 0;
double PWM_Correction_Right = 0;
double PWM_Correction_Left = 0;

// PWM to inch/sec speed
double oldVelocity_Left = PWM_L*0.0074-0.7058;
double oldVelocity_Right = PWM_R*0.0073-0.7642;

long time;

DualMC33926MotorShield md;

/* initialize c and j, where c is the turning coefficient and
   j represents the current state in finite state machine */
double c=1;
int j=1;

void stopIfFault()

{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}


void SetMotor1speed(int x){
  // Set the motor 1 speed to x, where x units in PWM
  md.setM1Speed(x);
  stopIfFault();
}

void SetMotor2speed(int y){
  // Set the motor 2 speed to x, where x units in PWM
  md.setM2Speed(y);
  stopIfFault();

}

void go(int m_l, int m_r){
  // set the speeds of both motors
  SetMotor1speed(m_l);
  SetMotor2speed(m_r);

}

int VelocityToPWM_Right(double velocity){
/* For the right motor, take a velocity (unit:inch/sec) 
   and return its corresponding PWM
   */
  return int((velocity+0.7642)/(0.0074));
}

int VelocityToPWM_Left(double velocity){
/* For the right motor, take a velocity (unit:inch/sec) 
   and return its corresponding PWM
   */
  return int((velocity+0.7058)/(0.0073));
}

void setup()  

{
  Serial.begin(9600);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();
  time = millis();
}


void loop()

{
  newLeftPos = -leftEnc.read();
  newRightPos = rightEnc.read(); 
  double delta_left = newLeftPos-oldLeftPos;
  double delta_right = newRightPos-oldRightPos;
  
  // if either wheel rotates for at least one division
  if (newLeftPos != oldLeftPos || newRightPos != oldRightPos){
    // compute the current odometry
    double d_l = wheel_distance(newLeftPos);
    double d_r = wheel_distance(newRightPos);
    double theta = heading_change(d_l,d_r);
    double displacement = robot_distance(d_l, d_r);
    
    // finite state machine for demo
    if (displacement>=42 && j==1){
      //START TURNING TILL THE HEADING REACHES 1
      theta=0;
      c=3.2;  
      j++;
    }
    if (j!=1){
      if (abs(theta)>=1){
        //STOP TURNING
        c=1.0;
        // tune left wheel speed a little bit for better performance
        PWM_L=207;
        oldVelocity_Left = PWM_L*0.0074-0.7058;
      }
    }
    if(displacement>=72){
      // if reached destination, stay for a few minutes
      go(0,0);
      delay(100000);
    }

    Serial.print(displacement); 
    Serial.print(' ');
    Serial.print(j); 
    Serial.print(' ');
    Serial.println(theta);
    // update encoder readings
    oldLeftPos = newLeftPos;
    oldRightPos = newRightPos;
  }
  // PD controller computation
  double cur_time = millis();
  long delta_t = cur_time - time;
  double v_left = delta_left/delta_t;
  double v_right = delta_right/delta_t;
  error = v_left - c*v_right;
  delta_v = -K*(error)-B*(error-oldError);

  time = cur_time;

  oldError = error;

  //COMPSATE DELTA V ON RIGHT, 
  //IF DELTA_V -> +VE => RIGHT IS GOING FAST,

  // update new velocity and set them to the motors
  double newVelocity_Right=oldVelocity_Right-delta_v/2;
  double newVelocity_Left=oldVelocity_Left+delta_v/2;
  
  int temp1=VelocityToPWM_Right(newVelocity_Right);
  int temp2=VelocityToPWM_Left(newVelocity_Left);

  go(temp2,temp1);
  
  delay(100);  

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
