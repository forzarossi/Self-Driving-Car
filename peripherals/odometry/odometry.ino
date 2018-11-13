
#include <math.h>
#define circumference 8.625
#define n_sector 12
#define wheel_base 4

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Odometry Test:");
  double left = wheel_distance(8);
  double right = wheel_distance(0);
  double s = robot_distance(left,right);
  double theta = heading_change(left,right);
  double* returned = culmulative_displacement(0,0,0,left,right);
  Serial.print("Left wheel displacement: ");
  Serial.println(left,4);
  Serial.print("Right wheel displacement: ");
  Serial.println(right,4);
  Serial.print("Robot displacement: ");
  Serial.println(s,4);
  Serial.print("Heading change: ");
  Serial.println(theta,4);
  Serial.print("New theta: ");
  Serial.println(returned[0],4);
  Serial.print("New x: ");
  Serial.println(returned[1],4);
  Serial.print("New y: ");
  Serial.println(returned[2],4);
}

void loop() {
  // put your main code here, to run repeatedly:
  
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

double* culmulative_displacement(double theta, double x, double y, double s_left, double s_right){
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
