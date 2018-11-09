import numpy as np

def wheel_distance(counter,circumference=(8+5/8),n_sector=40):
	'''
	The distance traveled by a wheel, i.e. delta_s(i).

	Args:
		counter: the counter from quadrature encoder, integer.
		circumference: the circumference of the wheel, positive real number.
		n_sector: the number of sectors on the disk, positive integer.
	Returns:
		A float representing the distance traveled by this wheel, in inches.
	'''
	return (counter%(n_sector*2)/(n_sector*2))*circumference \
			+int(counter/(n_sector*2))*circumference

def robot_distance(s_left,s_right):
	'''
	The distance traveled by the robot, i.e. delta_x.

	Args:
		s_left: the distance traveled by the left wheel.
		s_right: the distance traveled by the right wheel.

	Returns:
		The distance trabeled by this robot.
	'''
	return (s_left+s_right)/2

def heading_change(s_left,s_right,wheel_base = 4):
	'''
	The heading change of the robot, i.e. delta_theta.

	Args:
		s_left: the distance traveled by the left wheel.
		s_right: the distance traveled by the right wheel.
		wheel_base: wheel base of the robot.

	Returns:
		The heading change in radians.
	'''
	return np.arctan2((s_right-s_left)/2, wheel_base/2)

def culmulative_displacement(theta,x,y,s_left,s_right):
	'''
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
	'''
	delta_theta = heading_change(s_left,s_right)
	delta_x = robot_distance(s_left,s_right)
	new_theta = theta + delta_theta
	return (new_theta,x+delta_x*np.cos(new_theta),y+delta_x*np.sin(new_theta))