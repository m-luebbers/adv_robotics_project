#!/usr/bin/env python
#Revision V1 11-19-19
#Updated with PID controller inital test
#
#
#
#
#
import rospy
from std_msgs.msg import Float32MultiArray
from ros_pololu_servo.msg import MotorCommand
import numpy as np
import time

# Set constants
MAX_SPEED = 0.3
MIN_SPEED = 0
MAX_ANGLE = 0.54
SERVO_ACCELERATION = 0.04
DRIVE_ACCELERATION = 0.01
TURN_ANGLE = 0.54

## Configure servo command channel
# Servo -0.54(left) to + 0.54(right)
servo_commands = MotorCommand()
servo_commands.joint_name = 'servo' 
servo_commands.position = 0
servo_commands.speed = 0
servo_commands.acceleration = SERVO_ACCELERATION

## Configure drive motor command channel
# DC Motor Min 0.05 Max 0.45 (0.62 but we don't see a change)
drive_commands = MotorCommand()
drive_commands.joint_name = 'motor' 
drive_commands.position = 0 #Set speed here
drive_commands.speed = 0
drive_commands.acceleration = DRIVE_ACCELERATION


s_angle = 0

raw_input("Press Enter to RUMBLEEEE")

pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=1)

# initialize variables
car_state = 1
time_start = time.time()

def PID(servo_error, previous_angle):
    kp = 0.01/1.2
    u = kp*servo_error
    new_angle = max(min(u,MAX_ANGLE), -MAX_ANGLE)
    # return new_angle
    if abs(new_angle - previous_angle) > .03:
        return new_angle
    else:
        return previous_angle

def callback(data):
	global servo_commands
	global drive_commands
	global s_angle
	global car_state
	global time_start
	
	# remove all zero values from data
	x_zeroless = [ix for ix in data.data if ix != 0]

	# group zeroless depth_row data into N bins
	N = 100
	x = []
	lx = len(x_zeroless)
	for ix in range(N):
		bin_vals = x_zeroless[int(round(lx*ix/N)):int(round(lx*(ix+1)/N))]
		bin_vals_ave = min(np.average(bin_vals),10)
		x.append(bin_vals_ave)
	
	# find the gap and how far it is from center
	index_max = np.argwhere(x == np.amax(x))
	if len(index_max) > 1:
		index_max = np.average(index_max)
	center_bin_delta = index_max-2 - round(N/2)


	# scale speed by x_mid
	x_mid = x[int(round(N/2))]
	drive_commands.position = min(x_mid/10*MAX_SPEED + MIN_SPEED, MAX_SPEED)
	
	time_elapsed = time.time()-time_start
	if car_state == 1:				# first straightaway
		s_angle = PID(center_bin_delta,s_angle)
		if time_elapsed > 3: 		# wait a while before slowing down and turning
			drive_commands.acceleration = 0.02
			drive_commands.position = drive_commands.position*1.5/time_elapsed	# *2 worked pretty well
			if max(x) < 9.5:		# start turning when all values are below a threshold 
				print("state 1.5")
				car_state = 1.5
				time_start = time.time()
	elif car_state == 1.5:			# first turn
		servo_commands.acceleration = 0.04
		drive_commands.acceleration = 0.01
		if time_elapsed < 0.6: 
			s_angle = TURN_ANGLE
			drive_commands.position = 0.05
			print("Yo need to sloooow the fuck down")
		elif time_elapsed < 1:
			#s_angle = -TURN_ANGLE*.7
			drive_commands.acceleration = 0.01
			drive_commands.position = 0.1
			print("Hide your girl cause we Psliding into dat hole")
		else: #5o to the next straight
			print("State 2")
			car_state = 2
			time_start = time.time()
			servo_commands.acceleration = SERVO_ACCELERATION
			drive_commands.acceleration = DRIVE_ACCELERATION
	elif car_state == 2:			# second straightaway
		s_angle = PID(center_bin_delta,s_angle)
		if time_elapsed > 5:
			drive_commands.position = drive_commands.position*2.5/time_elapsed
			if max(x) < 8:
				car_state = 2.5 #enters turning corner
				print("state 2.5")
				time_start = time.time()
	elif car_state == 2.5:			# second turn
		servo_commands.acceleration = 0
		if time_elapsed < 0.6:
			s_angle = TURN_ANGLE
			drive_commands.position = 0.1
			print("Yo need to sloooow the fuck down too damn fast again")
		elif time_elapsed < 1:
			#s_angle = TURN_ANGLE
			drive_commands.acceleration = 0.01
			drive_commands.position = 0.1 
			print("Hide your girl cause we Psliding into dat other hole")
		else: #5o to the next straight
			print("State 3")
			car_state = 3
			time_start = time.time()
			servo_commands.acceleration = SERVO_ACCELERATION
			drive_commands.acceleration = DRIVE_ACCELERATION
	elif car_state == 3:     		# last straightaway
		s_angle = PID(center_bin_delta,s_angle)

	servo_commands.position = s_angle
	pub.publish(servo_commands)
	pub.publish(drive_commands)
#	print("motors angle", servo_commands.position)

def depth_data_processor():
    #Data from the realsense
	rospy.init_node("depth_data_processor")
	pub.publish(drive_commands)
	rospy.Subscriber('/depth_row', Float32MultiArray, callback)


if __name__ == '__main__':
	try:
		print("Fuck dicks bals")
		depth_data_processor()
	except KeyboardInterrupt:
		drive_commands.position=0
	rospy.spin()
