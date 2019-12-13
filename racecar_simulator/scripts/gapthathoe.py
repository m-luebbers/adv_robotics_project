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
# from ackermann_msgs.msg import AckermannDriveStamped
from ros_pololu_servo.msg import MotorCommand
# from sensor_msgs.msg import LaserScan
import numpy as np
import time
servo_commands = MotorCommand()
drive_commands = MotorCommand()
# DC Motor Min 0.1 Max 0.45 (0.62 but we don't see a change)
servo_commands.joint_name = 'servo' #Check name here
drive_commands.joint_name = 'motor' #Check name here
drive_commands.position = 0 #Set speed here
drive_commands.speed = 0
# acceleration for best run = .2
drive_commands.acceleration = .07
servo_commands.position = 0
servo_commands.speed = 0.2

servo_commands.acceleration = 0.05
turn_number = 0
s_angle = 0
pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=1)
#pub.publish(drive_commands)
raw_input("Press Enter to RUMBLEEEE")
#State Machine Variable
car_state = 1

max_speed = .3
# min_speed of best run = .05
min_speed = -.1
t_prev = 0
x_right_prev = 0
error_I = 0
time_start = time.time()
error_previous = 0
def PID(servo_error, previous_angle, dt):
    global error_I
    global error_previous

    max_angle = 0.54
    # kp of best run = .02
    kp = 0.015/100
    ki = 0.00
    kd = 0.03/600
    error_I += servo_error*dt
    error_D = (servo_error-error_previous)/dt
    u = (kp*servo_error + ki*error_I + kd*error_D)
    #print("u",u)
    error_previous = servo_error
    new_angle = max(min(u,max_angle), -max_angle)
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
	global t_prev
	global x_turn_prev
	global time_start
	turning = False
	turn_angle = .54
	turn_thresh = .8
	N = 100
	x = []
	turning_speed = 0.1

    #TODO: this chunk of commented code can replace the 5 lines below if there aren't a ton of zeros
#    lx = len(data.data)
 #   for ix in range(N):
#	    bin_vals = data.data[int(round(lx*ix/N)):int(round(lx*(ix+1)/N))]
#	    if ix == N-1:
#		std_right = np.std(bin_vals)
#	    elif ix == N-2:
#		range_right = max(bin_vals)-min(bin_vals)
#		std_right_2 = np.std(bin_vals)
#		#print(bin_vals)
#	    bin_vals = [val for val in bin_vals if val != 0]
#	    if len(bin_vals) > 0:
#	        x.append(np.average(bin_vals))
#	    else:
#		x.append(100)

	x_zeroless = [ix for ix in data.data if ix != 0]
	lx = len(x_zeroless)
	for ix in range(N):
		bin_vals = x_zeroless[int(round(lx*ix/N)):int(round(lx*(ix+1)/N))]
		x.append(np.average(bin_vals))
		if ix == N-1:
			std_right = np.std(bin_vals)
    #x_turn = np.average(x_zeroless[-6:-1])

    #x_red[0] is far left and x_red[end] is far right
    #Min value is 100 for reading
    #Servo -0.54(left) to + 0.54(right)
	
	index_max = np.argmax(x)	
	center_bin_delta = index_max - round(N/2)

    # divide by 1000 to convert mm to m
	x_right = x[-1]
	x_left = x[0]
	x_mid = x[int(round(N/2))]
	right_minus_left = x_right - 2.5 #x_left

	if t_prev == 0:
		dt = 0
        #d_x_turn = 0
		t_prev = time.time()
		time_start = time.time()
	else:
		t_new = time.time()
		dt = t_new - t_prev
		t_prev = t_new
        #d_x_turn = x_turn - x_turn_prev

    #x_turn_prev = x_turn

	if x_mid < 0.75 and x_left < 0.75 and x_right < 0.75:
		if car_state == 3 or car_state == 4:
			car_state = 4
		else:
			car_state = 0
	    #car_state = 1

	drive_commands.position = min(x_mid/10*max_speed + min_speed,max_speed)

	if car_state == 1:     #First straightaway
		s_angle = PID(center_bin_delta,s_angle,dt)
        # if x_mid < 4.75 and x_left < 3 and x_right > 4.5: #TODO: modify this condition
		if std_right > turn_thresh and x_mid < 6 and time.time()-time_start > 0.4: # or std_right_2 > turn_thresh:
			print("state 1.5")
			print(" X_right = ", x_right,"X_mid = ", x_mid, " X Left = ", x_left)
			car_state = 1.5 #enters first turn
			print("std_right", std_right)
	elif car_state == 1.5: #First turn
		s_angle = turn_angle
		drive_commands.position -= turning_speed
		if x_mid > 7: #5o to the next straight
			print("State 2")
			car_state = 2
			print(" X_right = ", x_right,"X_mid = ", x_mid, " X Left = ", x_left)
	elif car_state == 2: #Second straightaway
        # right_minus_left = x_right - 2
        #TODO: choose the best way to do this.. semi middling, semi right-wall following?
		right_minus_left = x_right - 2.5 #x_left/3 - 1.3
		s_angle = PID(right_minus_left,s_angle,dt)
		if std_right > turn_thresh and x_mid < 5: # or std_right_2 > turn_thresh:
        # if x_mid < 4.5 and x_left < 4 and x_right > 4:  #TODO: modify this condition
			car_state = 2.5 #enters turning corner
			print("state 2.5")
			print(" X_right = ", x_right,"X_mid = ", x_mid, " X Left = ", x_left)
			print("std right", std_right)
	elif car_state == 2.5: #Second turn
		s_angle = turn_angle
		drive_commands.position -= turning_speed
		if x_mid > 8: #Makes the car go to the next straight
			car_state = 3
			print("State 3")
			print(" X_right = ", x_right,"X_mid = ", x_mid, " X Left = ", x_left)
	elif car_state == 3:     #Last straightaway
		s_angle = PID(right_minus_left,s_angle,dt)
	elif car_state == 0:
		print("oh fuckkkkkkk")
		drive_commands.position = 0
		servo_commands.position = 0
	elif car_state == 4:
		print("oh yeahhhhhhhhh")
		drive_commands.position = 0
		servo_commands.position = 0

	servo_commands.position = s_angle
	pub.publish(servo_commands)
	pub.publish(drive_commands)
#    print("d_x_turn",d_x_turn)
#	print("car_state",car_state)
#    print(" X_right = ", x_right,"X_mid = ", x_mid, " X Left = ", x_left)
	print("s_angle =", s_angle)
#    print("-------------------------")
#    print("std_right", std_right)
#    print("motors peed", drive_commands.position)

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
		print('boobs')
		drive_commands.position=0
	rospy.spin()