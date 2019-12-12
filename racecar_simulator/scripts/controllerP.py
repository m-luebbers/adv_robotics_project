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
<<<<<<< HEAD
from ros_pololu_servo.msg import MotorCommand
=======
# from ackermann_msgs.msg import AckermannDriveStamped
from ros_pololu_servo.msg import MotorCommand
# from sensor_msgs.msg import LaserScan
>>>>>>> b3d33bc610d80c5d83b6a6dbe5851b8287471114
import numpy as np
import time
servo_commands = MotorCommand()
drive_commands = MotorCommand()
    #string joint_name
    #float64 position
    #float32 speed
    #float32 acceleration
    # DC Motor Min 0.1 Max 0.45 (0.62 but we don't see a change)
servo_commands.joint_name = 'servo' #Check name here
drive_commands.joint_name = 'motor' #Check name here
#
drive_commands.position = 0 #Set speed here
#
drive_commands.speed = 0
drive_commands.acceleration = 0
servo_commands.position = 0
servo_commands.speed = 0.2
servo_commands.acceleration = 0
turn_number = 0
s_angle = 0

#State Machine Variable
car_state = 1

<<<<<<< HEAD
pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=1)
=======
pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=10)
>>>>>>> b3d33bc610d80c5d83b6a6dbe5851b8287471114
#pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

    # rate.sleep()    
t_prev = 0 
x_right_prev = 0
error_I = 0
error_previous = 0
    # rate.sleep()    
def PID(servo_error, previous_angle, dt):
    global error_I
    global error_previous

<<<<<<< HEAD
    max_angle = 0.54
    kp = 0.005
=======
    max_angle = 0.54-.14
    kp = 0.01
>>>>>>> b3d33bc610d80c5d83b6a6dbe5851b8287471114
    ki = 0.00
    kd = 0.01
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

    turning = False
    turn_angle = .4
    turn_thresh = 2.5
    N = 25
    x = []


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
    
    x_turn = np.average(x_zeroless[-4:-1])

    #x_red[0] is far left and x_red[end] is far right
    #Min value is 100 for reading
    #Servo -0.54(left) to + 0.54(right)

    # divide by 1000 to convert mm to m
    x_right = x[-1]
    x_left = x[0]
    x_mid = x[int(round(N/2))]
    right_minus_left = x_right - x_left

    if t_prev == 0:
        dt = 0
        d_x_turn = 0
        t_prev = time.time()
    else:
        t_new = time.time()
        dt = t_new - t_prev
        t_prev = t_new
        d_x_turn = x_turn - x_turn_prev

    x_turn_prev = x_turn

    if x_mid < 0.75 and x_left < 0.75 and x_right < 0.75:
	if car_state == 3 or car_state == 4:
	    car_state = 4
	else:
	    car_state = 0
	    #car_state = 1

    if car_state == 1:     #First straightaway
        s_angle = PID(right_minus_left,s_angle,dt)
        # if x_mid < 4.75 and x_left < 3 and x_right > 4.5: #TODO: modify this condition
        if d_x_turn > turn_thresh: # or std_right_2 > turn_thresh:
            car_state = 1.5 #enters first turn
	    turning = True
	    print("state 1.5")
    elif car_state == 1.5: #First turn
        s_angle = turn_angle
<<<<<<< HEAD
        if x_mid > 7: #5o to the next straight
=======
        if x_right > 5: #5o to the next straight
>>>>>>> b3d33bc610d80c5d83b6a6dbe5851b8287471114
	    print("State 2")
            car_state = 2
    elif car_state == 2:#Second straightaway
        # right_minus_left = x_right - 2
        #TODO: choose the best way to do this.. semi middling, semi right-wall following?
        right_minus_left = x_right - 2#x_left/3 - 1.3
        s_angle = PID(right_minus_left,s_angle,dt)
        if d_x_turn > turn_thresh: # or std_right_2 > turn_thresh:
        # if x_mid < 4.5 and x_left < 4 and x_right > 4:  #TODO: modify this condition
            car_state = 2.5 #enters turning corner
	    turning = True
    elif car_state == 2.5: #Second turn
        s_angle = turn_angle
        if x_mid > 8: #Makes the car go to the next straight
            car_state = 3
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
    if turning:
	print("waiting")
#	time.sleep(.2)
    #pub.publish(drive_commands)
<<<<<<< HEAD
#    print("d_x_turn",d_x_turn)
    print("car_state",car_state)
    print(" X_right = ", x_right,"X_mid = ", x_mid, " X Left = ", x_left)
    print("s_angle =", s_angle)
#    print("-------------------------")
=======
    print("d_x_turn",d_x_turn)
    print("car_state",car_state)
    print(" X_right = ", x_right,"X_mid = ", x_mid, " X Left = ", x_left)
    print("s_angle =", s_angle)
    print("-------------------------")
>>>>>>> b3d33bc610d80c5d83b6a6dbe5851b8287471114
#    print("d_x_right", d_x_right)
def depth_data_processor():
    #Data from the realsense
    rospy.init_node("depth_data_processor")
    rospy.Subscriber('/depth_row', Float32MultiArray, callback)


if __name__ == '__main__':
    try:
        depth_data_processor()
    except KeyboardInterrupt:
	print('boobs')
        drive_commands.position=0
	pub.publish(drive_commands)
    rospy.spin()
