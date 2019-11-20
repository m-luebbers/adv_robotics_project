#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
# from ackermann_msgs.msg import AckermannDriveStamped
from ros_pololu_servo.msg import MotorCommand
# from sensor_msgs.msg import LaserScan
import numpy as np
servo_commands = MotorCommand()
drive_commands = MotorCommand()
    #string joint_name
    #float64 position
    #float32 speed
    #float32 acceleration
    # DC Motor Min 0.1 Max 0.45 (0.62 but we don't see a change)
servo_commands.joint_name = 'servo' #Check name here
drive_commands.joint_name = 'motor' #Check name here
drive_commands.position = 0 #Set speed here
drive_commands.speed = 0
drive_commands.acceleration = 0
servo_commands.position = 0
servo_commands.speed = 0
servo_commands.acceleration = 0
turn_number = 0
s_angle = 0

#State Machine Variable
car_state = 1

pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=10)
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
    
    max_angle = 0.54
    kp = .1
    ki = .02
    kd = -.001
    error_I += servo_error*dt
    error_D = (servo_error-error_previous)/dt
    u = -(kp*servo_error + ki*error_I + kd*error_D)
    # print(u)
    error_previous = servo_error
    new_angle = max(min(u,max_angle), -max_angle)
    # return new_angle
    if abs(new_angle - previous_angle) > .05:
        return new_angle
    else:
        return previous_angle

def callback(data):
    global servo_commands
    global drive_commands
    global s_angle
    global car_state
    global t_prev
    global x_right_prev

    turn_angle = -.4
    
    N = 50 #Gives bin of 10 per
    x = []


    #TODO: this chunk of commented code can replace the 5 lines below if there aren't a ton of zeros
    # lx = len(data.data)
    # for ix in range(N):
	#     bin_vals = data.data[int(round(lx*ix/N)):int(round(lx*(ix+1)/N))]
	#     x.append(np.average([iy for iy in bin_vals if iy != 0]))

    x_zeroless = [ix for ix in data.data if ix != 0]
    lx = len(x_zeroless)
    for ix in range(N):
	    bin_vals = x_zeroless[int(round(lx*ix/N)):int(round(lx*(ix+1)/N))]
	    x.append(np.average([iy for iy in bin_vals]))

    #x_red[0] is far left and x_red[end] is far right
    #Min value is 100 for reading
    #Servo -0.54(left) to + 0.54(right)

    # divide by 1000 to convert mm to m
    x_right = x[-1]/1000
    x_left = x[0]/1000
    x_mid = x[int(round(N/2))]/1000
    right_minus_left = x_right - x_left

    if t_prev == 0:
        dt = 0
        d_x_right = 0
        t_prev = time.time()
    else:
        t_new = time.time()
        dt = t_new - t_prev
        t_prev = t_new
        d_x_right = x_right - x_right_prev
    
    x_right_prev = x_right

    if car_state == 1:     #First straightaway
        s_angle = PID(right_minus_left,s_angle,dt)
        # if x_mid < 4.75 and x_left < 3 and x_right > 4.5: #TODO: modify this condition
        if d_x_right > 2.5:
            car_state = 1.5 #enters first turn        
    elif car_state == 1.5: #First turn
        s_angle = turn_angle
        if x_mid > 8: #Makes the car go to the next straight
            car_state = 2
    elif car_state == 2:#Second straightaway
        # right_minus_left = x_right - 2
        #TODO: choose the best way to do this.. semi middling, semi right-wall following?
        right_minus_left = x_right - x_left/2 - 1
        s_angle = PID(right_minus_left,s_angle,dt)
        if d_x_right > 2.5:
        # if x_mid < 4.5 and x_left < 4 and x_right > 4:  #TODO: modify this condition
            car_state = 2.5 #enters turning corner
    elif car_state == 2.5: #Second turn
        s_angle = turn_angle
        if x_mid > 8: #Makes the car go to the next straight
            car_state = 3
    elif car_state == 3:     #Last straightaway
        s_angle = PID(right_minus_left,s_angle)    
    
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
    #pub.publish(drive_commands)
    print(" X_right = ", x_right, " X Left = ", x_left," X Mid = ", x_mid)
    print("State =",car_state)
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
