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
drive_commands.position = 0.05 #Set speed here
drive_commands.speed = 0
drive_commands.acceleration = 0
servo_commands.position = 0
servo_commands.speed = 0
servo_commands.acceleration = 0
turn_number = 0

#State Machine Variable
car_state = 1

pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=10)
#pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

    # rate.sleep()    
at_pillar = False
past_pillar = False
    # rate.sleep()    
def PID(servo_error, previous_angle):
    max_angle = 0.54
    kp = .1
    if abs(servo_error) < 0.1:
        new_angle = previous_angle
    else:
        u = kp * servo_error
        print(u)
        new_angle = max(min(u,max_angle), -max_angle)
    return new_angle

def callback(data):
    global servo_commands
    global drive_commands
    global car_state
    global turn_number
    global at_pillar
    global past_pillar
    
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
    x_mid = x[round(N/2)]/1000
    right_minus_left = x_right - x_left

    if car_state == 1:     #First straightaway
        s_angle = PID(right_minus_left,s_angle)
        if x_mid < 4.75 and x_left < 3 and x_right > 4.5: #TODO: modify this condition
            car_state = 1.5 #enters first turn        
    elif car_state == 1.5: #First turn
        s_angle = -0.4
        if x_mid > 8: #Makes the car go to the next straight
            car_state = 2
    elif car_state == 2:#Second straightaway
        if x_left < 6:
            s_angle = PID(right_minus_left,s_angle)
        else:
            car_state = 2.25 #enters avoiding the open space
    elif car_state == 2.25:# Open wall straightaway
        print("value of x_left ", x_left)
        global at_pillar, past_pillar
        right_minus_left = x_right - 2
        s_angle = PID(right_minus_left,s_angle)
        if x_left < 3 and not past_pillar:
            at_pillar = True
        elif x_left > 3 and at_pillar:
            past_pillar = True
        elif x_left < 2.5 and past_pillar:
            car_state = 2.5
        # if x_mid > 4.75 and x_left < 3 and x_right < 3 and t > 1.5:  #TODO: modify this condition
        #     car_state = 2.5 #enters second straight post opening          
    elif car_state == 2.5:#Second straightaway
        s_angle = PID(right_minus_left,s_angle)
        if x_mid < 4 and x_left < 4 and x_right > 4:  #TODO: modify this condition
            car_state = 2.75 #enters turning corner        
    elif car_state == 2.75: #Second turn
        s_angle = -0.4
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
    pub.publish(drive_commands)
    print(" X_right = ", x_right, " X Left = ", x_left," X Mid = " x_mid)


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
