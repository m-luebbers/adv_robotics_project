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

def callback(data):
    global servo_commands
    global drive_commands
    global car_state
    global turn_number
    lx = len(data.data)
    x_red = []
    N = 5
    for ix in range(N):
	bin_vals = data.data[int(round(lx*ix/N)):int(round(lx*(ix+1)/N))]
	x_red.append(np.average([x for x in bin_vals if x >= 10]))
    #x_red[0] is far left and x_red[end] is far right
    #Min value is 100 for reading
    #Servo -0.54(left) to + 0.54(right)
    #Threshold values for turning
    th1 = 1300
    th2 = 1000
    th3 = 700
    th4 = 500
    if car_state == 1: #Driving straight in the hallway
        #Turns car to the right
        if x_red[0] < th1 and x_red[2] > th1 and x_red[-1] > th1:
            print("Turing Right")
            if x_red[0] < th4:
                servo_commands.position = 0.4
            elif x_red[0] < th3:
                servo_commands.position = 0.3
            elif x_red[0] < th2:
                servo_commands.position = 0.2
            else:
                servo_commands.position = 0.1
        #Turns car to the Left
        elif x_red[0] > th1 and x_red[2] > th1 and x_red[-1] < th1:
            print("Turing Left")
            if x_red[-1] < th4:
                servo_commands.position = -0.4
            elif x_red[-1] < th3:
                servo_commands.position = -0.3
            elif x_red[-1] < th2:
                servo_commands.position = -0.2
            else:
                servo_commands.position = -0.1
        #Turns car to the Straight
        elif x_red[0] > th1 and x_red[2] > th1 and x_red[-1] > th1:
            print("Driving Straight")
            servo_commands.position = 0
        #State transitions
	if turn_number == 0: #First Turn
	    t_val = 4000
	    if x_red[2] < t_val and x_red[0] < t_val+1000 and x_red[-1] < t_val:
		car_state = 2 # Going around the corner
	    if x_red[0] < th2 and x_red[1] < th2 and x_red[2] < th2 and x_red[3] < th2 and x_red[4] < th2:
		car_state = 0 # Stopping the car
        elif turn_number == 1: #Second Turn
	    t_val = 4000
	    if x_red[2] < t_val and x_red[0] < t_val and x_red[-1] < t_val:
		car_state = 3 # Going around the corner
	    if x_red[0] < th2 and x_red[1] < th2 and x_red[2] < th2 and x_red[3] < th2 and x_red[4] < th2:
		car_state = 0 # Stopping the car
        elif turn_number == 2: #Stopping
	    t_val = 4000
	    if x_red[2] < t_val and x_red[0] < t_val and x_red[-1] < t_val:
		car_state = 0 # Going around the corner
	    if x_red[0] < th2 and x_red[1] < th2 and x_red[2] < th2 and x_red[3] < th2 and x_red[4] < th2:
		car_state = 0 # Stopping the car
    #State for turing the corner 1
    elif car_state == 2:
	turn_number = 1
        print("Turing this Bitch around dat corner 1")
        servo_commands.position = 0.4
        if x_red[0] > 1000 and x_red[1] > 2000 and x_red[2] > 6000 and x_red[3] < 9000 and x_red[4] < 7000: 
            car_state = 1
            print("We Made it?")
        if x_red[0] < th2 and x_red[1] < th2 and x_red[2] < th2 and x_red[3] < th2 and x_red[4] < th2:
            car_state = 0 # Stopping the car
    #State for turing the corner 2
    elif car_state == 3:
	turn_number = 2
        print("Turing this Bitch around dat corner 2")
        servo_commands.position = 0.4
        if x_red[0] > 1000 and x_red[1] > 2000 and x_red[2] > 6000 and x_red[3] < 9000 and x_red[4] < 7000: 
            car_state = 1
            print("We Made it?")
        if x_red[0] < th2 and x_red[1] < th2 and x_red[2] < th2 and x_red[3] < th2 and x_red[4] < th2:
            car_state = 0 # Stopping the car
    #State for stopping the car
    elif car_state == 0:
        print("Either we Won or We Fucked up")
        drive_commands.position = 0 #Set speed here
        drive_commands.speed = 0
        drive_commands.acceleration = 0
        servo_commands.position = 0
        servo_commands.speed = 0
        servo_commands.acceleration = 0
    pub.publish(servo_commands)    
    pub.publish(drive_commands)
    print("ranges left to right", x_red)


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
