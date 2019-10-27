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
servo_commands.joint_name = 'servo' #Check name here
drive_commands.joint_name = 'drive' #Check name here
drive_commands.position = 0
drive_commands.speed = 0
drive_commands.acceleration = 0
servo_commands.position = 0



#pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

    # rate.sleep()    

def callback(data):
    lx = len(data.data)
    x_red = []
    N = 10
    for ix in range(N):
	bin_vals = data.data[int(round(lx*ix/N)):int(round(lx*(ix+1)/N))]
	x_red.append(np.average([x for x in bin_vals if x >= 10]))
    
    print("ranges left to right", x_red)


def depth_data_processor():
    #Data from the realsense
    rospy.init_node("depth_data_processor")
    rospy.Subscriber('/depth_row', Float32MultiArray, callback)


if __name__ == '__main__':
    try:
        depth_data_processor()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
