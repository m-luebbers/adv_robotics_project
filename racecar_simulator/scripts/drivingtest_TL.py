#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np

pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
drive_msg = AckermannDriveStamped()
drive_msg.header.seq = 0
drive_msg.header.stamp = rospy.Time()
drive_msg.header.frame_id = 'test'
#(-) is to the right (+) is to the left
s_angle = 0
drive_msg.drive.steering_angle = s_angle
drive_msg.drive.steering_angle_velocity = 3
drive_msg.drive.speed = 11
drive_msg.drive.acceleration = 10
drive_msg.drive.jerk = 0
car_state = 0
    # rate.sleep()    

def callback(data):
    global s_angle
    global car_state
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    x = [min(xi,10) for xi in data.ranges]

    length_x = len(x)
    x_right = x[0:int(round(length_x/5))]
    x_right_mid = x[int(round(length_x/5)):int(round(length_x*2/5))]
    x_mid = x[int(round(length_x*2/5)):int(round(length_x*3/5))]
    x_left_mid = x[int(round(length_x*3/5)):int(round(length_x*4/5))]
    x_left = x[int(round(length_x*4/5)):length_x]
    print("Average value of x_right", np.average(x_right))
    print("Average value of x_right_mid", np.average(x_right_mid))
    print("Average value of x_mid", np.average(x_mid))
    print("Average value of x_left_mid", np.average(x_left_mid))
    print("Average value of x_left_mid", np.average(x_left))
    if car_state == 0:
        if np.average(x_right) < 1.5:
            # drive_msg.drive.speed = 0
            s_angle = s_angle + 0.3
            drive_msg.drive.steering_angle = s_angle
            print("Turning Hard Left")
        elif np.average(x_right_mid) < 2:
            s_angle = s_angle + 0.1
            drive_msg.drive.steering_angle = s_angle
            print("Turning Left")
        elif np.average(x_left_mid) < 1.5:
            s_angle = s_angle - 0.1
            drive_msg.drive.steering_angle = s_angle
            print("Turning Right")
        elif np.average(x_left) < 1:
            s_angle = s_angle - 0.3
            drive_msg.drive.steering_angle = s_angle
            print("Turning Hard Right")
        else:
            s_angle = 0
            drive_msg.drive.steering_angle = s_angle
            print("dRIVING sTRIGHT")
        print(s_angle)
        if np.average(x_mid) < 5 and np.average(x_left) < 4 and np.average(x_right) > 4:
            car_state = 1
        if np.average(x_mid) < 1 and np.average(x_left) < 1 and np.average(x_right) < 1:
            car_state = 2 #Stops the car since all sides are too close to the car
    elif car_state == 1:
        drive_msg.drive.steering_angle = -0.1
        pub.publish(drive_msg)
        print("Turing this bitch")
        print("Average value of x_right", np.average(x_right))
        print("Average value of x_left", np.average(x_left))
        print("Average value of x_mid", np.average(x_mid))
        if np.average(x_mid) < .5 and np.average(x_left) < .5 and np.average(x_right) < .5:
            car_state = 2
        if np.average(x_mid) > 8 and np.average(x_left) < 4 and np.average(x_right) < 4:
            car_state = 0
            print("We made the fucking turn")
    elif car_state == 2:
        print("MADE IT BITCH")
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.speed = 0


    pub.publish(drive_msg)
    # print(drive_msg.drive.speed)
     


def scanner(pub, drive_msg):

    #This is for the simulator
    rospy.init_node("scanner")
    rospy.Subscriber('/scan', LaserScan, callback)


if __name__ == '__main__':
    try:
        scanner(pub,drive_msg)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()