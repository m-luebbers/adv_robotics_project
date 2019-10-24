#!/usr/bin/env python

import random
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

drive_msg = AckermannDriveStamped()
drive_msg.header.seq = 0
drive_msg.header.stamp = rospy.Time()
drive_msg.header.frame_id = 'test'
drive_msg.drive.steering_angle = 0
drive_msg.drive.steering_angle_velocity = 1
drive_msg.drive.speed = 3
drive_msg.drive.acceleration = 0
drive_msg.drive.jerk = 0
pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

def driver_callback(data):

    global s_angle
    global car_state
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    x = [min(xi,max_range) for xi in data.ranges]
    lx = len(x)
    N = 5
    x_r = []
    for i in range(N):
        x_r[i] = np.average(x[int(lx*i/N):int(lx*(i+1)/N)])
        print("x_r at",i,"is",x_r(i))

    global drive_msg
    ranges = data.ranges
    ranges = [r for r in ranges if r <=10]
    if len(ranges) == 0:
        return
    # rospy.loginfo(ranges)
    if min(ranges) < .3:
        drive_msg.drive.speed = -1
        drive_msg.drive.steering_angle = random.random() - 0.9
    elif min(ranges) > 1:
        drive_msg.drive.speed = 3
        drive_msg.drive.steering_angle = random.random() - 0.5
    # rospy.loginfo(drive_msg.drive.steering_angle)
    drive_msg.header.stamp = rospy.Time()
    pub.publish(drive_msg)

def odom_callback(data):
    pos = data.pose.pose.position
    ori = data.pose.pose.orientation
    rospy.loginfo(pos)


def scanner():
    rospy.init_node('scanner')
    rospy.Subscriber('/scan', LaserScan, driver_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        scanner()
    except rospy.ROSInterruptException:
        pass