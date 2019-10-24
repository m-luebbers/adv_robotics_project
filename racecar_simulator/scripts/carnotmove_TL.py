#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

def driver():
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
    rospy.init_node('driver', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    speed_counter = 0
    while not rospy.is_shutdown():
        msg = AckermannDriveStamped()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time()
        msg.header.frame_id = 'test'
        msg.drive.steering_angle = 0
        msg.drive.steering_angle_velocity = 0
        msg.drive.speed = speed_counter
        msg.drive.acceleration = 0
        msg.drive.jerk = 0
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()    








if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass