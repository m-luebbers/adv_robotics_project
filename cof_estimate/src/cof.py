#!/usr/bin/env python
import rospy
from std_msgs.msg import Vector3

def callback(data):
    rospy.loginfo()
    
def cof():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('cof', anonymous=True)

    rospy.Subscriber("sensor_msgs/Imu", String, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cof()