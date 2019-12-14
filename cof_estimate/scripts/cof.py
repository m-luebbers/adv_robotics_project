#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import sqrt

mass = 2 #need to measure
weight = 2*9.8
first = True
gravity = [0,0,1]


def imu_data(whole_data):
    data = whole_data.linear_acceleration
    acc = [data.x, data.y, data.z]
    #print(acc)
    print("acc is", acc[0], acc[1], acc[2])

    global first
    global gravity 

    if first == True:
    	first = False
    	gravity = acc

    true_acc = [acc[0] - gravity[0], acc[1] - gravity[1], acc[2] - gravity[2]]
    acc_abs = sqrt(true_acc[0]*true_acc[0] + true_acc[1]*true_acc[1] + true_acc[2] * true_acc[2])
    friction_force = acc_abs * mass
    cof = friction_force / weight
    print("cof is",cof)
    
def cof():
    rospy.init_node('cof', anonymous=True)

    rospy.Subscriber("imu/data", Imu, imu_data)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cof()
