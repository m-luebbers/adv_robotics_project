#!/usr/bin/env python
import rospy
from geometry_msgs import Vector3

first = True
gravity = [0,0,1]

def acc_data(data):
    acc = [data.x, data.y, data.z]
    print(acc)
    print("here is the current cof estimation")

    global first
    global gravity 

    if first == True:
    	first = False
    	gravity = acc

    true_acc = [acc[0] - gravity[0], acc[0] - gravity[0], acc[0] - gravity[0]]

    
def cof():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('cof', anonymous=True)

    rospy.Subscriber("rmu_data/raw", Vector3, acc_data)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()



if __name__ == '__main__':
    cof()
