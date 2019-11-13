#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import sys
import os
import numpy as np

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.pub = rospy.Publisher('depth_row', Float32MultiArray, queue_size=10)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))

            #avg row init
            depth_row = Float32MultiArray()
            depth_row.data = []
            #For a full row
            for i in range(data.width):
		count = 3.0
		if(cv_image[pix[1],i] == 0.0):
			count = count - 1.0
                if(cv_image[pix[1]-5,i] == 0.0):
			count = count - 1.0
		if(cv_image[pix[1]-10,i] == 0.0):
			count = count - 1.0
		if(count == 0.0):
			depth_row.data = np.append(depth_row.data, 0.0)
		else:
			depth_row.data = np.append(depth_row.data, ((cv_image[pix[1],i]+cv_image[pix[1]-5,i]+cv_image[pix[1]-10,i])/count))
                #Gets all the values from three discrete rows from the middle up and averages them
		#depth_row.data = np.append(depth_row.data, cv_image[pix[1],i])
               # depth_row.data = np.append(depth_row.data, ((cv_image[pix[1],i]+cv_image[pix[1]-5,i]+cv_image[pix[1]-10,i])/3.0))
                #sys.stdout.write('%s: Depth at center Row, Column %d: %f(mm)\n' % (self.topic, i, depth_row[i]))
        
            #####
            self.pub.publish(depth_row)           
            #sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return

def main():
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
