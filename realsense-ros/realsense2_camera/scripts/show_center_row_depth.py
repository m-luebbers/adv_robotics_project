#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            depth_row = []
            #For a full row
            for i in range(data.width):
                #Gets all the values of a single row in the middle of the frame
                depth_row = np.append(depth_row,cv_image[pix[1],i])
            	sys.stdout.write('%s: Depth at center Row, Column %d: %f(mm)\n' % (self.topic, i, depth_row[i]))
            #####
            sys.stdout.flush()
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
