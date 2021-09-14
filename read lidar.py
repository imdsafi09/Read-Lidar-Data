
#!/usr/bin/env python
import rospy, math, random
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from laser_geometry import LaserProjection
from  sensor_msgs.msg import PointCloud2
from  sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
import os
import matplotlib.pyplot as plt
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
i =0
j=0
def get_point(scan):
    scan = ros_numpy.numpify(scan)
    global i

    PointCloud = np.zeros((scan.shape[-1],4))
    print(PointCloud.shape)
    PointCloud[:,0] = scan['x']
    PointCloud[:,1] = scan['y']
    PointCloud[:,2] = scan['z']
    PointCloud[:,3] = scan['intensity']
    np.set_printoptions(precision=3)
    len_of_i=len(str(i))
    filename_str=''
    for _ in range(6-len_of_i):
        filename_str=filename_str + '0'
        
    filename_str=filename_str+ str(i)
        
    #PointCloud.astype("float32").tofile(str(i)+'.bin')
    PointCloud.astype("float32").tofile(filename_str+'.bin')
    i = i+1
    print(PointCloud)

def camera_imad(ros_data):
    global j
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, 1)
    #cv2.imshow("imad",image_np)
    #cv2.waitKey(1)
    
    len_of_j=len(str(j))
    filename_str=''
    for _ in range(6-len_of_j):
        filename_str=filename_str + '0'
        
    filename_str=filename_str+ str(j)+".png"
    cv2.imwrite(filename_str,image_np)
    j = j+1
     
def listener():

    rospy.init_node('listener', anonymous=True)
    print("okay")
    lidar = rospy.Subscriber("/points_raw", PointCloud2, get_point)
    camera = rospy.Subscriber("/simulator/camera_node/image/compressed", CompressedImage, camera_imad)
    rospy.spin()

if __name__ == '__main__':
    listener()





