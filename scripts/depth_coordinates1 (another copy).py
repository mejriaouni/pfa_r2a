#!/usr/bin/env python3
import rospy
import sys
import message_filters
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from qt_rviz.msg import Shapes,Shape
import math
import numpy as np

def callback(rgb_data, depth_data, camera_info):
    try:
        global pub
        camera_info_K = np.array(camera_info.K)
        m_fx = camera_info.K[0]
        m_fy = camera_info.K[4]
        m_cx = camera_info.K[2]
        m_cy = camera_info.K[5]
        inv_fx = 1. / m_fx
        inv_fy = 1. / m_fy
        bridge = CvBridge()
        cv_rgb = bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")
        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        depth_8 = (depth_array * 255).round().astype(np.uint8)
        cv_depth = np.zeros_like(cv_rgb)
        cv_depth[:,:,0] = depth_8
        cv_depth[:,:,1] = depth_8
        cv_depth[:,:,2] = depth_8
        
        
        img_gray = cv2.cvtColor(cv_rgb,cv2.COLOR_BGR2GRAY)
        noise_removal = cv2.bilateralFilter(img_gray,9,75,75)
        ret,thresh_image = cv2.threshold(noise_removal,200,255,cv2.THRESH_BINARY)
        canny_image = cv2.Canny(thresh_image,250,255)
        kernel = np.ones((3,3), np.uint8)
        dilated_image = cv2.dilate(canny_image,kernel,iterations=1)
        contours, h = cv2.findContours(dilated_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        contours= sorted(contours, key = cv2.contourArea, reverse = True)
        pt = (300, 2 * cv_rgb.shape[0] // 5)
        shapes_to_publish=Shapes()
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            cv2.drawContours(cv_rgb,[cnt],-1,(255,255,0),3)
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            (x,y)=(cX,cY)
            cv2.circle(cv_rgb, (cX, cY), 7, (0, 255, 255), -1)
            roi_depth = depth_image[y-20:y+20, x-20:x+20]
            value = roi_depth.item(20, 20)
            point_x = value 
            point_y = -1*(x - m_cx) * point_x * inv_fx
            point_z = 0.64-1*(y  - m_cy) * point_x * inv_fy
            print(point_x)
        #print((x,y))
            x_str = "X: " + str(format(point_x, '.2f'))
            y_str = "Y: " + str(format(point_y, '.2f'))
            z_str = "Z: " + str(format(point_z, '.2f'))
            cv2.putText(cv_rgb, x_str, (x+30, y), cv2.FONT_HERSHEY_SIMPLEX,  
                       0.7, (0,0,255), 1, cv2.LINE_AA)
            cv2.putText(cv_rgb, y_str, (x+30, y+20), cv2.FONT_HERSHEY_SIMPLEX,  
                       0.7, (0,0,255), 1, cv2.LINE_AA)
            cv2.putText(cv_rgb, z_str, (x+30, y+40), cv2.FONT_HERSHEY_SIMPLEX,  
                       0.7, (0,0,255), 1, cv2.LINE_AA)
                   
            dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
            dist_str = "dist:" + str(format(dist, '.2f')) + "m"
            n=0
            if len(approx) ==6 :
                cv2.putText(cv_rgb,'Cube', pt ,cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 2, [0,255, 255], 2)
                n=1
            elif len(approx) == 7:
                cv2.putText(cv_rgb,'Cube', pt ,cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 2, [0, 100, 255], 2)
                n=1
            elif len(approx) == 8:
                cv2.putText(cv_rgb,'Cylinder', pt ,cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 2, [0, 0, 255], 2)
                n=3
            elif len(approx) > 10:
                cv2.putText(cv_rgb,'Sphere', pt ,cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 2, [255, 0, 0], 2)
                n=2
            sh=Shape()
            if (n>0):
                sh.shape_type=n
            sh.pose.position.x=point_x
            sh.pose.position.y=point_y
            sh.pose.position.z=point_z
            sh.scales.x=0.3
            sh.scales.y=0.3
            sh.scales.z=0.3
            sh.color.r=1
            sh.color.g=0
            sh.color.b=0
            sh.color.a=1
            if (n>0):
                shapes_to_publish.shapes.append(sh)
            cv2.imshow("Depth Image",roi_depth)
        pub.publish(shapes_to_publish)    
        cv2.imshow("rgb Image",cv_rgb)
        cv2.waitKey(1)
    except Exception as err:
        print (err)


def start_node():
    rospy.init_node('depth_coordinates')
    rospy.loginfo('depth_coordinates node started')
    global pub
    pub = rospy.Publisher('shapouet', Shapes, queue_size=10)
    camera_info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    image_sub = message_filters.Subscriber("/camera/color/image_raw",Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_raw",Image)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub, camera_info_sub], queue_size=10, slop=0.5)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
