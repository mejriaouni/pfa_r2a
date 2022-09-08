#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# known pump geometry
#  - units are pixels (of half-size image)

def process_image(msg):
    try:
        # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        #cv2.imshow("Gray Converted Image",img_gray)
        # Noise removal with iterative bilateral filter(removes noise while preserving edges)
        noise_removal = cv2.bilateralFilter(img_gray,9,75,75)
        #cv2.imshow("Noise Removed Image",noise_removal)
        ret,thresh_image = cv2.threshold(noise_removal,200,255,cv2.THRESH_BINARY)
        #cv2.imshow("Image after Thresholding",thresh_image)
        canny_image = cv2.Canny(thresh_image,250,255)
        #cv2.imshow("Image after applying Canny",canny_image)
        kernel = np.ones((3,3), np.uint8)
        dilated_image = cv2.dilate(canny_image,kernel,iterations=1)
        #cv2.imshow("Dilation", dilated_image)
        contours, h = cv2.findContours(dilated_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours= sorted(contours, key = cv2.contourArea, reverse = True)
        pt = (300, 2 * img.shape[0] // 5)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            cv2.drawContours(img,[cnt],-1,(255,255,0),3)
            cv2.putText(img,str(len(approx)), pt ,cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 2, [0, 255, 255], 2)
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            #print(int(x+w))
            color=(250,84,167)
            cv2.rectangle(img , (int(x-w/2),int(y-h/2)), (int(x+w/2),int(y+h/2)), color, 1)
            cv2.circle(img, (int(x), int(y)), 7, (0, 255, 255), -1)
        #cv2.imshow("Original Image",img)
        cv2.waitKey(1)
    except Exception as err:
        print (err)
def start_node():
    rospy.init_node('edge_detection')
    rospy.loginfo('edge_detection node started')
    rospy.Subscriber("camera/color/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
