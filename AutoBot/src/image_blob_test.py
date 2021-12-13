#!/usr/bin/env python3

from numpy.lib.shape_base import dsplit
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
global cx,cy

bridge = CvBridge()

def read_rgb_image(image_name, show, side):
    #rgb_image = cv2.imread(image_name)
    rgb_image = image_name
    width = np.size(rgb_image,0)
    height = np.size(rgb_image,1)
    w_perc = 0.1
    offsetR = 50
    offsetL = 50
    if side=="R":
        crp_img = rgb_image[150:, 0+offsetR:int(width*w_perc)+offsetR]
    elif side=="L":
        crp_img = rgb_image[150:, width - int(width*(w_perc))-offsetL:width-offsetL]
        #crp_img = rgb_image[100:, 500:640]
    #print([width,height]) #[512, 640]
    if show: 
        cv2.imshow("RGB Image",rgb_image)
        cv2.imshow("Cropped RGB Image",crp_img)
    return crp_img

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",rgb_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(rgb_image, lower_bound_color, upper_bound_color)

    return mask

def getContours(binary_image):     
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours

def get_lane_contour(binary_image, rgb_image, contours):
    global cx,cy
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    contours_arr = []
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>150):
            #cx, cy = get_contour_center(c)
            contours_arr.append(get_contour_center(c))
            #cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            #cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            #cx, cy = get_contour_center(c)
            #cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            #cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            #cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            #print ("Area: {}, Perimeter: {}".format(area, perimeter))
    #print ("number of contours: {}".format(len(contours)))
    #cv2.imshow("RGB Image Contours",rgb_image)
    #cv2.imshow("Black Image Contours",black_image)
    return contours_arr

def get_contour_center(contour):
    
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_lane_in_a_frame(image_frame,side):
    yellowLower =(0, 0, 100)
    yellowUpper = (35, 255, 255)
    rgb_image = read_rgb_image(image_frame, False,side)
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    return get_lane_contour(binary_image_mask, rgb_image,contours)

def filter_contours(contour):
    #n = np.shape(contour) #[num cont, [x,y]]
    #print(n)
    try:
        lowest_cont = (0,0)
        phldr = contour[0][1]
        for c in contour:
            if phldr <= c[1]:
                phldr = c[1]
                lowest_cont = c
    except IndexError:
        print("no lanes found")
    #print(lowest_cont)
    return lowest_cont

        

def image_callback_R(ros_image):
  # define publisher
  pub_lane_cent_right = rospy.Publisher('lane_cent_right', Float32MultiArray, queue_size=10)
  #print('got an image')
  global bridge
  #print(cx)
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)

  contours_arr = detect_lane_in_a_frame(cv_image,side="R")
  #print(contours_arr)
  cent = filter_contours(contours_arr)
  cent_data_pub = Float32MultiArray()
  cent_data_pub.data = cent
  pub_lane_cent_right.publish(cent_data_pub)
  
  #cv2.imshow("Image window", cv_image)
  #cv2.waitKey(3)

def image_callback_L(ros_image):
  # define publisher
  pub_lane_cent_left = rospy.Publisher('lane_cent_left', Float32MultiArray, queue_size=10)
  #print('got an image')
  global bridge
  #print(cx)
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)

  contours_arr = detect_lane_in_a_frame(cv_image,side="L")
  #print(contours_arr)
  cent = filter_contours(contours_arr)
  cent_data_pub = Float32MultiArray()
  cent_data_pub.data = cent
  pub_lane_cent_left.publish(cent_data_pub)
  
  #cv2.imshow("Image window", cv_image)
  cv2.waitKey(3)

  
def main(args):
  #print("test1")
  rospy.init_node('image_converter', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  image_topic1="/usb_cam_right/image_raw_right"
  image_topic2="/usb_cam_left/image_raw_left"
  #print("test2")
  image_sub1 = rospy.Subscriber(image_topic1, Image, image_callback_R)
  image_sub2 = rospy.Subscriber(image_topic2, Image, image_callback_L)

  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)