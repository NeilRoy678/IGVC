#!/usr/bin/env python3
from cmath import cos, inf
from dis import dis

import sys
from tarfile import ExFileObject
import cv2
from matplotlib.pyplot import flag, sca

import numpy as np


import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist

bridge = CvBridge()
move = Twist()
flag = False
side = None

def read_rgb_image(image_name):
    rgb_image = image_name

    width = np.size(rgb_image,0)
    height = np.size(rgb_image,1)


    return rgb_image
def perp(a) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

def seg_intersect(a1,a2, b1,b2) :

        da = a2-a1
        db = b2-b1
        dp = a1-b1
        dap = perp(da)
        denom = np.dot( dap, db)
        num = np.dot( dap, dp )
        return (num / denom.astype(float))*db + b1


def get_coords(x, y, angle, imwidth, imheight):    

    length = 1000


    x =  int(np.round(x + length * np.cos(angle * np.pi / 180.0)))
    y =  int(np.round(y + length * np.sin(angle * np.pi / 180.0)))
    return x,y 

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",rgb_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(rgb_image, lower_bound_color, upper_bound_color)

    return mask

def detect_lane_in_a_frame(image_frame):
    global flag
    yellowLower =(0, 0, 100)
    yellowUpper = (35, 255, 255)
    #kernel = np.ones((50,50),np.float32)/25

    #rgb_image = read_rgb_image(image_frame)
    binary_image_mask = filter_color(image_frame, yellowLower, yellowUpper)

    #contours = getContours(binary_image_mask)
    binary_image_mask = binary_image_mask[220:600,:]
    binary_image_mask = cv2.resize(binary_image_mask,(700,700))
    #inary_image_mask = cv2.filter2D(binary_image_mask,-1,kernel)
    binary_image_mask = cv2.rotate(binary_image_mask, cv2.ROTATE_90_CLOCKWISE)
    
    black_image = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
    blank_grid = np.zeros([image_frame.shape[0], image_frame.shape[1],3],'uint8')
    contours, hierarchy = cv2.findContours(binary_image_mask.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)

                                        
    #contours_arr = []
    for c in contours:
        area = cv2.contourArea(c)
            
        if (area>1000):
            M = cv2.moments(c)
            area = cv2.contourArea(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                left = tuple(c[c[:, :, 0].argmin()][0])
                right = tuple(c[c[:, :, 0].argmax()][0])
                top = tuple(c[c[:, :, 1].argmin()][0])
                bottom = tuple(c[c[:, :, 1].argmax()][0])

                #mid_point = ((top[0] + right[0])//2, (right[1] + left[1])//2)
                cv2.drawContours(image_frame, [c], -1, (0, 255, 0), 2)
                cv2.circle(binary_image_mask, (cx,cy), 10, (0, 0, 0), -1)

                #cv2.line(image_frame,(cx,int(endx)),(cy,int(endy)),(0,0,0),5,-1)

                #print(offset)
                distance = cx*0.004
                if flag == True:
                    if side == "right":
                        distance_ = (2.596-(np.cos(1.6) * 2.64))/2
                        value = ((np.cos(96) * 2.64) + 1.588) + distance_
                    
                        offset = value - distance - 0.10
                        ed = 0.2
                        p = 0.22
                        d = 0.13
                        #steering_angle = p*offset + d*ed 
                        
                        #velocity_pub.publish(move)
                        move.linear.x = 0.2
                    
                        steering_angle = p*offset + d*ed 
                        print(steering_angle)

                        print("TURN OBSTACLE")
                    
                    if side == "left":
                        distance_ = (2.596-(np.cos(1.6) * 2.64))/2
                        value = ((np.cos(96) * 2.64) + 1.588) + distance_
                    
                        offset = 1.4 - distance
                        ed = 0.2
                        p = 0.32
                        d = 0.13
                        #steering_angle = p*offset + d*ed 
                        
                        #velocity_pub.publish(move)
                        move.linear.x = 0.2
                    
                        steering_angle = p*offset + d*ed 
                        print(steering_angle)

                        print("TURN OBSTACLE")
                    
                    #print(distance,value,offset,steering_angle)
                else:
                   # print("No obstacle")
                    
                    offset = 2 - distance
                    ed = 0.2
                    p = 0.32
                    d = 0.10
                    move.linear.x = 0.2
                    steering_angle = p*offset + d*ed 

                #move.angular.z = steering_angle
                
                #print(distance)
                #print(distance,steering_angle)
            
                cv2.drawContours(black_image, [c], -1, (255,255,255), -1)


    return black_image,blank_grid,binary_image_mask,image_frame,top,bottom,distance,steering_angle

def scan_callback(scan):
    global flag
    global side
    try:
        idx = [i/2 for i, arr in enumerate(scan.ranges) if np.isfinite(arr).all()]
        sum = idx[-4] + idx[4]
        print(sum)
        if sum < 150:
            side = "right"
            print("turn left")
        elif sum<207 and sum>165:
            side = "center"
            print("turn left or right")
        else:
            side = "left"
            print("turn right")
        #print(sum)
        print(scan.ranges[int(idx[6]*2)])
        if scan.ranges[int(idx[6]*2)] <4 :
            flag = True
        else:
            flag = False
    except Exception as e:
        print(e)
        flag = False

def image_callback(ros_image):


    inf_ = np.float(inf)
    # define publisher
    #pub_lane_cent_left = rospy.Publisher('lane_cent_left', Float32MultiArray, queue_size=10)
    #print('got an image')
    global bridge
    global move

    pc_pub = rospy.Publisher("/revised_scan",LaserScan,queue_size=50)
    velocity_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=50)

    current_time = rospy.Time.now()


    scann = LaserScan()
    scann.header.stamp = current_time
    scann.header.frame_id = 'lidar'
    scann.angle_min = -1.535889983177185

    scann.angle_max =0.785398 #1.535889983177185

    scann.angle_increment = 0.008556489832699299

    scann.time_increment = 0.0

    scann.range_min = 0.05000000074505806
    scann.range_max = 30.0

    reference_point = (0,350)
    #print(cx)
     
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    image = read_rgb_image(cv_image)
    image = cv2.resize(image,(700,700))
    
    #image wrap
    pts = [(600,400),(675,550),(50,550),(200, 400)]



    #for (x, y) in pts:
    cv2.circle(image, reference_point, 5, (0, 255, 0), -1)

    #warped = cv2.resize(warped, (150, 175))
    filt_warped,_,binary_image_mask,image,top,bottom,value_distance,sterring_angel = detect_lane_in_a_frame(image)

    move.angular.z = sterring_angel
    #move.linear.x = 0.4
    
    velocity_pub.publish(move)

    cv2.imshow("Warped Image window",image)
    cv2.imshow("Masked Warped Image window", binary_image_mask)
    #cv2.imshow("Image window", image)
    cv2.waitKey(1)

def main(args):
  #print("test1")

  rospy.init_node('GridMapping', anonymous=True)

  image_topic="/usb_cam_right/image_raw_right"

  image_sub1 = rospy.Subscriber(image_topic, Image, image_callback)
  laser_to_laser_sub = rospy.Subscriber("/scan", LaserScan, scan_callback)

  
  try:
    rospy.Rate(10)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == "__main__":
    
    main(sys.argv[1:])