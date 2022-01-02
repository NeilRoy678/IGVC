
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import matplotlib.pyplot as plt
import message_filters
import numpy as np 
import sensor_msgs.msg
from sensor_msgs.msg import LaserScan

class LaneFollower:

    def __init__(self) -> None:
        self.bridge_object = CvBridge()
        image_sub = message_filters.Subscriber("/usb_cam_center/image_raw_center",Image)
        depth_sub = message_filters.Subscriber("/kinect_cam/depth/image_raw",Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5,allow_headerless=True)
        
        self.ts.registerCallback(self.camera_callback)
         
       
        self.pub = rospy.Publisher("/revised_scan", LaserScan, queue_size = 10)
 

        self.scann = LaserScan()
    def draw_lane(self,img, lane_lines_img, y_vals, left_x_vals, right_x_vals, M,left_fit,right_fit,depth_image):
    
        # Prepare the x/y points for cv2.fillPoly()
        left_points = np.array([np.vstack([left_x_vals, y_vals]).T])
        right_points = np.array([np.flipud(np.vstack([right_x_vals, y_vals]).T)])


        # right_points = np.array([np.vstack([right_x_vals, y_vals]).T])
        points = np.hstack((left_points, right_points))
        points_left= np.int_([left_points])
        points_right= np.int_([right_points])
        
        # Color the area between the lines (the lane)
        # for i in range(len(points_left[0][0])):
        #     tup = tuple(np.int_(left_points[0][i]))
        #     cv2.circle(lane_lines_img,tup,3,(0,255,0),3)
        
        # for i in range(len(points_right[0][0])):
        #     tup = tuple(np.int_(right_points[0][i]))
        #     cv2.circle(lane_lines_img,tup,3,(0,255,0),3)



        lane = np.zeros_like(lane_lines_img)  # Create a blank canvas to draw the lane on
        #cv2.fillPoly(lane, np.int_([points]), (0, 255, 0))
        unwarped_lane_info = cv2.addWeighted(lane_lines_img, 1, lane, .3, 0)
        # coord = np.argwhere(unwarped_lane_info >=255)
        # print(coord)
        
        Minv = np.linalg.pinv(M)

        warped_lane_info = cv2.warpPerspective(unwarped_lane_info, Minv, (img.shape[1], img.shape[0]))

        real_img = cv2.addWeighted(img, 1, warped_lane_info, 1, 0)
        unwarped_lane_info = cv2.cvtColor(unwarped_lane_info,cv2.COLOR_BGR2GRAY)
        unwarped_lane_info = np.array(unwarped_lane_info,dtype = np.dtype('f8'))


        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        img = np.array(img,dtype = np.dtype('f8'))

        drawn_img = cv2.addWeighted(depth_image, 1, unwarped_lane_info, 1, 0)
        #cv2.circle(drawn_img,(coord[699][0],coord[699][1]),50,(255,255,0),-1)
        return drawn_img,unwarped_lane_info,real_img

    def window_search(self,binary_warped):

        histogram = np.sum(binary_warped[int(binary_warped.shape[0]/2):,:], axis=0)
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 15
        # Set height of windows
        window_height = np.int(binary_warped.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            #cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
            #scv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 


            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 




        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        # Generate black image and colour lane lines
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [1, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 1]
            
        # Draw polyline on image
        right = np.asarray(tuple(zip(right_fitx, ploty)), np.int32)
        left = np.asarray(tuple(zip(left_fitx, ploty)), np.int32)
        left_ = (right[0]+left[0])/2
        right_ = (right[1]+left[1])/2

    
        #Center Circle  
        #cv2.circle(out_img,(int(left_[0]),int(right_[0])),10,(0,255,0),-1)

        #Draws the lane 
        cv2.polylines(out_img, [right], False, (0,0,255), thickness=5)
        cv2.polylines(out_img, [left], False, (255,0,0), thickness=5)

        return left_lane_inds, right_lane_inds, out_img,ploty,left_fitx,right_fitx,left_fit,right_fit,right,left

    def warp_perspective(self,lane,image):
        height = image.shape[0]
        width = image.shape[1]
        pts1 = np.float32([lane[3],lane[0],lane[2],lane[1]])
        pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(image,M,(width,height))
        return dst,M

    def camera_callback(self,data,depth_data):
        try:
            current_time = rospy.Time.now()
            self.scann.header.stamp = current_time
            self.scann.header.frame_id = 'lidar'
            self.scann.angle_min = -3.1415
            self.scann.angle_max = 3.1415
            self.scann.angle_increment = 0.01
            self.scann.time_increment = 4.99999987369e-05
            self.scann.range_min = 0
            self.scann.range_max = 10

            
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding='bgr8')   
            depth_image = self.bridge_object.imgmsg_to_cv2(depth_data, "32FC1")
            depth_image = np.array(depth_image, dtype = np.dtype('f8'))
            #cv_image_array = np.array(depth_image, dtype = np.dtype('f8'))
            image = cv2.resize(cv_image,(700,700))
            depth_image = cv2.resize(depth_image,(700,700))
            lane = [(600,400),(675,550),(50,550),(200, 400)]
            for i in lane:
                image = cv2.circle(image,i,4,(0,255,0),-1)
            warp_image,M = self.warp_perspective(lane,image)
            #warp_image = cv2.resize(warp_image ,(700,700))
            warp_image = cv2.cvtColor(warp_image,cv2.COLOR_BGR2GRAY)
            warp_threshold = cv2.adaptiveThreshold(warp_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,19,5)
            left_,right_,out,ploty,left_fitx,right_fitx,left_fit,right_fit,right,left = self.window_search(warp_threshold)
            
            
            
            cv_image_norm = cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)

            cv_image_resized = cv2.resize(cv_image_norm, (warp_image.shape[0],warp_image.shape[1]), interpolation = cv2.INTER_CUBIC)

            final_image,unwarped,real_img = self.draw_lane(image,out,ploty,left_fitx,right_fitx,M,left_fit,right_fit,cv_image_resized)

            right_distance = []
            left_distance = []

            print("left " + str(depth_image[left[500][0]][left[500][1]]))
            print("right " + str(depth_image[right[500][0]][right[500][1]]))
            
            cv2.circle(final_image,tuple(left[500]),10,(255,255,0),-1)
            cv2.circle(final_image,tuple(right[500]),10,(255,255,0),-1)
            
            #right_distance.append(depth_image[right[500][0]][right[500][1]])
            intensites = np.ones((int((self.scann.angle_max-self.scann.angle_min)/self.scann.angle_increment),1))
            self.scann.ranges = list(np.ones((int((self.scann.angle_max-self.scann.angle_min)/self.scann.angle_increment))))
            self.scann.intensities = []

            print(len(self.scann.ranges))

            self.pub.publish(self.scann)
            # for i in range(len(left)-2):
            #     left_distance.append(depth_image[lehow to create a node in ros 
            
            cv2.imshow("frame",final_image)
            cv2.imshow("frame1",unwarped)
            cv2.imshow("frame2",real_img)
            
            
            #cv2.imshow("depth",cv_image_norm)
       
            cv2.waitKey(1)

        except Exception as e:
            print(e)



def main():

    
    lane_follower_object = LaneFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("SHUTTING DOWN ")

if __name__ =="__main__":

    rospy.init_node('lane_following_node',anonymous=True)


    main()