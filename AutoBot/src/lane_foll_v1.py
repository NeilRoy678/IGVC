#!/usr/bin/env python3
import rospy
from geometry_msgs.msg  import Twist
from std_msgs.msg import String, Float32MultiArray
from turtlesim.msg import Pose
from math import pow,atan2,sqrt

class Autobot():

    def __init__(self):
        #Creating node,publisher and subscriber
        rospy.init_node('lane_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # subscribing to lane_cent_left
        self.left_cent_sub = rospy.Subscriber('/lane_cent_left', Float32MultiArray, self.callbackL)
        self.left_cent = Float32MultiArray()

        # subscribing to lane_cent_right 
        self.right_cent_sub = rospy.Subscriber('/lane_cent_right', Float32MultiArray, self.callbackR)
        self.right_cent = Float32MultiArray()

        self.curr_cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd_vel)
        self.curr_cmd_vel = Twist()

        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callbackL(self, data):
        self.left_cent = data
    
    def callbackR(self, data):
        self.right_cent = data

    def callback_cmd_vel(self, data):
        self.curr_cmd_vel = data

    def get_error(self, left, right):
        print(left)
        print(right)
        left_px = left[1]
        right_px = right[1]
        err = left_px - right_px
        print(err)
        return err

    def follow_lane(self):
        #goal_pose = Pose()
        #goal_pose.x = input("Set your x goal:")
        #goal_pose.y = input("Set your y goal:")
        #distance_tolerance = input("Set your tolerance:")
        vel_msg = Twist()
        px_tolerance = 10
        lin_vel = 0.3
        #self.get_error(self.left_cent,self.right_cent)

        if  self.get_error(self.left_cent.data,self.right_cent.data) >= px_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = lin_vel
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            kp = 4
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.curr_cmd_vel.angular.z + kp * self.get_error(self.left_cent,self.right_cent)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        else:
            #Stopping our robot after the movement is over
            vel_msg.linear.x = lin_vel
            vel_msg.angular.z = self.curr_cmd_vel.angular.z
            self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = Autobot()
        x.follow_lane()

    except rospy.ROSInterruptException: pass