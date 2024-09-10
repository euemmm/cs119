#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from time import sleep

# BasicMover
class BasicMoverJames:

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        # self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # Current heading of the robot.
        # self.cur_yaw = None
        self.cur_point = Point()
        self.cur_point.x = 0
        self.cur_point.y = 0
        self.cur_point.z = 0
        
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""

        # print("----------")
        # print(msg)
        # print("----------")

        self.cur_point.x = msg.pose.pose.position.x
        self.cur_point.y = msg.pose.pose.position.y
        self.cur_point.z = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        yaw = self.cur_point.z[2] * 180 / math.pi

        if yaw < 0 :
            yaw = 360 + yaw
        
        self.cur_point.z = yaw

        # self.cur_point.z = [self.cur_point.z[0] * 180 / math.pi, self.cur_point.z[1] * 180 / math.pi, yaw]

        print("==========")
        print(self.cur_point)
        print("==========")
        # self.my_odom_pub.publish(data)

    def turn_to_heading_deg(self, target_yaw):

        self.turn_to_heading(target_yaw * math.pi / 180)

    def turn_to_heading(self, target_yaw): #target_yaw is rad
        
        sleep(0.1) #wait for odom input

        twist = Twist()

        twist.angular.z = 0.01
        self.cmd_vel_pub.publish(twist)

        #rad -> deg == *180/pi
        #deg -> rad == *pi/180

        rate = rospy.Rate(100)
        
        target_yaw = target_yaw * 180 / math.pi

        while target_yaw < 0 :

            target_yaw = 360 + target_yaw

        while target_yaw > 360 :

            target_yaw = target_yaw - 360

        if target_yaw == 0 or target_yaw == 360:
            
            twist.angular.z = 0.2
            self.cmd_vel_pub.publish(twist)

            while 360 - self.cur_point.z > 10:
                twist.angular.z = 0.1
                self.cmd_vel_pub.publish(twist)

            while 360 - self.cur_point.z > 0.5:
                twist.angular.z = 0.05
                self.cmd_vel_pub.publish(twist)

            twist.angular.z = 0.05
            self.cmd_vel_pub.publish(twist)
            return

        if target_yaw > self.cur_point.z :

            while target_yaw - self.cur_point.z > 0:

                if target_yaw - self.cur_point.z < 10 : #Decelerate when target angle is less than 10 left

                    twist.angular.z = 0.05

                else :

                    twist.angular.z = 0.2

                self.cmd_vel_pub.publish(twist)

        elif target_yaw < self.cur_point.z :

            self.turn_to_heading_deg(0)

            self.turn_to_heading_deg(target_yaw)

        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
     
    def move_forward(self, target_dist):
        
        sleep(0.1)

        prev_x = self.cur_point.x
        prev_y = self.cur_point.y

        rate = rospy.Rate(100)

        twist = Twist()

        while pow(target_dist,2) > pow(abs(prev_x - self.cur_point.x),2) + pow(abs(prev_y - self.cur_point.y),2):

            if pow(abs(prev_x - self.cur_point.x),2) + pow(abs(prev_y - self.cur_point.y),2) + 1 > pow(target_dist,2):

                twist.linear.x = 0.1
            
            else :

                twist.linear.x = 0.5

            self.cmd_vel_pub.publish(twist)

            rate.sleep()

        twist.linear.x = 0

        self.cmd_vel_pub.publish(twist)

    def out_and_back(self, target_dist):
        self.move_forward(target_dist)
        if self.cur_point.z + 180 > 360:
            self.turn_to_heading_deg(self.cur_point.z + 180 - 360)
        else :
            self.turn_to_heading_deg(self.cur_point.z + 180)
        self.move_forward(target_dist)

    def draw_square(self, side_length):
        for _ in range(0,4) :
            self.move_forward(side_length)
            if self.cur_point.z + 90 > 360 :
                self.turn_to_heading_deg(self.cur_point.z + 90 - 360)
            else :
                self.turn_to_heading_deg(self.cur_point.z + 90)

    def move_in_a_circle(self, r):

        twist = Twist()

        twist.linear.x = 0.5

        # velocity = 0.5

        # radius = (velocity * 360/yaw) / (2 * math.pi)

        # yaw = 360 / ((radius * (2 * math.pi)) / 0.5)

        twist.angular.z = (360 / ((r * (2 * math.pi)) / 0.5)) * math.pi / 180
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            
            self.cmd_vel_pub.publish(twist)
            
            rate.sleep()
        
    def rotate_in_place(self):
        """For debugging."""
        twist = Twist()
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    # BasicMoverJames().move_forward(2)
    # BasicMoverJames().turn_to_heading_deg(180)
    # BasicMoverJames().out_and_back(1)
    BasicMoverJames().draw_square(1)
    # BasicMoverJames().move_in_a_circle(1)
    # BasicMoverJames().rotate_in_place()
