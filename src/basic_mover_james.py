#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# BasicMover
class BasicMoverJames:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        # self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # Current heading of the robot.
        # self.cur_yaw = None
        self.cur_point = Point()

    # def my_odom_cb(self, msg):
    #     print("-----")
    #     print(msg)
    #     print("-----")

    #     self.cur_point.x = msg.x
    #     self.cur_point.y = msg.y
    #     self.cur_point.z = msg.z
        
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""

        # print("----------")
        # print(msg)
        # print("----------")

        self.cur_point.x = msg.pose.pose.position.x
        self.cur_point.y = msg.pose.pose.position.y
        self.cur_point.z = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.cur_point.z = [self.cur_point.z[0] * 180 / math.pi, self.cur_point.z[1] * 180 / math.pi, self.cur_point.z[2] * 180 / math.pi]

        print("==========")
        print(self.cur_point)
        print("==========")
        # self.my_odom_pub.publish(data)

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        raise NotImplementedError
        
    def move_forward(self, target_dist):
        
        prev_x = self.cur_point.x
        prev_y = self.cur_point.y

        rate = rospy.Rate(100)

        twist = Twist()

        while pow(target_dist,2) > pow(abs(prev_x - self.cur_point.x),2) + pow(abs(prev_y - self.cur_point.y),2):

            if pow(abs(prev_x - self.cur_point.x),2) + pow(abs(prev_y - self.cur_point.y),2) + 0.5 > pow(target_dist,2):

                twist.linear.x = 0.1
            
            else :

                twist.linear.x = 0.5

            self.cmd_vel_pub.publish(twist)

            rate.sleep()

        twist.linear.x = 0

        self.cmd_vel_pub.publish(twist)

    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        raise NotImplementedError

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """ 
        raise NotImplementedError

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

    def run(self):

        rate = rospy.Rate(20)

        twist = Twist()

        twist.linear.x = 0.05

        while not rospy.is_shutdown():

            self.cmd_vel_pub.publish(twist)

            rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    BasicMoverJames().move_forward(1)
    # BasicMoverJames().run()
    # BasicMoverJames().out_and_back(1)
    # BasicMoverJames().draw_square(1)
    # BasicMoverJames().move_in_a_circle(1)
    # BasicMoverJames().rotate_in_place()
