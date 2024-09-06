#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

# BasicMover
class BasicMoverJames:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # Current heading of the robot.
        self.cur_yaw = None

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        raise NotImplementedError

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        raise NotImplementedError
        
    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        raise NotImplementedError

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
        """Moves the robot in a circle with radius `r`"""
        raise NotImplementedError
        
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

        twist.linear.x = 0.3

        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    BasicMoverJames().run()
    # BasicMoverJames().out_and_back(1)
    # BasicMoverJames().draw_square(1)
    # BasicMoverJames().move_in_a_circle(1)
    # BasicMoverJames().rotate_in_place()
