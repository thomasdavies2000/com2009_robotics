#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi
from com2009_msgs.srv import Approach, ApproachResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class Approach:
    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "approach_block"
        self.vel_cmd = Twist()
        self.lidar_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.callback_lidar)
        self.startup = True
        self.turn = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def print_stuff(self, a_message):
        print(a_message)
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")


    def callback_lidar(self, lidar_data):
        """Obtain a subset of the LaserScan.ranges array corresponding to a +/-10 degree arc in front of it.
        Convert this subset to a numpy array to allow for more advanced processing."""
        print(min(lidar_data.ranges))
        left_arc = lidar_data.ranges[0:10]
        right_arc = lidar_data.ranges[-10:]
        front_arc = np.array(left_arc + right_arc)
        # find the miniumum object distance within the frontal laserscan arc:
        self.object_distance = front_arc.min()
        
    def main_loop(self):
        
        status = ""
        wait = 0
        self.vel_cmd.linear.x = 0.26
        while not self.ctrl_c:
            self.callback_lidar
            
            
            

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

            if self.object_distance < 0.5:
                print("not yet")
                self.vel_cmd.linear.x = 0
            self.pub.publish(self.vel)
            self.print_stuff(status)
            self.rate.sleep()

if __name__ == '__main__':
    approach_instance = Approach()
    try:
        approach_instance.main_loop()
    except rospy.ROSInterruptException:
        pass