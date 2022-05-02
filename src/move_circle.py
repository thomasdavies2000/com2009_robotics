#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# additionally, import the "euler_from_quaternion" function from the tf library
# for converting the raw orientation values from the odometry message:
from tf.transformations import euler_from_quaternion

class Circle:
    def callback(self, topic_data):
        orientation_x = topic_data.pose.pose.orientation.x
        orientation_y = topic_data.pose.pose.orientation.y
        orientation_z = topic_data.pose.pose.orientation.z
        orientation_w = topic_data.pose.pose.orientation.w

        position_x = topic_data.pose.pose.position.x
        position_y = topic_data.pose.pose.position.y
        position_z = topic_data.pose.pose.position.z

        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')
        self.x = position_x
        self.y = position_y
        self.theta_z = yaw 
        # robot_odom = [position_x, position_y, position_z, roll, pitch, yaw]

        if self.startup:
            self.startup = False   
            # self.df['initial'] = robot_odom
            # self.got_start_data = True
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z



    def __init__(self):
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback)
        self.startup = True
        rospy.init_node('move_circle', anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.vel_cmd = Twist()
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
        rospy.loginfo("the 'move_circle' node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s
        

        print("stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)

        self.ctrl_c = True
    
    def main_loop(self):
        count = 0
        loop_count = 0
        while not self.ctrl_c:
            count+=1
            yaw_diff = abs(self.theta_z0 - self.theta_z)
            print("yaw diff " + str(yaw_diff))
            print(count)
            print(self.x)
            if yaw_diff <0.1 and yaw_diff > -0.1 and count >50:
                count = 0
                loop_count +=1
                print("yoyo")
            
            # specify the radius of the circle:
            path_rad = 0.5 # m
            # linear velocity must be below 0.26m/s:
            lin_vel = 0.1 # m/s

            self.vel_cmd.linear.x = lin_vel
            if loop_count % 2 == 0:
                self.vel_cmd.angular.z = lin_vel / path_rad # rad/s
            else:
                self.vel_cmd.angular.z = -lin_vel / path_rad # rad/s
                

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass