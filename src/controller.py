#!/usr/bin/python3

import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 
         
        
    # checks whether there is an obstacle in front of the robot
    # or not
    def laser_callback(self, msg: LaserScan):
        if msg.ranges[0] <= self.stop_distance:
            self.state = self.ROTATE
    
    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw
    
    def run(self):
        
        while not rospy.is_shutdown():
            
            # check whether state is changed or not
            if self.state == self.GO:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
                continue
            
            self.cmd_publisher.publish(Twist())
            
            rospy.sleep(1)
            
            remaining = self.goal_angle
            prev_angle = self.get_heading()
            
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_publisher.publish(twist)
            
            # rotation loop
            while remaining >= self.epsilon:
                current_angle = self.get_heading()
                delta = abs(prev_angle - current_angle)
                remaining -= delta
                prev_angle = current_angle
            
            self.cmd_publisher.publish(Twist())

            rospy.sleep(1)
            
            self.state = self.GO


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()