#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LaserScanProcessor:
    def __init__(self):
        rospy.init_node('laser_scan_processor')

        # Publisher to /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to /kobuki/laser/scan
        self.laser_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.laser_callback)

    def laser_callback(self, laser_data):
        # Check laser readings to determine robot behavior
        front_distance = laser_data.ranges[len(laser_data.ranges)//2]  # Reading in front of the robot
        right_distance = min(laser_data.ranges[:len(laser_data.ranges)//4])  # Reading at the right side of the robot
        left_distance = min(laser_data.ranges[3*len(laser_data.ranges)//4:])  # Reading at the left side of the robot

        cmd_vel = Twist()

        if front_distance > 1.0:
            # No obstacle in front, move forward
            cmd_vel.linear.x = 0.2  # Adjust the desired forward velocity as needed
            cmd_vel.angular.z = 0.0
        else:
            # Obstacle in front, turn left
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Adjust the desired angular velocity for left turn

        # Check right side
        if right_distance < 1.0:
            # Obstacle at the right, turn left
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Adjust the desired angular velocity for left turn

        # Check left side
        if left_distance < 1.0:
            # Obstacle at the left, turn right
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -0.5  # Adjust the desired angular velocity for right turn

        self.cmd_vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
        laser_scan_processor = LaserScanProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
