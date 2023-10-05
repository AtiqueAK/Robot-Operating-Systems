#!/usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist
import math

def move_bb8_in_square(request):
    rospy.loginfo("Moving BB-8 in a square...")

    # Define the side length, repetitions, and velocity from the request
    side_length = request.side
    repetitions = request.repetitions
    velocity = request.velocity

    # Initialize a publisher for the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # Adjust the rate as needed for the desired movement speed

    # Create a Twist message for linear and angular velocities
    twist_msg = Twist()

    # Calculate the time needed to travel one side length
    side_time = side_length / velocity

    for _ in range(repetitions):
        for _ in range(4):
            # Move forward (one side of the square)
            twist_msg.linear.x = velocity
            twist_msg.angular.z = 0.0
            pub.publish(twist_msg)
            rospy.sleep(side_time)

            # Stop
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            pub.publish(twist_msg)

            # Rotate (turning 90 degrees)
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = math.pi / 2  # 90 degrees in radians
            pub.publish(twist_msg)
            rospy.sleep(1.0)  # Adjust rotation time as needed

            # Stop
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            pub.publish(twist_msg)

    return BB8CustomServiceMessageResponse(True)

if __name__ == '__main__':
    rospy.init_node('bb8_move_in_square_service_server')

    # Create a service for /move_bb8_in_square_custom
    service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage, move_bb8_in_square)
    rospy.loginfo("Ready to move BB-8 in a square.")
    rospy.spin()
