#!/usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest

def move_bb8_in_square_client():
    # Initialize the ROS node
    rospy.init_node('bb8_move_in_square_service_client')

    # Wait for the /move_bb8_in_square_custom service to become available
    rospy.wait_for_service('/move_bb8_in_square_custom')

    try:
        # Create a service proxy to call the service
        move_bb8_in_square = rospy.ServiceProxy('/move_bb8_in_square_custom', BB8CustomServiceMessage)

        # Set the desired velocity and side length for a small square
        small_square_velocity = 0.2  # Example velocity for a small square
        small_square_side_length = 1.0  # Example side length for a small square

        # Create a service request object for a small square
        small_square_request = BB8CustomServiceMessageRequest()
        small_square_request.side = small_square_side_length
        small_square_request.repetitions = 2
        small_square_request.velocity = small_square_velocity

        # Call the service with the request for a small square
        small_square_response = move_bb8_in_square(small_square_request)

        # Check if the service call for a small square was successful
        if small_square_response.success:
            rospy.loginfo("BB-8 successfully moved in a small square.")
        else:
            rospy.logerr("BB-8 failed to move in a small square.")

        # Set the desired velocity and side length for a big square
        big_square_velocity = 0.1  # Example velocity for a big square
        big_square_side_length = 2.0  # Example side length for a big square

        # Create a service request object for a big square
        big_square_request = BB8CustomServiceMessageRequest()
        big_square_request.side = big_square_side_length
        big_square_request.repetitions = 1
        big_square_request.velocity = big_square_velocity

        # Call the service with the request for a big square
        big_square_response = move_bb8_in_square(big_square_request)

        # Check if the service call for a big square was successful
        if big_square_response.success:
            rospy.loginfo("BB-8 successfully moved in a big square.")
        else:
            rospy.logerr("BB-8 failed to move in a big square.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    move_bb8_in_square_client()
