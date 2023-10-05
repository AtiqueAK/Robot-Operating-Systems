#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse  # Import the service message python classes generated from Empty.srv

# Function to call the move_bb8_in_circle service
def move_bb8_in_circle_client():
    rospy.wait_for_service('/move_bb8_in_circle')
    try:
        move_bb8_in_circle = rospy.ServiceProxy('/move_bb8_in_circle', Empty)
        response = move_bb8_in_circle()
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

# Callback function for the my_service
def my_callback(request):
    rospy.loginfo("my_service has been called. Calling move_bb8_in_circle service...")
    move_bb8_in_circle_client()  # Call the move_bb8_in_circle service
    return EmptyResponse()  # Return an EmptyResponse

if __name__ == '__main__':
    rospy.init_node('service_server')
    rospy.loginfo("Initializing service_server node...")

    # Create a service for the my_service
    my_service = rospy.Service('/my_service', Empty, my_callback)
    rospy.loginfo("Ready to respond to service calls on /my_service.")

    rospy.spin()  # Maintain the service open.
