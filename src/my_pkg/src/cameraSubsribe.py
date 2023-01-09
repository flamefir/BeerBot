#!/usr/bin/env python2
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import rosbag
import rospy

# Global variable to store the latest message from the publisher
latest_message = None

def callback(msg):
    # Update the global variable with the latest message
    global latest_message
    latest_message = msg.data

def service_callback(request):
    # Return the latest message as the service response
    response = TriggerResponse()
    response.success = True
    response.message = latest_message
    return response

# Initialize the node
rospy.init_node('my_node')

# Create a publisher
pub = rospy.Publisher('my_topic', String, queue_size=10)

# Create a subscriber
sub = rospy.Subscriber('CupInfo', String, callback)

# Create a service server
service = rospy.Service('get_latest_message', Trigger, service_callback)

# Spin until interrupted
rospy.spin()
