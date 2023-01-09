#!/usr/bin/env python2
import rospy
from std_msgs.msg import String

#publish 2 messages / per second
def MessagePublisher(message, topic):

    #init publisher node
    rospy.init_node('messagePubNode')

    #create topic
    topic = rospy.Publisher(topic, String, queue_size=10)

    #publish rate
    rate = rospy.Rate(2)

    #Keep publishing the messages until the user interrupts
    while not rospy.is_shutdown():

        #display to terminal 
        rospy.loginfo("Pubslished: " + message)
        #Publish to topic
        topic.publish(message)

        #wait until node is published
        rate.sleep()

def main():

    MessagePublisher("160/-179.99", "CupInfo")

if "__name__" == main():
    main()