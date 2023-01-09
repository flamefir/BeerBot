#!/usr/bin/env python2
import cv2
import numpy as np
import threading
import Queue as queue
import rospy as ros
import signal
from std_msgs.msg import String
import math 


### SETUP ###
# Function for adjusting contrast without changing brightness
def adjust_contrast_brightness(img, contrast=1.0, brightness=0):
    """
    Adjusts contrast and brightness of an uint8 image.
    contrast:   (0.0,  inf) with 1.0 leaving the contrast as is
    brightness: [-255, 255] with 0 leaving the brightness as is
    """
    brightness += int(round(255*(1-contrast)/2))
    return cv2.addWeighted(img, contrast, img, 0, brightness)

# Handler for stopping the program with ctrl-c. Releases camera gracefully.
def sigint_handler(signum, frame):
    print("\nimgprocess.py interrupt recieved")
    global flag
    flag = True
    t.join()
    cap.release()
    cv2.destroyAllWindows()

signal.signal(signal.SIGINT, sigint_handler)

def MessagePublisher(message, topic):

    #create topic
    topic = ros.Publisher(topic, String, queue_size=10)

    ros.init_node('messagePubNode')

    rate = ros.Rate(2)

    #Keep publishing the messages until the user interrupts

    #display to terminal 
    ros.loginfo("Pubslished: " + message)
    #Publish to topic
    topic.publish(message)

    #wait until node is published
    rate.sleep()


# Create queue for threading. Max size is 1 so ROS message will take the newest frame.
q = queue.Queue(maxsize=1)

# Flag for thread
flag = False

# Open the camera and grab center pixel
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error opening video stream. Add removable device or change VideoCapture(0) to 1")

# Set the width and height of the capture
cap.set(3, 1920)
cap.set(4, 1080)

# Center of frame
framecenter_hz = []

### LOOP + MAIN ###

def loop_thread():
    while not flag:
        # Capture frame-by-frame
        _, frame = cap.read()

        # Cut image to view of cups
        frame = frame[420:600, 820:1100] #+- 120, 60 vertical +- 140 horzontal from center of 1920*1080

        #Center of modified frame
        framecenter_hz.append(frame.shape[1]/2)

        # Blur the image
        blur = cv2.GaussianBlur(frame, (5,5), 0)
        blur = cv2.GaussianBlur(blur, (3,3), 0)

        # Convert to grayscale
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # Increase the contrast. First equalize the distribution of color(gray) then increaase.
        contrastimg = cv2.equalizeHist(gray)

        contrastimg2 = adjust_contrast_brightness(contrastimg, contrast=4.0, brightness=-175) #Nat_: 1, 0

        # Convert to black and white using a threshold value
        ret, bw = cv2.threshold(contrastimg2, 254, 255, cv2.THRESH_BINARY) #Nat: 254-255

        #Erosion and dilation
        erokernel = np.ones((3,3),np.uint8)
        dilkernel = np.ones((3,3),np.uint8)

        #ero = cv2.erode(bw,erokernel,cv2.BORDER_REFLECT)
        ero = cv2.dilate(bw,dilkernel,cv2.BORDER_REFLECT)

        im2, contours, hierarchy = cv2.findContours(ero, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        # Create iterator for enumerating ellipses.
        i = 0
        ellipses = []
        # Iterate through the contours
        for c in contours:
            if  cv2.contourArea(c) > 40 and cv2.contourArea(c) < 700 and cv2.arcLength(c,True) > 80:
                hull = cv2.convexHull(c)
                if len(hull) >= 5: #fitEllipse throws exception with less than 5    
                    ellipse = cv2.fitEllipse(hull)
                    # Draw the ellipse on the image
                    center = (int(ellipse[0][0]), int(ellipse[0][1]))
                    ellipses.append((center, ellipse))
                    # Put the label on the image
        
        # Sort ellipses bottom to top, then left to right
        ellipses = sorted(ellipses, key=lambda e: (e[0][1], e[0][0]), reverse=True)
        
        #Pop first element of queue if not empty. Prevents queue blocking.
        while not q.empty():
            q.get()

        if len(ellipses) > 0:
            q.put(ellipses)

        # Draw ellipses
        if len(ellipses) > 0:
            for i, (center, ellipse) in enumerate(ellipses):
                cv2.ellipse(frame, ellipse, (0, 255, 0), 2)

                temp_ellipse = ellipses[i][1]
                _,axes,_ = temp_ellipse

                cv2.putText(frame, str(i + 1), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            #cv2.putText(frame, str(int(center[0])) + "," + str(int(center[1])), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            #cv2.putText(frame, str(round(axes[0],2)) + "," + str(round(axes[1],2)), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


        # Draw ALL the contours on the original image
        #cv2.drawContours(frame, contours, -1, (0,255,0), 3)

        # Display the image with ellipses drawn
        cv2.imshow('Contours', ero)
        cv2.imshow('Ellipses', frame)

        # Check if the user hit the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Start loop thread
print("Interrupt imgprocess.py with CTRL-C + CTRL-Z")
t = threading.Thread(target=loop_thread)
t.daemon = True 
t.start()

while True:
    try:
        ellipses = q.get()
    except:
        print("Full queue for ellipes")
    
    #Getting axis values to determine width
    center = ellipses[0][0] #center was packed beside to prevent unpacking the ellipses object so many times.
    ellipse1 = ellipses[0][1]
    _,axes,_ = ellipse1
    cup_width_pix = round(float(axes[1]),3)

    #Offset from camera center pixel
    offset = int(center[0]-framecenter_hz[0])

    cam_focal = 1428 #Value documented in report
    cup_width = 0.091 #Measured (m)

    distance_hypo = (cup_width*cam_focal)/cup_width_pix #From camera to center of front cup
    h=0.969 #From camera to base, vertical
    d=0.425 #Camera to base, horizontal

    if distance_hypo > 0 and h < distance_hypo: #math domain error checking
        distance = math.sqrt((distance_hypo**2)-(h**2))-d

    offset_meters = (offset*distance)/cam_focal
    offset_angle = math.atan(offset_meters/distance)*(180 / math.pi)

    if len(ellipses) < 1:
        message = "False/False"
    else:
        message = str(round(distance*100,2)) + "/" + str(round(offset_angle,2))

    #print(message)
    #print(offset_meters)

    MessagePublisher(message, "CupInfo")


# #publish 2 messages / per second
# def MessagePublisher(message, topic):

#     #init publisher node
#     rospy.init_node('messagePubNode')

#     #create topic
#     topic = rospy.Publisher(topic, String, queue_size=10)

#     #publish rate
#     rate = rospy.Rate(2)

#     #Keep publishing the messages until the user interrupts
#     while not rospy.is_shutdown():

#         #display to terminal 
#         rospy.loginfo("Pubslished: " + message)
#         #Publish to topic
#         topic.publish(message)

#         #wait until node is published
#         rate.sleep()

# def main():

#     MessagePublisher("160/-179.99", "CupInfo")

# if "__name__" == main():
#     main()