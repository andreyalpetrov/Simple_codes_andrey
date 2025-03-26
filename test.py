import rospy
from clover import srv
from std_srvs.srv import Trigger

import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('computer_vision_sample')
bridge = CvBridge()


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

import math

def navigate_wait(x=0, y=0, z=2, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)





def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    lower_red = np.array([170,150,150])
    upper_red = np.array([180,255,255])

    lower_red_2 = np.array([0,150,150])
    upper_red_2 = np.array([10,255,255])

    lower_green = np.array([55,240,162])
    upper_green = np.array([60,250,170])

    
    mask_red = cv.inRange(hsv, lower_red, upper_red)
    mask_red_2 = cv.inRange(hsv, lower_red_2, upper_red_2)
    mask_green = cv.inRange(hsv, lower_green, upper_green)

    bitwise_red = cv.bitwise_or(mask_red, mask_red_2)

    # M = cv.moments(mask_green)
    # if M['m00'] > 373320:
    #     print("green color detected")



    bitwise_all = cv.bitwise_or(bitwise_red, mask_green)

    contours, _ = cv.findContours(bitwise_all, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    cv.drawContours(cv_image, contours, -1, (0,255,50), 2)



    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
# 
# 
# navigate_wait(z=1, frame_id='body', auto_arm=True)

# navigate_wait(3,3,3)
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher("Mask", Image)
rospy.spin()