import rospy
from clover.srv import SetLEDEffect
from clover import srv
from std_srvs.srv import Trigger
import math
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

rospy.init_node('flight')


bridge = CvBridge()
ledka = False
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

def navigate_wait(x=0, y=0, z=1.5, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def image_callback(data):
    global ledka
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv_image = cv_image[60:180, 80:240]
    img_hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    lower_blue = np.array([101,79,82])
    upper_blue = np.array([108,108,138])
    mask = cv.inRange(img_hsv, lower_blue, upper_blue)

    mask_2 = cv.inRange(img_hsv,(97,181,90), (101,230,115))

    bitwise_blue = cv.bitwise_or(mask, mask_2)

    M = cv.moments(bitwise_blue)
    print(M['m00'])
    if M['m00'] > 10455:
        set_effect(r=0, g=250, b=0)
    if M['m00'] < 10455:
        set_effect(r=0, g=0, b=0)
        image_pub.publish(bridge.cv2_to_imgmsg(bitwise_blue, 'mono8'))

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
image_pub = rospy.Publisher('CV_mask', Image)

navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True)
rospy.sleep(1)
navigate_wait(x = 0, y = 0)
rospy.sleep(0.2)
navigate_wait(x = 1, y = 5)
rospy.sleep(0.8)
land()

    

