import rospy
from clover import srv
from clover.srv import SetLEDEffect
from threading import Lock
from std_srvs.srv import Trigger
import math
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Range

rospy.init_node('flight')
telem_lock = Lock()

bridge = CvBridge()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)


def get_telemetry_locked(frame_id):
    with telem_lock:
        return get_telemetry(frame_id)

def navigate_wait(x=0, y=0, z=1.5, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry_locked(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
maxh = 0
def height_building(msg):
    global build_heig, maxh

    telem = get_telemetry_locked(frame_id='aruco_map')
    height_global = telem.z
    build_heig = height_global - msg.range
    if build_heig > maxh:
        maxh = build_heig




detect_blue = 0
detect_green = 0
detect_red = 0

blue_build = True
green_build = True
red_build = True

def image_callback(data):
    global maxh, detect_blue, blue_build, green_build, detect_green, detect_red, red_build
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv_image = cv_image[60:180, 80:240]
    img_hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    kernel = np.ones((3,3),np.uint8)

    lower_blue = np.array([88,196,57])
    upper_blue = np.array([105,255,255])
    mask_blue = cv.inRange(img_hsv, lower_blue, upper_blue)

    opening = cv.morphologyEx(mask_blue, cv.MORPH_OPEN, kernel)
    mask_blue = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)

    contours_blue, _ = cv.findContours(mask_blue, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    cv.drawContours(cv_image, contours_blue, -1, (50,255,0), 3)
    M = cv.moments(mask_blue)
    if M['m00'] > 45:
        detect_blue = detect_blue + 1


    if M['m00'] < 45 and detect_blue > 0 and blue_build:
        print("blue building detected! height = ")
        print(build_heig)
        maxh = 0
        blue_build = False

    lower_green = np.array([50,53,70])
    upper_green = np.array([67,200,245])
    mask_green = cv.inRange(img_hsv,lower_green, upper_green)

    opening = cv.morphologyEx(mask_green, cv.MORPH_OPEN, kernel)
    mask_green = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)

    contours_green, _ = cv.findContours(mask_green, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    cv.drawContours(cv_image, contours_green, -1, (50,255,0), 3)
    M = cv.moments(mask_green)
    if M['m00'] > 45:
        detect_green = detect_green + 1


    if M['m00'] < 45 and detect_green > 0 and green_build:
        print("green building detected! height = ")
        print(build_heig)
        maxh = 0
        green_build = False


    lower_red_1 = np.array([167,205,180])
    upper_red_1 = np.array([180,255,199])
    mask_red_1 = cv.inRange(img_hsv, lower_red_1, upper_red_1)

    lower_red_2 = np.array([0,145,55])
    upper_red_2 = np.array([13,230,234])
    mask_red_2 = cv.inRange(img_hsv, lower_red_2, upper_red_2)
    bitwise_red = cv.bitwise_or(mask_red_1, mask_red_2)

    opening = cv.morphologyEx(bitwise_red, cv.MORPH_OPEN, kernel)
    bitwise_red = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)


    contours_red, _ = cv.findContours(bitwise_red, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    cv.drawContours(cv_image, contours_red, -1, (50,255,0), 3)

    M = cv.moments(bitwise_red)
    if M['m00'] > 45:
        detect_red = detect_red + 1


    if M['m00'] < 45 and detect_red > 0 and red_build:
        print("red building detected! height = ")
        print(build_heig)
        maxh = 0
        red_build = False


    image_pub_con.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))



image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

rospy.Subscriber('rangefinder/range', Range, height_building)
image_pub_con = rospy.Publisher('Conturs', Image)


navigate_wait(x = 0,y = 0, z = 1.5, frame_id="body", auto_arm=True)
rospy.sleep(0.5)
navigate_wait(x = 0.5, y = 2, frame_id="aruco_map")
rospy.sleep(0.5)
navigate_wait(x = 1.5, y = 0.5, frame_id="aruco_map")
rospy.sleep(0.5)
navigate_wait(x = 2.5, y = 0.5, frame_id="aruco_map")
rospy.sleep(0.5)
navigate_wait(x = 0, y = 0, frame_id="aruco_map")
rospy.sleep(0.5)
land()









