from clover import srv
from std_srvs.srv import Trigger
import rospy
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover.srv import SetLEDEffect
from clover import long_callback
from threading import Lock

rospy.init_node('flight')

get_telemetry1 = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
bridge = CvBridge()
image_pub = rospy.Publisher('Color_check', Image)
telem_lock = Lock()

blueLow = (95, 19, 230)
blueHigh = (97, 240, 255)

s = 0
file = open("Report.txt", "w+")

Star = False
Arrows = False
KVADRAT = False
PRYAMOUG = False


def get_telemetry(frame_id):
    with telem_lock:
        return get_telemetry1(frame_id)


def navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=0, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)


def checkFigure(cnt):
    epsilon = 0.008 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    isSquere = False
    isTen = False

    x, y, w, h = cv2.boundingRect(cnt)

    if (len(approx) > 8):
        isTen = True
    if (abs(w - h) < 10):
        isSquere = True

    if (isTen and isSquere):
        return "STAR"
    elif (isTen):
        return "ARROWS"
    elif (isSquere):
        return "KVADRAT"
    else:
        return "PRYAMOUGILNIK"


def checkFigure1(cnt):
    epsilon = 0.008 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    isSquere = False
    isTen = False

    x, y, w, h = cv2.boundingRect(cnt)

    if (len(approx) > 8):
        isTen = True
    if (abs(w - h) < 10):
        isSquere = True

    if (isTen and isSquere):
        return "STAR"
    elif (isTen):
        return "ARROWS"
    elif (isSquere):
        return "KVADRAT"
    else:
        return "PRYAMOUGILNIK"


@long_callback
def image_callback(data):
    global ch, s, Star, Arrows, PRYAMOUG, KVADRAT
    s += 1
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    hsv = cv2.cvtColor(img[90:160, 130:190], cv2.COLOR_BGR2HSV)
    hsv2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    blueMask = cv2.inRange(hsv, blueLow, blueHigh)
    contours = cv2.findContours(blueMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
    blueMom = cv2.moments(blueMask, 1)

    try:
        blueCoord = (int(blueMom['m10'] / blueMom['m00']), int(blueMom['m01'] / blueMom['m00']))
    except ZeroDivisionError:
        ...

    telem = get_telemetry(frame_id='aruco_map')
    coord = (round(telem.x, 2), round(telem.y, 2))
    m = blueMom["m00"]
    if (m > 10):
        figure = checkFigure(contours[0])
        print(m)
        if (figure == "STAR" and not Star):
            file.write(f"STAR - {coord}\n")
            print(f"{figure} - {coord}")
            Star = True

        elif (figure == "ARROWS" and not Arrows and m > 240):
            file.write(f"ARROWS - {coord}\n")
            print(f"{figure} - {coord}")
            Arrows = True

        elif (figure == "KVADRAT" and not KVADRAT and m > 1000):
            file.write(f"KVADRAT - {coord}\n")
            print(f"{figure} - {coord}")
            KVADRAT = True

        elif (figure == "PRYAMOUGILNIK" and not PRYAMOUG):
            file.write(f"PRYAMOUG - {coord}\n")
            print(f"{figure} - {coord}")
            PRYAMOUG = True

    # For topic
    blueMask = cv2.inRange(hsv2, blueLow, blueHigh)
    cnt = cv2.findContours(blueMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
    for c in cnt:
        blueCoord = (-100, 0)
        try:
            blueMom = cv2.moments(c, 1)
            blueCoord = (int(blueMom['m10'] / blueMom['m00']), int(blueMom['m01'] / blueMom['m00']))
        except ZeroDivisionError:
            ...

        if (blueCoord[0] != -100):
            img = cv2.putText(img, f"{checkFigure1(c)}", blueCoord, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                              (0, 0, 0), 1, cv2.LINE_AA, False)
            img = cv2.rectangle(img, (blueCoord[0] - 20, blueCoord[1] - 20), (blueCoord[0] + 20, blueCoord[1] + 20),
                                (255, 0, 255), 1)

    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


navigate(z=1, frame_id='body', auto_arm=True)
rospy.sleep(5)

navigate_wait(0, 0)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

for i in range(7):
    if (i % 2 == 0):
        navigate_wait(i / 2, 0)
        navigate_wait(i / 2, 5)
    else:
        navigate_wait(i / 2, 5)
        navigate_wait(i / 2, 0)

navigate_wait(0, 0)
land()
file.close()