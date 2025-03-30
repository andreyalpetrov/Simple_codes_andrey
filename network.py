import math
import os
import sys
from threading import Lock, Thread
import rospy
from clover import srv
from std_srvs.srv import Trigger
from ultralytics import YOLO
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json

# Инициализация сервисов управления дроном
get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
navigate = rospy.ServiceProxy("navigate", srv.Navigate)
navigate_global = rospy.ServiceProxy("navigate_global", srv.NavigateGlobal)
set_position = rospy.ServiceProxy("set_position", srv.SetPosition)
set_velocity = rospy.ServiceProxy("set_velocity", srv.SetVelocity)
set_attitude = rospy.ServiceProxy("set_attitude", srv.SetAttitude)
set_rates = rospy.ServiceProxy("set_rates", srv.SetRates)
land = rospy.ServiceProxy("land", Trigger)

telem_lock = Lock()
bridge = CvBridge()
detected_objects = []
do_detect = False
model = YOLO("model_drone.pt")  # Замените путь на ваш путь к модели

os.chdir(os.path.dirname(sys.argv[0]))


def get_telemetry_locked(frame_id=""):
    with telem_lock:
        telem = get_telemetry(frame_id=frame_id)
    return telem


def distance(p0, p1):
    return (p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2


def navigate_wait(x=0, y=0, z=0, yaw=1.57, speed=1.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    """Навигация и ожидание пока дрон достигнет точки назначения"""
    print("Going to x: {} y: {}".format(x, y))
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry_locked(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
    print("Target reached")


def detect_objects(image_msg):
    """Распознавание объектов на кадре и запись их координат"""
    global do_detect, detected_objects
    if not do_detect:
        return

    # Конвертация ROS-сообщения в OpenCV-изображение
    frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    telem = get_telemetry_locked(frame_id='aruco_map')

    # Распознавание объектов с использованием модели
    results = model(frame)

    # Сохранение координат обнаруженных объектов
    for result in results[0].boxes:
        class_name = model.names[int(result.cls)]
        if class_name in ["apple", "banana", "strawberry", "cherry"]:
            bbox = result.xyxy[0].tolist()  # [x1, y1, x2, y2]
            detected_objects.append({
                "class": class_name,
                "coordinates": [bbox[0], bbox[1], bbox[2], bbox[3]],
                "drone_position": [telem.x, telem.y, telem.z]
            })

    # Запись обнаруженных объектов в файл
    with open("detected_objects.json", "w") as f:
        json.dump(detected_objects, f, indent=4)

    # (Опционально) Отображение результатов
    annotated_frame = results[0].plot()
    cv2.imshow("Detection", annotated_frame)
    cv2.waitKey(1)


def main_flight(z=1.0):
    # Подписка на топик изображения
    rospy.Subscriber("/main_camera/image_raw", Image, detect_objects)

    # Взлет
    navigate(x=0, y=0, z=1, speed=1.5, frame_id='body', auto_arm=True, yaw=float('nan'))
    rospy.sleep(2)
    print("Took off")
    navigate_wait(x=0, y=0, z=z, speed=0.5)
    print("Start position")

    global do_detect
    do_detect = True  # Включаем детектирование

    direction_forward = True

    # Полет по схеме "змейка"
    for x in range(0, 10, 1):
        if direction_forward:
            navigate_wait(x=x, y=0, z=z)
            navigate_wait(x=x, y=9, z=z)
        else:
            navigate_wait(x=x, y=9, z=z)
            navigate_wait(x=x, y=0, z=z)

        # Переключение направления
        direction_forward = not direction_forward

    # Возвращение к стартовой точке и посадка
    navigate_wait(x=0, y=0, z=z, frame_id="aruco_map")
    do_detect = False
    print("Landing")
    land()


if __name__ == "__main__":
    rospy.init_node("flight_with_detection", anonymous=True)
    main_flight()