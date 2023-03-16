import rospy
import math
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover.srv import SetLEDEffect
from clover import srv
from std_srvs.srv import Trigger
#импорт библиотек


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

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

hsv_min = np.array((100, 100, 100), np.uint8)
hsv_max = np.array((180, 255, 200), np.uint8)

def image_callback2(data):    #функция для выделения контуров и цетра фигуры
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv, hsv_min, hsv_max ) 
    contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    contours = list(filter(lambda x: cv.contourArea(x) > 1, contours))
    black_white = cv.bitwise_and(cv_image, cv_image, mask=thresh)
    if len(contours) != 0:
        cont = max(contours, key=cv.contourArea)

        epsilon = 0.1*cv.arcLength(cont,True)
        approx = cv.approxPolyDP(cont,epsilon,True)

        cv.drawContours(cv_image, cont, -1, (255,0,0), 3, cv.LINE_AA)
        moments = cv.moments(cont, 1)
        dM01 = moments['m01']
        dM10 = moments['m10']
        dArea = moments['m00']
        if dArea > 100:
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(cv_image, (x, y), 10, (0,255,254), -1)
            cv.putText(cv_image, (str(x)), (x, y), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 200), 2)
            cv.putText(cv_image, (str(y)), (x, y-30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 200), 2)

    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


def image_callback(data):    #функция для унавания распознаваемых он с помощью hsv
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv, hsv_min, hsv_max ) 
    contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours = list(filter(lambda x: cv.contourArea(x) > 1, contours))
    black_white = cv.bitwise_and(cv_image, cv_image, mask=thresh)
    if len(contours) != 0:
        cont = max(contours, key=cv.contourArea)
        cv.drawContours(cv_image, cont, -1, (255,0,0), 3, cv.LINE_AA)
        moments = cv.moments(cont, 1)
        dM01 = moments['m01']
        dM10 = moments['m10']
        dArea = moments['m00']
        if dArea > 100:
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(cv_image, (x, y), 10, (0,255,254), -1)
            cv.putText(cv_image, (str(x)), (x, y), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 200), 2)
            cv.putText(cv_image, (str(y)), (x, y-30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 200), 2)

    image_pub.publish(bridge.cv2_to_imgmsg(black_white, 'bgr8')) 


# def navigate_wait(x=0, y=0, z=1.5, yaw=0, speed=1, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
#     navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

#     while not rospy.is_shutdown():
#         telem = get_telemetry(frame_id='navigate_target')
#         if math.sqrt(telem.x2 + telem.y2 + telem.z**2) < tolerance:
#             break
#         rospy.sleep(0.2)


# def square(side=2): (полёт по квадрату)
#     navigate_wait(side, 0, 0, frame_id='navigate_target')
#     telemetry = get_telemetry()
#     set_effect(r=255, g=0, b=0)
#     print(telemetry.x, telemetry.y, telemetry.z)
#     navigate_wait(0, side, 0, frame_id='navigate_target')
#     set_effect(r=0, g=255, b=0)
#     telemetry = get_telemetry()
#     print(telemetry.x, telemetry.y, telemetry.z)
#     navigate_wait(-side, 0, 0, frame_id='navigate_target')
#     set_effect(r=0, g=0, b=255)
#     telemetry = get_telemetry()
#     print(telemetry.x, telemetry.y, telemetry.z)
#     navigate_wait(0, -side, 0, frame_id='navigate_target')
#     set_effect(r=100, g=100, b=0)


rospy.Subscriber('main_camera/image_raw', Image, image_callback2, queue_size=1)
image_pub = rospy.Publisher('~debug', Image, queue_size = 1)
rospy.spin()
