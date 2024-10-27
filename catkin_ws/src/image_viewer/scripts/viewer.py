#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Imu
import cv2
import numpy as np

# Inizializza le variabili globali per le immagini e i dati IMU
image_cam0 = None
image_cam1 = None
image_cam3 = None
image_cam4 = None
imu_data = None

def cam0_callback(msg):
    global image_cam0
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_cam0 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def cam1_callback(msg):
    global image_cam1
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_cam1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def cam3_callback(msg):
    global image_cam3
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_cam3 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def cam4_callback(msg):
    global image_cam4
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_cam4 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def imu_callback(msg):
    global imu_data
    imu_data = msg  # Salva i dati IMU

def main():
    # Inizializza il nodo ROS
    rospy.init_node('image_viewer', anonymous=True)

    # Sottoscrivi ai topic delle immagini e dei dati IMU
    rospy.Subscriber("/alphasense_driver_ros/cam0/compressed", CompressedImage, cam0_callback)
    rospy.Subscriber("/alphasense_driver_ros/cam1/compressed", CompressedImage, cam1_callback)
    rospy.Subscriber("/alphasense_driver_ros/cam3/compressed", CompressedImage, cam3_callback)
    rospy.Subscriber("/alphasense_driver_ros/cam4/compressed", CompressedImage, cam4_callback)
    rospy.Subscriber("/alphasense_driver_ros/imu", Imu, imu_callback)

    # Loop principale
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if image_cam0 is not None:
            cv2.imshow("Camera 0", image_cam0)
            cv2.waitKey(1)  # Aspetta un millisecondo

        if image_cam1 is not None:
            cv2.imshow("Camera 1", image_cam1)
            cv2.waitKey(1)  # Aspetta un millisecondo

        if image_cam3 is not None:
            cv2.imshow("Camera 3", image_cam3)
            cv2.waitKey(1)  # Aspetta un millisecondo

        if image_cam4 is not None:
            cv2.imshow("Camera 4", image_cam4)
            cv2.waitKey(1)  # Aspetta un millisecondo

        if imu_data is not None:
            # Stampa i dati IMU
            print("IMU Data:")
            print("Orientation:", imu_data.orientation)
            print("Angular Velocity:", imu_data.angular_velocity)
            print("Linear Acceleration:", imu_data.linear_acceleration)

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
