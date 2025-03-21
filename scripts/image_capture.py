#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

num = 0
CHESSBOARD_SIZE = (5, 7)
bridge = CvBridge()
cam_topic = "/alphasense_driver_ros/cam2"
def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    cv2.imshow("Calibration", frame)
    key = cv2.waitKey(1)
    global num 
    if key == ord(" "):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
        if not ret:
            return
        cv2.imwrite(f"image{num}.jpg", frame)
        num = num+1
        print(f"image saved {num}")
    

def main():
    rospy.init_node("camera_calibration_node")
    rospy.Subscriber(cam_topic, Image, image_callback)
    print("[INFO] Attendi mentre vengono raccolti i frame per la calibrazione...")
    rospy.spin()

if __name__ == "__main__":
    main()
