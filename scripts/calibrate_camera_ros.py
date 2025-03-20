#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CHESSBOARD_SIZE = (6, 9) 
SQUARE_SIZE = 0.030

objectPoints = []
imagePoints = []
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE
bridge = CvBridge()
frame_count = 0
calibration_done = False
cam_topic = "/alphasense_driver_ros/cam2"


def image_callback(msg):
    # rate = rospy.Rate(10) 
    global frame_count, calibration_done

    if calibration_done:
        return
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
    ret = None
    print("imagecallback")
    if ret:
        objectPoints.append(objp)
        imagePoints.append(corners)
        cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners, ret)
        frame_count += 1
        print(f"[INFO] Frame acquisiti: {frame_count}/20")
    cv2.imshow("Calibration", frame)
    cv2.waitKey(1)
    if frame_count >= 20:
        calibrate_camera(gray.shape[::-1])
    # rate.sleep()

def calibrate_camera(image_size):
    global calibration_done
    print("[INFO] Calibrating camera...")

    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
        objectPoints, imagePoints, image_size, None, None
    )

    print("\nMatrice Intrinseca:\n", cameraMatrix)
    print("\nCoefficienti di Distorsione:\n", distCoeffs)
    np.save("camera_matrix.npy", cameraMatrix)
    np.save("dist_coeffs.npy", distCoeffs)

    print("[INFO] Parametri salvati.")
    calibration_done = True
    cv2.destroyAllWindows()

def main():
    rospy.init_node("camera_calibration_node")
    rospy.Subscriber(cam_topic, Image, image_callback)
    print("[INFO] Attendi mentre vengono raccolti i frame per la calibrazione...")
    rospy.spin()

if __name__ == "__main__":
    main()
