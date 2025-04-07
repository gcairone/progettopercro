#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco


with open('camera_calibration_usb.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Estrai i dati e convertili in numpy arrays
camera_matrix = np.array(data['camera_matrix'], dtype=np.float32)
dist_coeffs = np.array(data['dist_coeffs'][0], dtype=np.float32)




map_path = "map.yaml"
# cam_topic = "/alphasense_driver_ros/cam2"
cam_topic = "/usb_camera/image_raw"

def load_marker_map(yaml_file):
    with open(yaml_file, 'r') as f:
        marker_map = yaml.load(f, Loader=yaml.FullLoader)
    return marker_map





def average_pose(poses):
    if not poses:
        raise ValueError("La lista delle pose è vuota")
    
    t_mean = np.mean([t for _, t in poses], axis=0)
    
    R_stack = np.stack([R_mat for R_mat, _ in poses], axis=0)
    R_mean = average_rotation(R_stack)
    
    return R_mean, t_mean

def average_rotation(R_matrices):
    R_sum = np.sum(R_matrices, axis=0)
    
    U, _, Vt = np.linalg.svd(R_sum)
    
    R_mean = U @ Vt
    
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    
    return R_mean





def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    i = 0
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, corners[i], mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash



class ArucoTracker:
    def __init__(self):
        rospy.init_node("aruco_tracker", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(cam_topic, Image, self.image_callback)
        self.marker_length = 0.0955  # 100 mm (0.1 metri)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        self.marker_map = self.load_marker_map(map_path)

        rospy.loginfo("Aruco Tracker inizializzato")

    def load_marker_map(self, yaml_file):
        with open(yaml_file, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        
        marker_map = {}
        for marker in data['aruco_map']['markers']:
            marker_map[marker['id']] = {
                'position': marker['position'],
                'orientation': marker['orientation']
            }
        
        return marker_map
    
    def compute_camera_pose(self, rvec, tvec, marker_id):
        # rvec e tvec sono la pose del marker rispetto alla camera
        rvec_marker_world = np.array(self.marker_map[marker_id]['orientation'], dtype=np.float32).reshape(3, 1)
        t_marker_world = np.array(self.marker_map[marker_id]['position'], dtype=np.float32).reshape(3, 1)

        R_marker_world, _ = cv2.Rodrigues(rvec_marker_world)
        R_camera_marker, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float32).reshape(3, 1))
        # camera rispetto al marker
        R_marker_camera = R_camera_marker.T  
        t_marker_camera = -R_marker_camera @ np.array(tvec, dtype=np.float32).reshape(3, 1) 

        # camera rispetto al mondo
        R_camera_world = R_marker_world @ R_marker_camera
        t_camera_world = R_marker_world @ t_marker_camera + t_marker_world
        
        return R_camera_world, t_camera_world
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Errore nella conversione dell'immagine: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            # rospy.loginfo(f"Detected {len(ids)} markers")
            ids = ids.flatten()
            aruco.drawDetectedMarkers(frame, corners, ids)
            R_t_list = []
            tvec_list = []
            # i = 0
            for i in range(len(ids)):
                if ids[i] < 2:
                    continue
                rvec, tvec, _ = estimatePoseSingleMarkers(
                    corners[i], self.marker_length, camera_matrix, dist_coeffs
                )
                print(np.linalg.norm(corners[i][0][1]-corners[i][0][0]))
                tvec_list.append(tvec[0])
                print(f"Marker id:{ids[i]}")
                # print(f"Posizione del marker rispetto alla camera: \nTvec: {tvec}, \nR: {rvec}")

                # terna del marker
                # cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                R_camera_world, t_camera_world = self.compute_camera_pose(rvec[0], tvec[0], ids[i])
                R_t_list.append((R_camera_world, t_camera_world))



                # print(f"Posizione della camera rispetto a FIX: \nTvec: {t_camera_world}, \nR: {R_camera_world}")
            with open('logs/usbcam_small.txt', 'a') as f:
                for t1 in tvec_list:
                    for t2 in tvec_list:
                        d = np.linalg.norm(t1-t2)
                        print(f"distanza: {d}")
                        f.write(f"{d}\n")
                    # print(len(tvec_list))
            R_mean, t_mean = average_pose(R_t_list)
            # print(f"Pose MEDIA: \nTvec: {t_mean}, \nR: {R_mean}")

        else:
            print("NO MARKERS DETECTED")
        print()
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        cv2.imshow("Aruco Tracker", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        ArucoTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
