#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco

"""
cam2 rispetto a imu in coordinate omogenee
data: [-0.999882532472986, -0.004601227925155, -0.01462019004626, 0.010305663957831, 
        -0.004654198203865, 0.999982720947361, 0.00359113433205, -0.00680945377751, 
        0.014603413795655, 0.003658757752635, -0.99988667047682, -0.035041702625671, 
        0.0, 0.0, 0.0, 1.0]

"""
camera_matrix = np.array([[348.2598, 0, 360.4975], 
                          [0, 348.1871, 267.4945], 
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.array([-0.038939327701505, -0.002920367195425, 0.000792105310707, -0.000368236895244], dtype=np.float32)  
map_path = "map.yaml"
cam_topic = "/alphasense_driver_ros/cam2"

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

class ArucoTracker:
    def __init__(self):
        rospy.init_node("aruco_tracker", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(cam_topic, Image, self.image_callback)
        self.marker_length = 0.1  # 100 mm (0.1 metri)
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
            rospy.loginfo(f"Detected {len(ids)} markers")
            ids = ids.flatten()
            aruco.drawDetectedMarkers(frame, corners, ids)
            R_t_list = []
            tvec_list = []
            # i = 0
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_length, camera_matrix, dist_coeffs
                )
                tvec_list.append(tvec)
                print(f"Marker id:{ids[i]}")
                print(f"Posizione del marker rispetto alla camera: \nTvec: {tvec}, \nR: {rvec}")

                # terna del marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                R_camera_world, t_camera_world = self.compute_camera_pose(rvec, tvec, ids[i])
                R_t_list.append((R_camera_world, t_camera_world))



                print(f"Posizione della camera rispetto a FIX: \nTvec: {t_camera_world}, \nR: {R_camera_world}")
            if len(tvec_list)==2:
                print(f"distanza {np.linalg.norm(tvec_list[0]-tvec_list[1])}")
            R_mean, t_mean = average_pose(R_t_list)
            print(f"Pose MEDIA: \nTvec: {t_mean}, \nR: {R_mean}")

        else:
            print("NO MARKERS DETECTED")
        print()
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
