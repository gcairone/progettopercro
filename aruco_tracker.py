#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco


camera_matrix = np.array([[600, 0, 320], 
                          [0, 600, 240], 
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)  
map_path = "map.yaml"
cam_topic = "/webcam/image_raw"

def load_marker_map(yaml_file):
    with open(yaml_file, 'r') as f:
        marker_map = yaml.load(f, Loader=yaml.FullLoader)
    return marker_map

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
            print(f"Detected {len(ids)} markers")
            ids = ids.flatten()
            aruco.drawDetectedMarkers(frame, corners, ids)

            i = 0
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], self.marker_length, camera_matrix, dist_coeffs
            )
            print(f"Marker id:{ids[i]}")

            # terna del marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

            R_camera_world, t_camera_world = self.compute_camera_pose(rvec, tvec, ids[i])
            print(f"Posizione della camera rispetto a FIX: Tvec: {t_camera_world}, Rvec: {R_camera_world}")
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
