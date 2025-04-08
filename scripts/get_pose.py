import cv2
import numpy as np
import re
import ast
import yaml


def load_camera_parameters(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    
    camera_matrix = np.array(data['camera_matrix'], dtype=np.float32)
    dist_coeffs = np.array(data['dist_coeffs'], dtype=np.float32).flatten()  # Flatten per un array 1D
    
    return camera_matrix, dist_coeffs



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


def load_marker_map(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    
    marker_map = {}
    for marker in data['aruco_map']['markers']:
        marker_map[marker['id']] = {
            'orientation': np.array(marker['orientation'], dtype=np.float32),
            'position': np.array(marker['position'], dtype=np.float32)
        }
    return marker_map


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


def compute_camera_pose(rvec, tvec, marker_id):
    # rvec e tvec sono la pose del marker rispetto alla camera
    rvec_marker_world = np.array(marker_map[marker_id]['orientation'], dtype=np.float32).reshape(3, 1)
    t_marker_world = np.array(marker_map[marker_id]['position'], dtype=np.float32).reshape(3, 1)

    R_marker_world, _ = cv2.Rodrigues(rvec_marker_world)
    R_camera_marker, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float32).reshape(3, 1))
    # camera rispetto al marker
    R_marker_camera = R_camera_marker.T  
    t_marker_camera = -R_marker_camera @ np.array(tvec, dtype=np.float32).reshape(3, 1) 

    # camera rispetto al mondo
    R_camera_world = R_marker_world @ R_marker_camera
    t_camera_world = R_marker_world @ t_marker_camera + t_marker_world
    
    return R_camera_world, t_camera_world



def process_file(filepath, marker_length, camera_matrix, dist_coeffs):
    with open(filepath, 'r') as f:
        content = f.read()

    # Match ogni frame con corners, ids e timestamp
    pattern = r'(\[.*?\])\s*\[\[(.*?)\]\]\s*Timestamp:\s*([0-9.]+)'
    matches = re.findall(pattern, content, re.DOTALL)
    results = []  
    for corners_raw, ids_str, timestamp in matches:
        cleaned = corners_raw.replace('array', 'np.array').replace('dtype=float32', '')
        
        try:
            corners_list = eval(cleaned, {"np": np})
            corners = np.array(corners_list, dtype=np.float32)
        except Exception as e:
            print(f"Errore parsing marker block: {e}")
            continue

        try:
            ids = np.array(eval("[" + ids_str.replace("\n", ",") + "]")).reshape(-1, 1)
        except Exception as e:
            print(f"Errore parsing IDs: {e}")
            continue




        tvec_list = []
        R_t_list = []

        # Calcolo della posa per ogni marker
        for i in range(len(ids)):
            rvec, tvec, _ = estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, dist_coeffs)
            tvec_list.append(tvec[0])
            # print(f"Marker id:{ids[i][0]}")
            # print(f"Posizione del marker rispetto alla camera: \nTvec: {tvec}, \nR: {rvec}")

            R_camera_world, t_camera_world = compute_camera_pose(rvec[0], tvec[0], ids[i][0])
            R_t_list.append((R_camera_world, t_camera_world))

            # print(f"Posizione della camera rispetto a FIX: \nTvec: {t_camera_world.T}, \nR:\n{R_camera_world}")

        # Media delle pose
        R_mean, t_mean = average_pose(R_t_list)
        print(timestamp)
        print(f"Pose MEDIA: \nTvec: {t_mean.T}, \nR:\n{R_mean}\n")


        results.append({
            'timestamp': float(timestamp),
            'R': R_mean.tolist(),
            'T': t_mean.tolist()
        })
    with open(output_yaml, 'w') as yaml_file:
        yaml.dump({'frames': results}, yaml_file, default_flow_style=False)

output_yaml = "poses_path_3.yaml"
marker_map = load_marker_map("aruco_map_lab.yaml")
camera_matrix, dist_coeffs = load_camera_parameters("camera_calibration.yaml")
marker_length = 0.159
process_file("logs/aruco_corners_3.txt", marker_length, camera_matrix, dist_coeffs)