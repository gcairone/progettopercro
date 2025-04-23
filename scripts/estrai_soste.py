import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tf_trans
"""
prova1:
1: 0.9317188417113824                       │process[master]: started with pid [57]
2: 1.7935997323817814                       │ROS_MASTER_URI=http://pc-huawei:11311/
3: 1.4557815770231473                       │
4: 1.2881770064707718                       │setting /run_id to 8ab6ced4-16e0-11f0-b2a7-28cdc
5: 4.636744547632531


"""
# Trasformazione
alpha = -1.76
R_transf = np.array([[np.cos(alpha), np.sin(alpha), 0.0],
               [-np.sin(alpha), np.cos(alpha), 0.0],
               [0.0, 0.0, 1.0]], dtype=np.float32)
t_transf = np.array([0, 0, 0.0], dtype=np.float32)
t_transf = np.array([20.0, 14.0, 6.0])
true_points_prova_1 = [[0.0, 0.0, 0.0], 
               [0.0, -16.8, 0.0],
               [25.2, -16.8, 0.0],
               [25.2, 0.0, 0.0],
               [0.0, 0.0, 0.0]]
predicted_points_prova_1 = [[-0.91, 0.20, 0.0],
                            [1.59, -17.63, 0.0],
                            [26.13, -17.92, 0.0],
                            [26.15, 0.87, 0.0],
                            [-4.63, 0.25, 0.0]]



true_points_girolab = [[0.0, 0.0, 0.0],
                       [0.0, -8.4, 0.0],
                       [0.0, -16.8, 0.0],
                       [8.4, -16.8, 0.0],
                       [16.8, -16.8, 0.0],
                       [25.2, -16.8, 0.0], 
                       [25.2, -8.4, 0.0],
                       [25.2, 0.0, 0.0]]
def plot_3d_points(points, true_points, title="3D Points", color='b', show=True):
    """
    Visualizza una lista di punti 3D con matplotlib.

    :param points: lista di punti 3D [(x, y, z), ...] o numpy array Nx3
    :param title: titolo del grafico
    :param color: colore dei punti
    :param show: se True, mostra subito il plot (può essere False per subplot)
    """
    points = 0.928*np.array(points)
    true_points = np.array(true_points)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_proj_type('ortho')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color, s=1)
    ax.scatter(true_points[:, 0], true_points[:, 1], true_points[:, 2], c='g', s=10)
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.axis('equal')
    ax.grid(True)
    ax.view_init(elev=90, azim=-90)  # vista inclinata per bellezza

    if show:
        plt.show()

def quaternion_mean_variance(quaternions):
    """
    Calcola la media e la varianza angolare di una popolazione di quaternioni unitari.
    
    :param quaternions: Nx4 array di quaternioni [x, y, z, w]
    :return: (media_quaternion [x, y, z, w], varianza_angolare)
    """
    Q = np.array(quaternions)
    
    for i in range(1, Q.shape[0]):
        if np.dot(Q[0], Q[i]) < 0:
            Q[i] = -Q[i]  # flip per evitare discontinuità

    A = np.zeros((4, 4))
    for q in Q:
        A += np.outer(q, q)

    A /= Q.shape[0]

    eigenvalues, eigenvectors = np.linalg.eigh(A)
    avg_quat = eigenvectors[:, np.argmax(eigenvalues)]
    
    avg_quat /= np.linalg.norm(avg_quat)

    var = 0
    for q in Q:
        dot = np.clip(np.dot(avg_quat, q), -1.0, 1.0)
        theta = 2 * np.arccos(np.abs(dot))  # distanza angolare
        var += theta**2
    var /= Q.shape[0]

    return avg_quat, var

def transform_pose(pose: Pose, R: np.ndarray, t: np.ndarray) -> Pose:
    """
    Applica una trasformazione omogenea (R, t) a una Pose e restituisce una nuova Pose trasformata.

    :param pose: Pose originale (geometry_msgs.msg.Pose)
    :param R: Matrice di rotazione 3x3 (numpy.ndarray)
    :param t: Vettore di traslazione 3x1 o 1x3 (numpy.ndarray)
    :return: Nuova Pose trasformata
    """
    # Posizione originale
    pos = np.array([pose.position.x, pose.position.y, pose.position.z])

    # Rotazione originale (quaternione) → matrice di rotazione
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_pose = tf_trans.quaternion_matrix(q)[:3, :3]

    # Composizione delle rotazioni: R_new = R * R_pose
    R_new = R @ R_pose

    # Composizione delle traslazioni: p_new = R * p + t
    pos_new = R @ pos + t

    # Nuova orientazione come quaternione
    q_new = tf_trans.quaternion_from_matrix(np.vstack((
        np.hstack((R_new, [[0], [0], [0]])),
        [0, 0, 0, 1]
    )))

    # Costruzione della nuova Pose
    new_pose = Pose()
    new_pose.position = Point(*pos_new)
    new_pose.orientation = Quaternion(*q_new)

    return new_pose

def parse_ranges_from_txt(file_path):
    ranges = []
    with open(file_path, 'r') as f:
        durations = []
        for line in f:
            if "Duration:" in line:
                parts = line.strip().split("Duration:")[1].split("/")
                start = float(parts[0].strip())
                durations.append(start)
                
        ranges = [(durations[i], durations[i+1]) for i in range(0, len(durations)-1, 2)]
    return ranges

def extract_poses_and_compute_stats(bag_path, txt_path, topic):
    ranges = parse_ranges_from_txt(txt_path)
    print(ranges)
    bag = rosbag.Bag(bag_path)

    pose_lists = [[] for _ in ranges]
    ori_lists = [[] for _ in ranges]
    bag_start_time = None
    all_poses = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        #if not isinstance(msg, PoseWithCovarianceStamped):
        #    print(type(msg))
        #    continue

        if bag_start_time is None:
            bag_start_time = t.to_sec()
        p = transform_pose(msg.pose.pose, R_transf, t_transf).position
        all_poses.append(np.array([p.x, p.y, p.z]))
        duration = t.to_sec() - bag_start_time
        #for idx, (start, end) in enumerate(ranges):
        #    if start <= duration <= end:
        #        pose_transformed = transform_pose(msg.pose.pose, R_transf, t_transf)
        #        pos = pose_transformed.position
        #        ori = pose_transformed.orientation
        #        pose_lists[idx].append((pos.x, pos.y, pos.z))
        #        ori_lists[idx].append((ori.x, ori.y, ori.z, ori.w))
        #        break
        if duration > 201.0:
            break

    bag.close()

    position_estimated = []
    orientation_estimed = []
    for idx, poses in enumerate(pose_lists):
        if not poses:
            print(f"[Range {idx+1}] Nessuna pose trovata.")
            continue

        poses_np = np.array(poses)
        p_mean = poses_np.mean(axis=0)
        o_mean, o_var = quaternion_mean_variance(ori_lists[idx])
        
        p_var = poses_np.var(axis=0)
        position_estimated.append(p_mean)
        orientation_estimed.append(o_mean)
        #print(f"\n[Range {idx+1}]")
        #print(f"  Pose Count: {len(poses)}")
        #print(f"  p media: {p_mean}")
        #print(f"  p sigma: {p_var}")
        #print(f"  o media: {o_mean}")
        #print(f"  o sigma: {o_var}")


    plot_3d_points(all_poses, true_points_girolab)


if __name__ == "__main__":
    bag_path = "logs/girolab_poses.bag"            
    txt_path = "girolab_punti.txt"         
    topic = "/ov_msckf/poseimu"    
    #true_points_prova_1 = np.array(true_points_prova_1)
    #predicted_points_prova_1 = np.array(predicted_points_prova_1)
    #print(f"1: {np.linalg.norm(true_points_prova_1[0]-predicted_points_prova_1[0])}")
    #print(f"2: {np.linalg.norm(true_points_prova_1[1]-predicted_points_prova_1[1])}")
    #print(f"3: {np.linalg.norm(true_points_prova_1[2]-predicted_points_prova_1[2])}")
    #print(f"4: {np.linalg.norm(true_points_prova_1[3]-predicted_points_prova_1[3])}")
    #print(f"5: {np.linalg.norm(true_points_prova_1[4]-predicted_points_prova_1[4])}")

    extract_poses_and_compute_stats(bag_path, txt_path, topic)
