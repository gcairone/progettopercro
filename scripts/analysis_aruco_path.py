import numpy as np
import yaml
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar
"""
per la prova 2:
    - RMS senza scala 4.375 m
    - RMS con scala 0.831 -> 1.289 m
"""
"""
per la prova 3:
    - RMS senza scala 2.057
    - RMS con scala 0.930 -> 1.147 m


    - Errore finale -> 1.88 

"""
def optimize_uniform_scale(est_positions, est_times, gt_positions, gt_times):
    def error_for_scale(s):
        scaled_positions = est_positions * s
        errors = compute_position_error(gt_times, gt_positions, est_times, scaled_positions)
        return np.mean(errors**2)

    res = minimize_scalar(error_for_scale, bounds=(0.5, 1.0), method='bounded')

    return res.x, res.fun

def transform_trajectory(positions, alpha=0.0, scale=(1.0, 1.0, 1.0), translation=(0.0, 0.0, 0.0)):
    Rz = np.array([
        [np.cos(alpha), -np.sin(alpha), 0],
        [np.sin(alpha),  np.cos(alpha), 0],
        [0,              0,             1]
    ])

    if isinstance(scale, (float, int)):
        S = np.diag([scale, scale, scale])
    else:
        S = np.diag(scale)

    transformed = (Rz @ positions.T).T         # Rotazione
    transformed = (S @ transformed.T).T        # Scala
    transformed += np.array(translation)       # Traslazione

    return transformed



def read_estimated_trajectory(txt_path):
    timestamps = []
    positions = []
    with open(txt_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            timestamp = float(parts[0])
            position = [float(x) for x in parts[1:4]]
            timestamps.append(timestamp)
            positions.append(position)
    return np.array(timestamps), np.array(positions)

def read_ground_truth(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    timestamps = []
    positions = []
    for frame in data['frames']:
        T = frame['T']
        timestamp = frame['timestamp']
        timestamps.append(timestamp)
        positions.append([T[0][0], T[1][0], T[2][0]])
    return np.array(timestamps), np.array(positions)

def compute_position_error(gt_times, gt_positions, est_times, est_positions):
    errors = []
    for gt_time, gt_pos in zip(gt_times, gt_positions):
        idx = np.argmin(np.abs(est_times - gt_time))
        est_pos = est_positions[idx]
        error = np.linalg.norm(gt_pos - est_pos)
        errors.append(error)
    return np.array(errors)

def plot_trajectories(est_positions, gt_positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_proj_type('ortho')
    ax.plot(est_positions[:, 0], est_positions[:, 1], est_positions[:, 2], label='Stimata', color='blue', alpha=0.6)
    ax.scatter(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2], label='Ground Truth', color='green', s=5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.axis('equal')
    ax.legend()
    ax.view_init(elev=90, azim=-90)
    plt.title('Traiettorie')
    plt.show()

if __name__ == "__main__":
    i = 3
    alpha = np.deg2rad(90)  
    scale = (1.0, 1.0, 1.0)  
    translation = (0.0, 0.0, 0.0)  

    est_times, est_positions = read_estimated_trajectory(f"logs/pose{i}_log.txt")
    gt_times, gt_positions = read_ground_truth(f"poses_path_{i}.yaml")

    # --- Errore e visualizzazione iniziale ---
    errors_before = compute_position_error(gt_times, gt_positions, est_times, est_positions)
    print(f"[ORIGINALE] Errore medio di posizione: {np.mean(errors_before):.3f} m")
    print("Plot: traiettorie originali (senza trasformazioni)")
    plot_trajectories(est_positions, gt_positions)

    # --- PRIMA FASE: Rotazione + Traslazione ---
    alpha = np.deg2rad(165)                # rotazione di 30 gradi intorno a Z
    translation = (14.0, 12.0, 0.0)         # traslazione
    est_positions_rot_trans = transform_trajectory(est_positions, alpha=alpha, translation=translation)

    errors_rot_trans = compute_position_error(gt_times, gt_positions, est_times, est_positions_rot_trans)
    print(f"[ROT+TRASL] Errore medio di posizione: {np.sqrt(np.mean(errors_rot_trans**2)):.3f} m")
    print("Plot: traiettorie con rotazione + traslazione")
    plot_trajectories(est_positions_rot_trans, gt_positions)

    # --- SECONDA FASE: aggiunta della Scala ---
    scale = (0.93, 0.93, 0.93)               # ingrandimento lungo gli assi
    est_positions_full = transform_trajectory(est_positions, alpha=alpha, translation=translation, scale=scale)

    errors_full = compute_position_error(gt_times, gt_positions, est_times, est_positions_full)
    print(f"[ROT+TRASL+SCALA] Errore medio di posizione: {np.sqrt(np.mean(errors_full**2)):.3f} m")
    print("Plot: traiettorie con rotazione + traslazione + scala")
    plot_trajectories(est_positions_full, gt_positions)



    
