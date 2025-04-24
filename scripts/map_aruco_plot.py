import yaml
import matplotlib.pyplot as plt

def load_aruco_map(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['aruco_map']['markers']

def plot_aruco_map(markers):
    plt.figure(figsize=(6, 6))
    
    for marker in markers:
        x, y, _ = marker['position']
        marker_id = marker['id']
        plt.scatter(x, y, s=10, marker='s', label=f"ID {marker_id}")
        plt.text(x + 0.05, y + 0.05, f"ID {marker_id}", fontsize=10)
    
    plt.title("Mappa ArUco")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    yaml_file = "aruco_map_lab.yaml"  
    markers = load_aruco_map(yaml_file)
    plot_aruco_map(markers)
