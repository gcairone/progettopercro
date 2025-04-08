import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def extract_positions(yaml_data):
    positions = []
    i = 0
    for frame in yaml_data['frames']:
        T = frame['T']  
        positions.append(T)  
        i = i+1
    print(f"{i} poses")
    return positions

def plot_positions(positions, x_range=None, y_range=None, z_range=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x_vals = [pos[0] for pos in positions]
    y_vals = [pos[1] for pos in positions]
    z_vals = [pos[2] for pos in positions]
    
    ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    if x_range:
        ax.set_xlim(x_range)
    if y_range:
        ax.set_ylim(y_range)
    if z_range:
        ax.set_zlim(z_range)
    
    ax.set_title('Posizioni della Fotocamera')
    
    plt.show()

yaml_file = "poses_path_2.yaml"  
yaml_data = load_yaml(yaml_file)

positions = extract_positions(yaml_data)

x_range = (-5, 30)  
y_range = (-20, 15)    
z_range = (0, 35)    


plot_positions(positions, x_range=x_range, y_range=y_range, z_range=z_range)
