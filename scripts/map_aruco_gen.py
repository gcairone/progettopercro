import yaml
import os

def load_yaml_map(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
    else:
        data = {'aruco_map': {'markers': []}}
    return data

def save_yaml_map(file_path, data):
    with open(file_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

def add_markers(file_path):
    data = load_yaml_map(file_path)
    markers = []

    spacing = 1.2
    top_y = 0.0
    bottom_y = -14 * spacing
    left_x = 0.0
    right_x = 21 * spacing

    # Lato superiore (id 0–21)
    for i in range(22):
        markers.append({
            'id': i,
            'position': [i * spacing, top_y, 0.0],
            'orientation': [0.0, 0.0, 0.0]
        })

    # Lato destro (id 22–35)
    for i in range(1, 15):
        markers.append({
            'id': 21 + i,
            'position': [right_x, -i * spacing, 0.0],
            'orientation': [0.0, 0.0, 0.0]
        })

    # Lato inferiore (id 36–57)
    for i in range(1, 22):
        markers.append({
            'id': 35 + i,
            'position': [right_x - i * spacing, bottom_y, 0.0],
            'orientation': [0.0, 0.0, 0.0]
        })

    # Lato sinistro (id 58–69)
    for i in range(13):
        markers.append({
            'id': 57 + i,
            'position': [left_x, bottom_y + (i+1) * spacing, 0.0],
            'orientation': [0.0, 0.0, 0.0]
        })

    data['aruco_map']['markers'] = markers
    save_yaml_map(file_path, data)
    print("✅ Marker aggiunti con successo. Totale:", len(markers))

# Esegui
if __name__ == "__main__":
    yaml_file = "aruco_map_lab.yaml"
    add_markers(yaml_file)
