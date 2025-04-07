import math

def parse_positions_from_file(filename):
    positions = []
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    current = {}
    for line in lines:
        line = line.strip()
        if line.startswith("x:"):
            current["x"] = float(line.split(":")[1].strip())
        elif line.startswith("y:"):
            current["y"] = float(line.split(":")[1].strip())
        elif line.startswith("z:"):
            current["z"] = float(line.split(":")[1].strip())
            positions.append((current["x"], current["y"], current["z"]))
            current = {}

    return positions

def euclidean_distance(p1, p2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

def main():
    filename = "truthpose.txt"
    positions = parse_positions_from_file(filename)
    
    for i in range(len(positions) - 1):
        d = euclidean_distance(positions[i], positions[i + 1])
        print(f"Distanza tra punto {i} e {i+1}: {d:.4f}")

if __name__ == "__main__":
    main()
