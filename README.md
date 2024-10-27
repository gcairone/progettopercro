# Costruzione iniziale dell'immagine
```bash
docker build -t coreresarch:latest .

# Avvio del container con il workspace montato
```bash
chmod +x run.sh
./run.sh

# Nel container, se si vuole usare rviz
```bash
sudo apt install ros-noetic-rviz
sudo apt install ros-noetic-image-transport-plugins
source /opt/ros/noetic/setup.bash
rviz