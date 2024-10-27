# Costruzione iniziale dell'immagine
```bash
docker build -t coreresearch:latest .
```

# Avvio del container con il workspace montato
Configurare run.sh in modo da montare i volumi nei path corretti
```bash
chmod +x run.sh
./run.sh
```

# Nel container, se si vuole usare rviz
```bash
sudo apt install ros-noetic-rviz
sudo apt install ros-noetic-image-transport-plugins
source /opt/ros/noetic/setup.bash
rviz
```
Da rviz si riescono a vedere Camera e ground truth, ma non i dati IMU

# Se si vuole vedere lo stream di Camera e Imu senza rviz
Usare il nodo viewer.py del pacchetto image_viewer, nel container:
```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
rosrun image_viewer viewer.py
```