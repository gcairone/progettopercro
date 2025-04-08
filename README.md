# Guida
Clonare questa repo, controllare che il Dockerfile sia preparato e nella directory corrente, costruire l'immagine
```bash
git clone https://github.com/gcairone/progettopercro/
docker build -t coreresearch:openvins .
```
Creare catkin_ws_ov (attenzione al path), clonare la repo
```bash
mkdir -p ~/progettopercro/catkin_ws_ov/src/
cd ~/progettopercro/catkin_ws_ov/src/
git clone https://github.com/rpng/open_vins/
```
Creare ~/progettopercro/bagfiles/ 
Scaricare quad-easy.bag e gt-nc-quad-easy (da tinyurl.com/newer-college-dataset-2023)
Ottenere V1_01_easy.bag (da http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag)
Creare ed entrare nel container con run_openvins.sh, compilare il progetto (se non trova catkin installare python3-catkin-tools)
```bash
chmod +x run_openvins.sh
./run_openvins.sh
catkin clean -y
catkin build -j1 -p1
```
In un primo terminale avvia il master
```bash
roscore
```

In un secondo terminale lanciare il pacchetto
```bash
source devel/setup.bash 
roslaunch ov_msckf subscribe.launch config:=euroc_mav dolivetraj:=true

```

In un terzo terminale lanciare rviz
```bash
cd ~/catkin_ws_ov/src/open_vins/launch
rviz -d display.rviz
```

In un quarto terminale lanciare lo stream di dati dal bag file
```bash
cd ~/bagfiles
rosbag play V1_01_easy.bag
```
Se invece si vuole lanciare il quad-easy dataset
```bash
cd ~/bagfiles
rosparam set image_transport compressed
rosrun image_transport republish compressed in:=/alphasense_driver_ros/cam0 raw out:=/cam0/image_raw & rosrun image_transport republish compressed in:=/alphasense_driver_ros/cam1 raw out:=/cam1/image_raw & rosbag play quad-easy.bag /alphasense_driver_ros/imu:=/imu0
```

Si dovrebbe iniziare a vedere tutto nell'interfaccia rviz


# Per usare con HW CoreResearch


Fare in terminali diversi
```bash
roscore
```
Lanciare openVINS con il file che si desidera:
- coreresearch_stereo.launch: Configurazione che usa la coppia di stereo davanti
- coreresearch_3mono.launch: Configurazione che usa le altre 3 camere come mono
- coreresearch_complete.launch: Configurazione che usa coppia stereo + 3 camere mono
```bash
source devel/setup.bash 
roslaunch ov_msckf coreresearch_stereo.launch

```
Lanciare il driver ros dell'HW alphasense
```bash
rosrun alphasense_driver_ros alphasense_driver_ros 
```
Per visualizzare l'odometria:
```bash
cd ~/catkin_ws_ov/src/open_vins/ov_msckf/launch
rviz -d display.rviz
```
Per far partire l'odometria bisogna dare un jerk minimo all'HW. 