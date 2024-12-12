xhost +local:root
docker run \
-it \
--privileged \
-e DISPLAY=unix$DISPLAY \
--env="QT_X11_NO_MITSSH=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-v ~/progettopercro/catkin_ws_ov:/root/catkin_ws_ov \
-v ~/progettopercro/bagfiles:/root/bagfiles \
--net=host \
coreresearch:openvins bash
