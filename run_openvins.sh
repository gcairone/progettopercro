xhost +local:root
docker run \
-it \
--privileged \
-e DISPLAY=unix$DISPLAY \
--env="QT_X11_NO_MITSSH=1" \
-v /dev:/dev \
-v /sys:/sys \
-v /proc:/proc \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-v ~/Desktop/progettopercro/catkin_ws_ov:/root/catkin_ws_ov \
-v ~/Desktop/progettopercro/bagfiles:/root/bagfiles \
-v ~/Desktop/progettopercro/scripts:/root/scripts \
--net=host \
coreresearch:openvins bash
