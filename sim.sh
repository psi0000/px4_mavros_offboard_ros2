docker run --gpus all \
--privileged \
-it \
--net=host \
--env=NVIDIA_VISIBLE_DEVICES=all \
--env=DISPLAY \
--env XDG_RUNTIME_DIR=/run/user/1000 \
--env=QT_X11_NO_MITSHM=1 \
-v /run/user/1000:/run/user/0 \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/Desktop/task_allocation:/root/task_allocation \
-v /dev:/dev \
--name=sim \
psi0921/multi_px4_ros2_setup:240805
