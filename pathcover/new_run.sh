docker run -it --rm \
  --name pathcover-drone \
  --gpus all \
  --network host \
  --ipc host \
  --user 0:0 \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/root/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
  -v "$PWD:/home/PathCover/PathCover_ws" \
  -w /home/PathCover/PathCover_ws \
  ros-noetic-pathcover:latest \
  bash ./launch_sim_drone.sh
