# launch lidar2grid_static

PROGRAM1="source ../../devel/setup.bash && rosrun perception lidar2grid_static.py"
gnome-terminal \
  --window --title="lidar2grid_static" --command="bash -c '$PROGRAM1; exec bash'"
