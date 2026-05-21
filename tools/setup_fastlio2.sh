
PROGRAM1="source /home/agilex/catkin_ws/devel/setup.bash && roslaunch scout_bringup open_rslidar.launch"
PROGRAM2="sleep 1 && source /home/agilex/catkin_ws/devel/setup.bash && roslaunch rs_to_velodyne velodyne.launch"
PROGRAM3="sleep 1 && source /home/agilex/catkin_ws/devel/setup.bash && roslaunch imu_launch imu_msg.launch"
PROGRAM4="sleep 1 && source /home/agilex/catkin_ws/devel/setup.bash && source /home/agilex/codes/fastlio_ws/devel/setup.bash && roslaunch fast_lio mapping_velodyne.launch"
gnome-terminal \
  --window --title="fastlio2" --command="bash -c '$PROGRAM1; exec bash'" \
  --tab --title="to velodyne format" --command="bash -c '$PROGRAM2; exec bash'" \
  --tab --title="imu launch" --command="bash -c '$PROGRAM3; exec bash'" \
  --tab --title="fastlio2" --command="bash -c '$PROGRAM4; exec bash'" \



# 1、启动速腾雷达
# roslaunch scout_bringup open_rslidar.launch
# 2、速腾格式转velodyne格式
# roslaunch rs_to_velodyne velodyne.launch
# 3、启动imu
# roslaunch imu_launch imu_msg.launch
# 4、启动fastlio2
# cd ~/codes/fastlio_ws
# source devel/setup.bash
# roslaunch fast_lio mapping_velodyne.launch