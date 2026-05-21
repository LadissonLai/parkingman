我正在采集一个根据人类指令自主寻找停车位的数据集，请你在写一个采集carla仿真器数据集的ros脚本，脚本的核心功能是订阅图像、里程计、检测到的车位话题以2Hz的频率保存数据，直到用户按回车键停止保存数据。
具体话题名称如下：
1、前后左右相机图片的话题如下:
/carla/ego_vehicle/rgb_front/image
/carla/ego_vehicle/rgb_rear/image
/carla/ego_vehicle/rgb_left/image
/carla/ego_vehicle/rgb_right/image
2、里程计话题为：/carla/ego_vehicle/odometry，类型为标准的ROS Odometry。
3、检测到的所有车位话题为：/parking_map/confirmed_spaces_in_world，类型为ParkingSpaceArray，自车的frame_id为ego_vehicle，这个话题订阅的消息是在全局坐标系下面，保存的时候，需要转换到自车坐标系ego_vehicle，通过tf库进行转换。

程序启动的时候，首先从txt配置文件中读取人类指令，然后输入1键开始记录数据，你需要记录起点的位姿start_frame_id，程序以2Hz的频率保存数据，只保存关键帧的数据，关键帧的定义为前后移动超过0.1米则认为是关键帧。具体的保存路径和数据要求如下：
1、对于图像数据，你需要新建一个文件夹叫做images，下面4个子文件夹，分别是front、rear、left、right，图片的编号从000001开始，每记录一次编号+1，例如保存为000001.png,000002.png,依次类推。
2、对于里程计数据，坐标系默认为map，你需要将其转化到起点坐标系下面，也就是start_frame_id下面，同样以2hz的频率保存关键帧，将其保存为odom.csv文件，文件的每一行，保存x,y,yaw,velocity_x, velocity_y，其中yaw转换为角度（范围为-180度到180度）。
3、对于检测到的车位数据，同样你需要键所有车位从全局map转换到自车坐标系ego_vehicle下面，以2hz的频率保存关键帧，将其保存为parking_slots.jsonl文件，每一行是一个json，记录所有车位的信息，例如{[{id=1,x=3.2,y=4.2,yaw=90,width=4.8,height=3.2},{id=1,x=3.2,y=4.2,yaw=90,width=4.8,height=3.2},...]},注意这里的yaw单位转成角度，width和height也需要变，width始终与朝向垂直，height沿着朝向。如何此时的关键帧保存时，没有车位消息，则保存为空的{},占据一行jsonl的位置。
4、当键盘输入回车时，则停止关键帧数据保存，然后要求用户从键盘输入最终泊入的车位id，车位id必须是在/parking_map/confirmed_spaces_in_world这个话题里面的，最后记录该决策id，将其保存到decision.txt中，第一行保存人类指令，第二行保存选择的空闲车位id。

最后，将所有数据保存到硬盘，程序退出。这样保存的每一次数据是一条轨迹是一个独立的文件夹，这个文件夹根据当前年-月-日-时-分来命名。

OK，以上就是全部要求，请按照我的要求写一个python脚本。如果你有什么不懂的或者信息缺失，你先提出来，我给你补充信息，你先给出代码方案，等我过目了，你再给出代码。