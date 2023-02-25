## lidar_localization 定位模块
- ### 输入
  - `/Inertial/gps/fix` : `sensor_msgs::NavSatFix` 从Inertial+得到的GPS经纬度，时间戳是系统时间
  - `/Inertial/imu/data` : `sensor_msgs::Imu` 从Inertial+得到的航向和角速度，时间戳是系统时间
  - `/Inertial/gps/vel` : `geometry_msgs::TwistWithCovarianceStamped` 从Inertial+得到的航向和角速度，时间戳是系统时间
  - `/driver/pandar/point_cloud` : `sensor_msgs::PointCloud2` 从Pandar40p得到的点云数据


- ### 输出
  - /slam/gps/fix sensor_msgs::NavSatFix 从SLAM输出的经纬度
  - /slam/gps/heading sensor_msgs::Imu 从SLAM输出的航向
  
- ### 开启
  -  城区停车场
  
        `roslaunch lidar_localization parkinglot.launch`

  -  高架底部
  
        `roslaunch lidar_localization highway.launch`

- ### 配置参数
    
  -  关闭rviz 
  
        launch文件中 `<arg name="rviz" default="true" />` 改为`false`

地图刷新逻辑：
主要承担的函数为grid_map_2d的UpdateGridMap函数，根据中心位置判断更新为mrpt需要的形式并存储在GridMapMatching类型的matcher_中，显示通过grid_map_localization的GetRosGridMap函数将地图格式转化在主程序中输出



main

grid_map_localization
	地图刷新：UpdatingMap 核心地图更新代码在grid_map_2d的UpdateGridMap

pose_initialization
	用于ui的初始化的地图加载等任务

mrpt_icp_matching

pcl2mrpt



