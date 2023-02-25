# Perception Module

## Dependency
```
1. ROS
2. sudo apt-get install libpcap-dev
3. sudo apt-get install ros-kinetic-jsk-recognition-msgs
4. sudo add-apt-repository ppa:joseluisblancoc/mrpt-1.5
   sudo apt-get update
   sudo apt-get install libmrpt-dev mrpt-apps
```

## Build
```
cd ~/mars_ws
catkin_make
```

## Interface
- ### Input
   - /driver/livox/point_cloud sensor_msgs/PointCloud2 车顶激光雷达点云
   - /localization/estimation cyber_msgs::LocalizationEstimate 定位模块的融合输出结果
- ### Output
   - /perception/global_objects　cyber_msgs::ObjectArray 全局坐标系下动态障碍物
   - /perception/local_grid_map　sensor_msgs/CompressedImage 局部静态点云栅格图(roi区域为前100m，后20m, 左右25m； 分辨率为0.2m)

## Run
### Just Run
#### Run in Real Car
```
cd ~/mars_ws
./src/perception/scripts/run.sh
```

### Run & Visualize
#### Local Visualize
```
cd ~/mars_ws
./src/visualization/lidar_visualize/scripts/local_visualize.sh
```
#### Global Visualize
```
cd ~/mars_ws
./src/visualization/lidar_visualize/scripts/global_visualize.sh
```
