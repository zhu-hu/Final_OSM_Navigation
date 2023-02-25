## 用于D Star路径规划算法的建图程序
### 接口
- #### sub topic:
  - /grid_map: 激光栅格图
  - /move_base_simple/goal: rviz手动设定goal
  - /DSMap/path_task: statemachine设定goal
  - /current_pose: 当前车辆位姿

- #### pub topic:
  - /DSMap/global_map
  - /DSMap/local_map

- #### srv client:
  - /DSMap/task_service

### 主要函数
- #### InitMap:
  - free上一次的global map
  - 根据next goal与current pose确定global map的边长,初始化新的地图
  - TimerCallback函数独占一个线程,开始发布world->global map的tf
  - 将goal从world转到global map,第一次更新地图,通过srv传递给D*节点

- #### UpdateMap:
  - 若global map尚未初始化,return
  - 每接收到5帧local map,更新一次global map
  - 更新一次global map,用srv传给D*节点
