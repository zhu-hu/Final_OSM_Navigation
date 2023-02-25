## D*规划算法
### 接口
- #### sub topic:
  - /current_pose: 当前车辆位姿
- #### pub topic:
  - /DStar/path_rviz: D*规划路径结果在rviz中显示
  - /DStar/local_target: D*规划路径上选取的局部目标点,作为RRT的终点发布给RRT
- #### srv server:
  - /DSMap/task_service: DSMap节点将目标点和地图信息通过srv请求D*路径规划

### 核心函数
- Replan()
  - 两种Replan,一种可设置时长,初始化用.一种默认时长,更新路径用.
- InitMap
  - 根据current pose, goal, map_update初始化地图,并生成一条路径.注意这里的Replan规划函数的时间参数较长,因为是第一次生成路径.
- UpdateMap
  - 根据map_update更新地图,更新路径.Replan规划函数的时间较短,因为只是对路径进行更新,不用重头计算.
- TaskCallback
  - 每次收到DSMap的srv请求,调用一次.
    - 若Init==1,重新创建任务.free(dstar)-->new dstar()-->InitMap()-->Replan().
    - 若Init!=1,UpdateMap()-->Replan().