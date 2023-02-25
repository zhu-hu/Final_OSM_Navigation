# 打包函数成.so库示例

## 如何生成 .so 库，以 localization 节点，打包GPS-UTM转换函数， src/common/map_frame.cpp 为例

#### 参考 localization/localization/CMakeLists.txt, 添加
```bash
add_library(map_frame_lib src/common/map_frame.cpp)
```
#### 其中 map_frame_lib 是生成的库文件的名称，src/common/map_frame.cpp 是包含函数定义的源代码
#### 然后执行 catkin_make, 在 devel/lib/ 文件夹下会生成相应的.so库文件

## 如何使用 .so 库，还是以 localization 节点为例
#### 在 localization/localization/ 下创建 lib/ 文件夹，把需要用的.so库放进去
#### 参考 localization/localization/CMakeLists.txt, 添加 lib/ 为链接库路径
```bash
link_directories(lib/)
```
#### 以 localization 生成 localization_test 可执行程序为例， 在 target_link_libraries 里添加链接到 libmap_frame_lib.so
```bash
add_executable(localization_test src/test.cpp)
add_dependencies(localization_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_test ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} libmap_frame_lib.so)
```
#### test.cpp 中 include 相应的头文件，在示例中是 map_frame.h
#### 然后正常 catkin_make 即可