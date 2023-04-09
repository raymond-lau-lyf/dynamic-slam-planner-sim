# Land Shaker Simulator

## 依赖

```bash
sudo apt install xterm ros-melodic-moveit 
sudo apt install ros-melodic-velocity-controllers
sudo apt install ros-melodic-effort-controllers
sudo apt install ros-melodic-control*
sudo apt install libusb-dev
sudo apt install libgoogle-glog-dev libgflags-dev
sudo apt install ros-melodic-octomap-ros ros-melodic-ros-controllers 
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-robot-state-publisher
```

## 使用

1. launch

**husky**
```bash
roslaunch sim_launch mr2000_city.launch
```
**land_shaker**
```bash
roslaunch sim_launch ls_city.launch
```
**mr2000**
```bash
roslaunch sim_launch mr2000_city.launch
```
**mr1000**
```bash
roslaunch sim_launch mr1000_city.launch
```

```bash
# 以上车型包括仿真，slam和局部规划器的启动，全局规划器需要另外启动
```

**global_planner**
```bash
roslaunch far_planner far_planner.launch
```


2. 更换地图场景

- 通过修改launch文件中world_name参数，可以更换为local_planner/cmu/vehicle_simulator/world中的其他场景
- simple_city_erboutput场景的model_pose的z轴应设为5.3，其他地图初始位置0 0 0
- 目前fast_lio和lio_sam仅适合在simple_city_erboutput和forest等开阔地形使用，在tunnel和indoor等场景中漂移严重

3. 更换SLAM
- 首先修改launch文件中的is_fast_lio参数以切换slam，然后launch文件中有fast_lio和lio_sam的注释，切换即可

4. 更换局部规划器
- 首先通过launch中的is_cmu参数切换局部规划器，然后如果是撼地者模型的话，将/local_planner/move_base/mm_navigation/config/ls参数文件中的base_link替换为base_footprint


## 问题说明

1. imu噪声
- 已查明原因，gazebo的物理引擎在当前模型下计算精度不够，当提升迭代次数，并且降低最小步长时会有明显的改善，但是实时性会丧失，卡顿严重，因此除了重构模型外目前没有解决方案（其他可选的物理引擎兼容性很差，换上之后机械臂会崩）

2. movebase无法使用
- 目前参数在以movebase为局部规划器时效果很不好，调参调了好久但是没有解决，之后会继续调参解决
