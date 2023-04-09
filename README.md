# dyna_slam_ws


## overview
```text
src/
├── global_planner
│   └── far_planner
├── local_planner
│   ├── cmu # cmu仿真环境
│   ├── livox_laser_simulation # livox lidar gazebo models,待测试
│   └── move_base 
├── simlaunch # 存放整个工程的启动文件，目前暂时只维护mr1000_city.launch，其他后续会跟进
├── SLAM
│   ├── dynamic_filter # cia-ssd 动态点云过滤节点
│   ├── FAST_LIO # dynamic
│   ├── FAST-LIVO # developing，dynamic
│   ├── LIO-SAM
│   ├── rpg_vikit # build prerequests
│   ├── ws_livox # build prerequests
│   └── yolo_ros # developing，图像动态目标检测
└── vehicle # 机器人的urdf模型
    ├── husky
    ├── land_shaker
    ├── mr1000
    └── mr2000
```

## branches discription
devel分支为当前正在正在开发的分支，大概率不能完全用
main分支是主分支，主要工作应在main分支展开 
stable分支是稳定版，比main还要保守稳定，可以大步回退版本进行验证

## devel(developing)

rrr
roslaunch fast_lio mapping_velodyne_ls.launch
c & pp
roslaunch dynamic_filter dynamic_filter.launch