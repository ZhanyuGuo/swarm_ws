# swarm_ws

## 准备工作

1. `PX4-Autopilot`.
   
   1. [Build Intructions](https://docs.px4.io/main/en/sim_gazebo_classic/multi_vehicle_simulation_gazebo.html#multiple-vehicles-with-mavros-and-gazebo-classic).
      1. "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo" > "xxx/sitl_gazebo-classic".

   2. 官方源码，路径最好为`~/Workspace/PX4-Autopilot/`，否则修改`./swarm_ws/scripts/px4.sh`中的第2行.
   
   3. 将`./multi_uav_mavros_sitl.launch`复制到`./PX4-Autopilot/launch/`.
   
   4. 将`./assets/outdoor.world`复制到`./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds`.
   
   5. ~~将`./assets/outdoor.pcd`复制到`~/`~~.
   
   6. 将`./assets/models/`里的文件复制到`.gazebo/models/`中.

2. `ego-planner-swarm`.
   
   1. 如果使用ZhanyuGuo的fork的master分支，则已经修复该问题.

   2. 否则，编译问题看issue[#9](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/issues/9).

3. `swarm_ws`.
   
   1. 编译前首先`source ego-planner-swarm/devel/setup.bash`，以便之后source该空间附带上ego-swarm的环境.

## 运行

按以下顺序，在`./swarm/scripts/`路径下依次运行.

1. `px4.sh`.
   1. 第一次运行可能需要将launch文件中的无人机都注释掉，保证gazebo正常启动.

2. `swarm.sh`.

3. (optional) `rviz.sh`.

4. `init_multi_uav.sh`.
    
5. `task_allocation.sh`.