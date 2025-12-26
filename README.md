# wheeltec_robot_sim
A nav2-gazebo sim project

1.克隆项目，进入工作空间，colcon build
```bash
git clone https://github.com/HeisenberrrRg/wheeltec_robot_sim.git
cd wheeltec_robot_sim
colcon build
. install/setup.bash 
```
2.建图

**打开gazebo仿真:**
```bash
#source环境
. install/setup.bash

#启动gazebo
ros2 launch wheeltec_robot_sim gazebo.launch.py
```

**启动建图:**
```bash
#打开新终端source环境
. install/setup.bash

#启动建图
ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=$(ros2 pkg prefix --share wheeltec_robot_sim)/params/mapper_params_sim.yaml \
use_sim_time:=True

#开新终端启动RVIZ（在此目录下找到wheeltec.rviz配置文件/wheeltec_robot_sim/src/wheeltec_robot_sim/rvizconfig）
rviz2 
```

**启动键盘控制小车移动建图:**
```bash
#打开新终端启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**保存地图***
```bash
#打开新终端保存地图（保存完毕可以关闭键盘控制和SLAM建图的终端）
ros2 run nav2_map_server map_saver_cli -f ./src/wheeltec_robot_sim/maps/my_map

#重新colcon build（保存成功后重新colcon build才能在导航中加载地图）
colcon build
#source环境
. install/setup.bash
```
3.Nav2导航
**启动导航:**
```bash
ros2 launch wheeltec_robot_sim nav2.launch.py 
```
