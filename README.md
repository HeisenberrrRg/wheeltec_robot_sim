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
ros2 launch wheeltec_robot_sim gazebo.launch.py
```

**启动建图:**
```bash
ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=$(ros2 pkg prefix --share wheeltec_robot_sim)/params/mapper_params_sim.yaml \
use_sim_time:=True
```

**启动键盘控制小车移动建图:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**保存地图***
```bash
ros2 run nav2_map_server map_saver_cli -f ./src/wheeltec_robot_sim/maps/my_map
```
3.Nav2导航
**启动导航:**
```bash
ros2 launch wheeltec_robot_sim nav2.launch.py 
```
