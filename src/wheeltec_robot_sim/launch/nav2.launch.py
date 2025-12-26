import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'wheeltec_robot_sim'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 1. 地图路径
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    
    # 2. 参数文件路径
    params_file_path = os.path.join(pkg_share, 'params', 'nav2_params_sim.yaml')

    # 3. RViz 配置文件路径 (新增部分)
    # 确保你的 CMakeLists.txt 已经将 rvizconfig 文件夹安装到了 share 目录
    rviz_config_file_path = os.path.join(pkg_share, 'rvizconfig', 'wheeltec.rviz')
    
    # Nav2 标准启动包路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # 1. 启动 Gazebo 仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
            )
        ),

        # 2. 启动 Nav2 导航
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file_path,
                'params_file': params_file_path,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_rviz': 'False' # 禁止 Nav2 启动默认的 RViz，我们要用下面自定义的
            }.items()
        ),

        # 3. 启动 RViz (加载指定配置)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # 使用 -d 加载配置文件，并保持 use_sim_time 为 true
            arguments=['-d', rviz_config_file_path, '--ros-args', '-p', 'use_sim_time:=true']
        ),
    ])
