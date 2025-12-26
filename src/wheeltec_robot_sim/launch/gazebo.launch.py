import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'wheeltec_robot_sim'
    urdf_file = 'senior_4wd_bs_robot.urdf'
    
    # 1. 获取包的安装路径
    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)

    # 2. 定义世界文件路径 (请确认你的文件夹叫 world 还是 worlds)
    # 假设是 world
    world_file_path = os.path.join(pkg_share, 'world', 'labroom.world')
    
    install_dir = os.path.dirname(pkg_share)
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir
    else:
        model_path = install_dir

    print(f"DEBUG: Adding to GAZEBO_MODEL_PATH: {model_path}")
    print(f"DEBUG: Loading World: {world_file_path}")

    return LaunchDescription([  # <--- 注意这里有个 [ 开始
        # 1. 设置环境变量
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),

        # 2. 启动 Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file_path],
            output='screen'),

        # 3. 发布机器人的关节状态 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read(),
                'use_sim_time': True
            }]),

        # 4. 生成小车实体
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'wheeltec_bot', '-z', '0.05'],
            output='screen'),
            
         #5. 启动 Rviz2 (导航时不需要这里启动，所以注释掉)
         #Node(
          #   package='rviz2',
          #   executable='rviz2',
          #   name='rviz2',
          #   output='screen',
          #   arguments=['--ros-args', '-p', 'use_sim_time:=true']),
        
    ]) # <--- 【关键】这行绝对不能少！这就是报错的原因！
