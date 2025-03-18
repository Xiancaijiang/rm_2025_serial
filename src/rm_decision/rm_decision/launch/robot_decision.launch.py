import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('rm_decision')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    init_x = LaunchConfiguration('init_x', default='0.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_z = LaunchConfiguration('init_z', default='0.0')
    init_yaw = LaunchConfiguration('init_yaw', default='0.0')
    target_distance = LaunchConfiguration('target_distance', default='5.0')
    
    # 声明参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true')
    
    declare_init_x_cmd = DeclareLaunchArgument(
        'init_x',
        default_value='0.0',
        description='Initial X position')
    
    declare_init_y_cmd = DeclareLaunchArgument(
        'init_y',
        default_value='0.0',
        description='Initial Y position')
    
    declare_init_z_cmd = DeclareLaunchArgument(
        'init_z',
        default_value='0.0',
        description='Initial Z position')
    
    declare_init_yaw_cmd = DeclareLaunchArgument(
        'init_yaw',
        default_value='0.0',
        description='Initial Yaw orientation')
    
    declare_target_distance_cmd = DeclareLaunchArgument(
        'target_distance',
        default_value='5.0',
        description='Target distance to move forward')
    
    # 启动决策节点
    robot_decision_node = Node(
        package='rm_decision',
        executable='robot_decision_node',
        name='robot_decision',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'init_x': init_x,
            'init_y': init_y,
            'init_z': init_z,
            'init_yaw': init_yaw,
            'target_distance': target_distance,
            'position_tolerance': 0.1,
            'orientation_tolerance': 0.1
        }]
    )
    
    # 创建并返回启动描述
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_init_x_cmd,
        declare_init_y_cmd,
        declare_init_z_cmd,
        declare_init_yaw_cmd,
        declare_target_distance_cmd,
        robot_decision_node
    ]) 