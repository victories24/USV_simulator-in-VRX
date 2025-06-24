from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    # 获取配置文件名
    config_name = LaunchConfiguration('config').perform(context)
    
    # 构建完整路径
    config_path = os.path.join(
        get_package_share_directory('vrx_gazebo'),
        'config',
        f"{config_name}.rviz"
    )

    # 创建RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_path],
        output='screen'
    )

    return [rviz_node]

def generate_launch_description():
    # 定义可配置参数
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='path_static_tf',  # 默认配置文件名（不带后缀）
        description='RViz config filename (without .rviz extension)'
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=launch_setup)
    ])
