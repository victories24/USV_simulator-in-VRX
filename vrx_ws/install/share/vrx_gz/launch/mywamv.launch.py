from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    # 获取配置参数
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_urdf = LaunchConfiguration('urdf').perform(context)
    gz_paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)

    launch_processes = []

    # 初始化模型列表
    models = []
    if config_file and config_file != '':
        with open(config_file, 'r') as stream:
            models = Model.FromConfig(stream)
    else:
      m = Model('wamv', 'wam-v', [-532, 162, 0, 0, 0, 1])
      if robot_urdf and robot_urdf != '':
          m.set_urdf(robot_urdf)
      models.append(m)

    # gazebo仿真
    world_name, ext = os.path.splitext(world_name)
    launch_processes.extend(vrx_gz.launch.simulation(world_name, headless, 
                                                     gz_paused, extra_gz_args))
    world_name_base = os.path.basename(world_name)
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name_base, models, robot))

    # 启动桥接
    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_name_base, competition_mode))
    
    inverse_kinematics_node = Node(
        package='mywamv_control',
        executable='mywamv_inverse_kinematics.py',
        name='mywamv_inverse_kinematics',
        output='screen',
    )
    '''    
    # 添加mywamv_wayfinding的功能
    wayfinding_node = Node(
        package='mywamv_control',
        executable='mywamv_wayfinding.py',
        name='mywamv_wayfinding',
        output='screen'
    )
    '''

    path_follow_node = Node(
        package='mywamv_control',
        executable='mywamv_path_follow.py',
        name='mywamv_path_follow',
        output='screen'
    )

        
    # 如果不是无头模式，启动RViz
    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'path_static_tf.rviz')]
    )
    
    path_generator_node = Node(
        package='mywamv_control',
        #executable='dubins_path_generator.py',
        executable='figure_eight_generator.py',
        name='path_generator',
        output='screen'
    )
    
    # 将所有节点添加到Launch进程中
    launch_processes.extend([
        inverse_kinematics_node,
        #wayfinding_node,
        path_follow_node,
        rviz_node,
        path_generator_node
        
    ])
    
    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Name of world'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='YAML configuration file to spawn'),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Name of robot to spawn if specified. '
                        'This must match one of the robots in the config_file'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='True to run simulation headless (no GUI). '),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF file of the wam-v model. '),
        DeclareLaunchArgument(
            'paused',
            default_value='False',
            description='True to start the simulation paused. '),
        DeclareLaunchArgument(
            'competition_mode',
            default_value='False',
            description='True to disable debug topics. '),
        DeclareLaunchArgument(
            'extra_gz_args',
            default_value='',
            description='Additional arguments to be passed to gz sim. '),
        OpaqueFunction(function=launch),
    ])
