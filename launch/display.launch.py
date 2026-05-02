import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your packages
    pkg_description = get_package_share_directory('a2_description')
    
    # Paths to files
    default_urdf_path = os.path.join(pkg_description, 'urdf', 'a2.urdf')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz', 'default.rviz')

    # Launch Arguments
    urdf_model_arg = DeclareLaunchArgument(
        'urdf_model', 
        default_value=default_urdf_path, 
        description='Absolute path to robot urdf file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', 
        default_value=default_rviz_config_path, 
        description='Absolute path to rviz config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (MuJoCo) clock if true'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', LaunchConfiguration('urdf_model')]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    sim_clock_node = Node(
        package='a2_sim_utils',
        executable='sim_clock',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    a2_bridge_node = Node(
        package='a2_sim_utils',
        executable='a2_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        urdf_model_arg,
        rviz_config_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        sim_clock_node,
        a2_bridge_node,
        rviz2_node
    ])