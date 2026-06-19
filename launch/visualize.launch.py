import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('a2_description')

    default_urdf_path = os.path.join(pkg_description, 'urdf', 'a2.urdf')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz', 'default.rviz')

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

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 alongside the robot description'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use the joint_state_publisher slider GUI (needs a display); '
                    'set false for the headless joint_state_publisher'
    )

    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf_model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    # GUI slider window for interactive joint control (needs a display)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui')),
        parameters=[{'use_sim_time': False}]
    )

    # Headless joint state publisher — publishes /joint_states without a GUI
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
        parameters=[{'use_sim_time': False}]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        urdf_model_arg,
        rviz_config_arg,
        launch_rviz_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz2_node,
    ])
