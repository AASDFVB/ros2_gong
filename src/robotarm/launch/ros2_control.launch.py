"""ros2_control 链路启动（第一步：mock 硬件，不接 CAN）。

    robot_state_publisher + ros2_control_node + joint_state_broadcaster
    + joint_trajectory_controller + rviz2

用法:
    ros2 launch robotarm ros2_control.launch.py
    ros2 launch robotarm ros2_control.launch.py rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robotarm')
    urdf = PathJoinSubstitution([share, 'urdf', 'robotarm.urdf'])
    ctrl_yaml = PathJoinSubstitution([share, 'config', 'ros2_controllers.yaml'])
    rviz_cfg = PathJoinSubstitution([share, 'rviz', 'display.rviz'])

    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    rviz = LaunchConfiguration('rviz')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, ctrl_yaml],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        rsp,
        control_node,
        # 先起 jsb，成功后再起 arm_controller
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner,
                          on_exit=[arm_spawner]),
        ),
        jsb_spawner,
        rviz_node,
    ])
