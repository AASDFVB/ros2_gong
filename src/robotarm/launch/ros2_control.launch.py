"""ros2_control 链路启动（真机：达妙 DM4310 / can0）。

    robot_state_publisher + ros2_control_node + joint_state_broadcaster
    + joint_trajectory_controller + rviz2

    硬件由 URDF 的 <ros2_control> 段声明（robotarm_hardware/DmSystemHardware），
    控制器参数来自 config/ros2_controllers.yaml，update_rate 也只在那里改。

用法:
    ros2 launch robotarm ros2_control.launch.py
    ros2 launch robotarm ros2_control.launch.py rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
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
        # 硬件 configure 失败会让它 abort；不加 on_exit 的话 launch 会一直挂着，
        # systemd 看着是 active 实际什么都没跑。这里让它死就整个 launch 退出。
        on_exit=Shutdown(),
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
