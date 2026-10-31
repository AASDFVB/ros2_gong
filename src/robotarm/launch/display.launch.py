"""在 RViz 2 中显示 robotarm 模型（ROS 1 display.launch 的 ROS 2 版本）。

用法:
    ros2 launch robotarm display.launch.py
    ros2 launch robotarm display.launch.py gui:=false
    ros2 launch robotarm display.launch.py rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model = PathJoinSubstitution(
        [FindPackageShare('robotarm'), 'urdf', 'robotarm.urdf'])
    default_rviz = PathJoinSubstitution(
        [FindPackageShare('robotarm'), 'rviz', 'display.rviz'])

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rvizconfig = LaunchConfiguration('rvizconfig')

    # 纯 URDF 也交给 xacro 处理，以后想加 xacro 宏不用改 launch
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=default_model,
            description='URDF / xacro 文件的绝对路径'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='true 用 joint_state_publisher_gui（带滑条）'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz 2'),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz,
            description='RViz 2 配置文件'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                     'publish_frequency': 50.0}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
            # 默认 10 Hz，模型会一顿一顿；提到 50 Hz 才顺
            parameters=[{'rate': 50.0}],
        ),
        # gui:=false 时也必须有 /joint_states，否则 TF 树是空的
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
            parameters=[{'robot_description': robot_description,
                     'publish_frequency': 50.0}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rvizconfig],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
