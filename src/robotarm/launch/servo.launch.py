"""RViz 滑条驱动真机：joint_state_publisher_gui → /joint_cmd → joint_servo → CAN

    ros2 launch robotarm servo.launch.py              # 只启动，不使能
    ros2 launch robotarm servo.launch.py send:=true   # 真正驱动
    ros2 launch robotarm servo.launch.py servo_script:=/path/to/joint_servo.py

tools/joint_servo.py 默认按工作空间相对路径查找，也可用环境变量
ROS2_GONG_TOOLS 指向 tools/ 目录。
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _find_servo_script():
    """找到工作空间里的 tools/joint_servo.py，不写死绝对路径。

    优先级：环境变量 ROS2_GONG_TOOLS 指定的 tools 目录 > 本文件的相对位置
    （--symlink-install 时 __file__ 指回 src/，工作空间根目录是上 4 层）
    > 当前目录 > ~/Desktop/ros2_gong。
    """
    tools_env = os.environ.get('ROS2_GONG_TOOLS')
    if tools_env:
        return str(Path(tools_env) / 'joint_servo.py')

    here = Path(__file__).resolve()
    candidates = [
        here.parents[3] / 'tools' / 'joint_servo.py',
        Path.cwd() / 'tools' / 'joint_servo.py',
        Path.home() / 'Desktop' / 'ros2_gong' / 'tools' / 'joint_servo.py',
    ]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return str(candidates[0])


SERVO = _find_servo_script()


def generate_launch_description():
    share = FindPackageShare('robotarm')
    urdf = PathJoinSubstitution([share, 'urdf', 'robotarm.urdf'])
    rviz_cfg = PathJoinSubstitution([share, 'rviz', 'display.rviz'])
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 50.0}],
    )

    # 滑条：发布到 /joint_cmd，不要占用 /joint_states
    gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        remappings=[('joint_states', 'joint_cmd')],
        parameters=[{'robot_description': robot_description, 'rate': 50.0}],
    )

    servo_script = LaunchConfiguration('servo_script')

    servo = ExecuteProcess(
        cmd=['python3', '-u', servo_script,
             '--send'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('send')),
    )

    servo_dry = ExecuteProcess(
        cmd=['python3', '-u', servo_script],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('send')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('send', default_value='false',
                              description='true 才真正使能驱动真机'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('servo_script', default_value=SERVO,
                              description='joint_servo.py 的绝对路径'),
        rsp, gui, servo, servo_dry, rviz_node,
    ])
