# The MIT License (MIT)
#
# Copyright (c) 2026 RT Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import FileContent, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    urdf = FileContent(
        PathJoinSubstitution(
            [FindPackageShare('mujina_description'), 'urdf', 'mujina.urdf']
        )
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(urdf, value_type=str)}
        ],
        arguments=[urdf],
    )

    
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        )

    rviz_config_file = get_package_share_directory(
        'mujina_description') + '/launch/config.rviz'

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='Set file path to RViz config file.'
    )

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', LaunchConfiguration('rviz_config')]
                )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_config)
    ld.add_action(rsp)
    ld.add_action(jsp_gui)
    ld.add_action(rviz)

    return ld
