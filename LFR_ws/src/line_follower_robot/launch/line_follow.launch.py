# Copyright 2025 Bishal Dutta (Bishalduttaoffcial@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue # Correct import
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_line_follower_robot = FindPackageShare('line_follower_robot')
    package_name = 'line_follower_robot'

    # Path to the robot's URDF file
    urdf_file = PathJoinSubstitution([pkg_line_follower_robot, 'urdf', 'robot.urdf'])

    # === MODIFIED: Path to the NEW warehouse world file ===
    world_file = PathJoinSubstitution([pkg_line_follower_robot, 'worlds', 'world.sdf'])

    # Standard ROS 2 launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Node to publish robot's state (transforms) using xacro command
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Wrap the command in ParameterValue as required
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
        }]
    )

    # Launch Gazebo
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        # Load the new world file
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # Topic to get URDF from
            '-name', 'lfr_bot',           # Name of the robot in Gazebo
            # === MODIFIED: New spawn position for the warehouse world ===
            '-x', '-12.0', # Start X position (near the beginning of segment_1)
            '-y', '-8.0',  # Start Y position (on the line)
            '-z', '0.1'    # Spawn slightly above ground
            # No rotation needed, starts facing along the line
        ],
        output='screen'
    )

    # Launch the ROS-Gazebo bridge
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # (Optional) Joint state publisher for non-fixed joints
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch our line follower Python node
    follower_node = Node(
        package='line_follower_robot',
        executable='follower_node',
        name='follower_node',
        output='screen'
    )

    # Launch rqt_image_view to see the camera feed
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        arguments=['/camera/image_raw'] # Tell rqt_image_view which topic to view
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,

        # Launch simulation with the new world
        gazebo_sim,

        # Publish robot model
        robot_state_publisher_node,

        # Spawn robot at the new start pose
        spawn_robot,

        # Bridge ROS and Gazebo topics
        gz_bridge_node,

        # Optional nodes
        joint_state_publisher_node,

        # Run our controller
        follower_node,

        # Add the camera view window
        image_view_node,
    ])


