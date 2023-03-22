# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    # mmWave
    mmwave_config = DeclareLaunchArgument(
        'mmwave_config', default_value=os.path.join(
            get_package_share_directory(
                'nvblox_examples_bringup'), 'config', 'Mobile_Tracker_6843_ISK.cfg'
        )
    )

    ti_mmwave_bringup_launch_dir = os.path.join(get_package_share_directory("ti_mmwave_ros2_pkg"), "launch")

    radar_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ti_mmwave_bringup_launch_dir, "/single_6843.launch.py"]),
            launch_arguments={"cfg_file": LaunchConfiguration('mmwave_config')}.items())


    radar_base_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1',
            'base_link', 'ti_mmwave']
    )

    odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1',
            'odom', 'base_link']
    )

    # Nvblox
    nvblox_config = DeclareLaunchArgument(
        'nvblox_config', default_value=os.path.join(
            get_package_share_directory(
                'nvblox_examples_bringup'), 'config', 'nvblox_radar.yaml'
        )
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[LaunchConfiguration('nvblox_config')],
        remappings=[
            ("depth/camera_info", "/camera/depth/camera_info"),
            ("depth/image", "/camera/realsense_splitter_node/output/depth"),
            ("color/camera_info", "/camera/color/camera_info"),
            ("color/image", "/camera/color/image_raw"),
            ("radar", "/ti_mmwave/radar_scan_pcl")
        ]
    )

    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            nvblox_node
        ],
        output='screen'
    )

    # Nav2
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            get_package_share_directory(
                'nvblox_nav2'), 'params', 'carter_nav2.yaml'),
        description='Full path to param file to load')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Omniverse Isaac Sim) clock if true')

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'params_file': LaunchConfiguration('params_file'),
                          'autostart': 'True'}.items())


    # RVIZ
    rviz_config_path = os.path.join(get_package_share_directory(
        'nvblox_examples_bringup'), 'config', 'nvblox_radar_nav2.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    return LaunchDescription([
        nvblox_config,
        mmwave_config,
        nvblox_container,
        radar_node,
        radar_base_link_tf_node,
        odom_tf_node,
        params_file_arg,
        use_sim_time_arg,
        nav2_launch,
        rviz
    ])
