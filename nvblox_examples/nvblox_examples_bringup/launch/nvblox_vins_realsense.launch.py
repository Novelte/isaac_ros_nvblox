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


def generate_launch_description():

    # RealSense
    realsense_config_file_path = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'realsense_imu.yaml'
    )  

    realsense_node = ComposableNode(
        namespace="camera",
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[realsense_config_file_path],
    )

    realsense_splitter_node = ComposableNode(
        namespace="camera",
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
                    'input_qos': 'SENSOR_DATA',
                    'output_qos': 'SENSOR_DATA'
        }],
        remappings=[('input/infra_1', '/camera/infra1/image_rect_raw'),
                    ('input/infra_1_metadata', '/camera/infra1/metadata'),
                    ('input/infra_2', '/camera/infra2/image_rect_raw'),
                    ('input/infra_2_metadata', '/camera/infra2/metadata'),
                    ('input/depth', '/camera/depth/image_rect_raw'),
                    ('input/depth_metadata', '/camera/depth/metadata'),
                    ('input/pointcloud', '/camera/depth/color/points'),
                    ('input/pointcloud_metadata', '/camera/depth/metadata'),
        ]
    )

    realsense_container = ComposableNodeContainer(
        name='realsense_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_node,
            realsense_splitter_node
        ],
        output='screen'
    )

    # VINS
    vins_config_file = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'vins', 'realsense_stereo_imu_config.yaml'
    )  

    vins_pkg_path = os.path.join(
        get_package_share_directory('loop_fusion'),
        'support_files'
    )  


    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        output='screen',
        parameters=[{'config_file': vins_config_file,}]
    )

    loop_node = Node(
                package='loop_fusion',
                executable='loop_fusion_node',
                name='loop_fusion',
                output='screen',
                parameters=[{'config_file': vins_config_file,
                            'pkg_path': vins_pkg_path,
                            }]
            )

    tf_nodes = [Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments = ['0', '0', '0', '0.5', '-0.5', '0.5', '0.5', 'camera', 'camera_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments = ['0', '0.2', '0.35', '0.0', '0.0', '0.0', '1.0', 'map', 'world'],
        ),             
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments = ['0', '0', '0', '0.5', '-0.5', '0.5', '0.5', 'front_camera_rect', 'camera_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments = ['0', '0.2', '0.35', '0.0', '0.0', '0.0', '1.0', 'map', 'world'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments = ['0', '0', '0', '0.0', '0.0', '0.0', '1.0', 'odom', 'base_link'],
        ),   
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments = ['0', '0', '0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'front_camera_rect'],
        )]  

    # Nvblox
    nvblox_config = DeclareLaunchArgument(
        'nvblox_config', default_value=os.path.join(
            get_package_share_directory(
                'nvblox_examples_bringup'), 'config', 'nvblox.yaml'
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
            ("/pose", "/vins_estimator/odometry"),
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



    # RVIZ
    rviz_config_path = os.path.join(get_package_share_directory(
        'nvblox_examples_bringup'), 'config', 'nvblox_vslam_realsense.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    return LaunchDescription([
        nvblox_config,
        realsense_container,
        # vslam_container,
        vins_node,
        loop_node,
        nvblox_container,
        rviz
    ] + tf_nodes)
