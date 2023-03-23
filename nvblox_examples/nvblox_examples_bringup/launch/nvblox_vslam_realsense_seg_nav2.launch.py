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

    # RealSense
    realsense_config_file_path = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'realsense.yaml'
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

    # VSLAM
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/vslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_localization_n_mapping': True,
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': True,
                    'image_qos': 'SENSOR_DATA'
        }],
        remappings=[('stereo_camera/left/image', '/camera/realsense_splitter_node/output/infra_1'),
                    ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                    ('stereo_camera/right/image', '/camera/realsense_splitter_node/output/infra_2'),
                    ('stereo_camera/right/camera_info', '/camera/infra2/camera_info')]
    )

    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    base_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1',
            'base_link', 'camera_link']
    )

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
            ("semantic/image", "/mask_out"),
            ("semantic/camera_info", "/camera/color/camera_info")
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

    # Yolov7 Seg
    yolov7_bringup_dir = get_package_share_directory('isaac_ros_yolov7')

    onnx_file_arg = DeclareLaunchArgument(
        'onnx_file',
        default_value=os.path.join(yolov7_bringup_dir, 'model', 'yolov7-seg.onnx'),
        description='Full path to the onnx file to use')
    engine_file_arg = DeclareLaunchArgument(
        'engine_file',
        default_value=os.path.join(yolov7_bringup_dir, 'model', 'yolov7-seg.engine'),
        description='Full path to the tensorrt engine file to use / build')


    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_encoders',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        parameters=[{
            'network_image_width': 640,
            'network_image_height': 640,
            'tensor_name': 'input_tensor',
            'image_mean': [0.0, 0.0, 0.0],
            'image_stddev': [1.0, 1.0, 1.0]
        }],
        remappings=[('encoded_tensor', 'tensor_pub'),
                    ('image','/camera/color/image_raw')]
    )

    tensorrt_node = ComposableNode(
        name='tensor_rt_node',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': LaunchConfiguration('onnx_file'),
            'engine_file_path': LaunchConfiguration('engine_file'),
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],
            'output_tensor_names': ["output", "onnx::Slice_528", "onnx::Slice_638", "onnx::Slice_748", "513"],
            'output_binding_names': ["output", "onnx::Slice_528", "onnx::Slice_638", "onnx::Slice_748", "513"],
            'verbose': False,
            'force_engine_update': False
        }])

    yolov7_segment_decoder_node = ComposableNode(
        name='yolov7_segment_decoder_node',
        package='isaac_ros_yolov7',
        plugin='isaac_ros::yolov7::YoloV7SegmentDecoderNode',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'conf_thres': 0.15,
        }], 
        remappings=[('camera_info', '/camera/color/camera_info')])

    seg_container = ComposableNodeContainer(
        name='decoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            encoder_node, 
            tensorrt_node, 
            yolov7_segment_decoder_node
            ],
        output='screen'
    )

    # Nav2
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            get_package_share_directory(
                'nvblox_examples_bringup'), 'config', 'nav2','seg_nav2.yaml'),
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
        'nvblox_examples_bringup'), 'config', 'nvblox_vslam_realsense_seg_nav2.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    return LaunchDescription([
        nvblox_config,
        realsense_container,
        vslam_container,
        nvblox_container,
        base_link_tf_node,
        onnx_file_arg,
        engine_file_arg,
        seg_container,
        params_file_arg,
        use_sim_time_arg,
        nav2_launch,
        rviz
    ])
