#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # parameter files
    ros_param_file = os.path.join(
            get_package_share_directory('yolo_object_detection_ros2'),
            'config',
            'cam_params.yaml')
            
    yolo_param_file = os.path.join(
            get_package_share_directory('yolo_object_detection_ros2'),
            'config',
            'yolov3-tiny-custom.yaml')

    # start front_cam
    front_cam_node=Node(
            package='usb_cam',
            namespace='FV1',
            name='usb_cam',
            executable='usb_cam_node_exe',
            parameters = [ros_param_file],
            remappings=[('image_raw', 'usb_cam/image_raw')],
            output='screen')

    # start darknet and ros wrapper
    darknet_node=Node(
            package='yolo_object_detection_ros2',
            namespace='FV1',
            name='yolo_object_detection_node',
            executable='yolo_object_detection_ros2',
            output='screen',
            parameters = [yolo_param_file])

    ld = LaunchDescription()

    ld.add_action(front_cam_node)
    ld.add_action(darknet_node)

    return ld
