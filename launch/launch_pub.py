# my_robot_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
# arm_left_pub  arm_right_pub  camera_high_pub  camera_low_pub  camera_wrist_left_pub  camera_wrist_right_pub  pub  
def generate_launch_description():
    return LaunchDescription([
        # 启动四个图像节点
        Node(
            package='data_pub',
            executable='cam_high_pub',
            name='camera_high',
            output='screen',
            parameters=[
                {'camera_name': 'camera_high'},
                {'device_id': '/dev/CAM_HIGH'},
                {'topic_name': 'usb_cam_high/image_raw'}
            ]
        ),
        Node(
            package='data_pub',
            executable='cam_wrist_left_pub',
            name='camera_wrist_left',
            output='screen',
            parameters=[
                {'camera_name': 'camera_wrist_left'},
                {'device_id': '/dev/CAM_LEFT_WRIST'},
                {'topic_name': 'usb_cam_left_wrist/image_raw'}
            ]
        ),
        Node(
            package='data_pub',
            executable='cam_low_pub',
            name='camera_low',
            output='screen',
            parameters=[
                {'camera_name': 'camera_low'},
                {'device_id': '/dev/CAM_LOW'},
                {'topic_name': 'usb_cam_low/image_raw'}
            ]
        ),
        Node(
            package='data_pub',
            executable='cam_wrist_right_pub',
            name='camera_wrist_right',
            output='screen',
            parameters=[
                {'camera_name': 'camera_wrist_right'},
                {'device_id': '/dev/CAM_RIGHT_WRIST'},
                {'topic_name': 'usb_cam_right_wrist/image_raw'}
            ]
        ),
    ])
