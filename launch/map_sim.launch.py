from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        
        # 1. OUR NEW BRIDGE (Sim Time is hardcoded inside)
        Node(
            package='midcone_rtabmap',  # Or whatever package your script is in
            executable='odom_to_tf',    # Your python script entry point
            output='screen',
            # CRITICAL FIX: This forces the Node to initialize with the ROS Clock (Sim Time)
            parameters=[{'use_sim_time': True}]
        ),

        # 2. STATIC TF (Camera) - FORCED SIM TIME
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{'use_sim_time': True}],
            arguments = ['0.1', '0', '0', '-1.57', '0', '-1.57', 'base_link', 'camera_link']
        ),

        # 3. RTAB-MAP (Standard Sim Config)
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'use_sim_time': True,
                'approx_sync': True,
                'wait_imu_to_init': False,
                'queue_size': 20,
                'qos_image': 2,
                'qos_camera_info': 2,
            }],
            remappings=[
                ('rgb/image', '/camera'),
                ('rgb/camera_info', '/camera_info'),
                ('depth/image', '/depth_camera'),
                ('odom', '/mavros/local_position/odom')
            ],
            arguments=['-d'] # Delete old map
        ),

        # 4. VISUALIZER
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('rgb/image', '/camera'),
                ('rgb/camera_info', '/camera_info'),
                ('depth/image', '/depth_camera'),
                ('odom', '/mavros/local_position/odom')
            ]
        ),
    ])