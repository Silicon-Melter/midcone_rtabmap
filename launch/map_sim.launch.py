import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- CONFIGURATION ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    
    # 1. RTAB-Map Parameters
    parameters=[{
          'frame_id': 'base_link',          # Robot body frame
          'odom_frame_id': 'odom',          # Odometry frame
          'subscribe_depth': True,
          'subscribe_odom_info': False,
          'approx_sync': True,
          'wait_imu_to_init': False,        # Don't wait for IMU
          'use_sim_time': use_sim_time,
          'database_path': database_path,
          
          # Sync Tolerance (The "Loose Zipper")
          'queue_size': 50,
          'approx_sync_max_interval': 0.1,
          
          # QoS Compatibility
          'qos_image': 2,
          'qos_camera_info': 2,
    }]

    # 2. Topic Remappings
    # Note: We do NOT use compressed transport here to keep it simple.
    # If using republisher, these should match the republisher output.
    remappings=[
          ('imu', '/mavros/imu/data'),
          ('rgb/image', '/camera/fixed'),          # <--- CHANGED
          ('rgb/camera_info', '/camera_info/fixed'), # <--- CHANGED 
          ('depth/image', '/depth_camera/fixed'),  # <--- CHANGED
          ('odom', '/mavros/local_position/odom')
    ]

    return LaunchDescription([
        
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),

        # --- BRIDGE 1: Odom Topic -> TF Transform ---
        # This creates the 'odom -> base_link' connection
        Node(
            package='midcone_rtabmap',      # <--- Your package name
            executable='odom_to_tf',        # <--- The name you set in setup.py
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # Remap the generic 'odom' to your specific mavros topic
            remappings=[('odom', '/mavros/local_position/odom')] 
        ),

        # --- BRIDGE 2: Base -> Camera Transform ---
        # This creates the 'base_link -> camera_link' connection
        # Adjust the numbers (x y z yaw pitch roll) if your camera is not at the center
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1', '--y', '0', '--z', '0', 
                         '--yaw', '-1.5707', '--pitch', '0', '--roll', '-1.5707', 
                         '--frame-id', 'base_link', 
                         '--child-frame-id', 'camera_link']
        ),

        # --- BRIDGE 3: Fix Camera Timestamps ---
        Node(
            package='midcone_rtabmap',
            executable='restamper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # --- MAIN MAPPING NODE ---
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'] # Delete old database
        ),

        # --- VISUALIZER ---
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        ),
    ])