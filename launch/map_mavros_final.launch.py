import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- CONFIGURATION ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    subscribe_odom_info = LaunchConfiguration('subscribe_odom_info')
    database_path = LaunchConfiguration('database_path')
    args = LaunchConfiguration('args')

    # --- RTAB-MAP PARAMETERS ---
    parameters=[{
          'frame_id': 'base_link', 
          'subscribe_depth': True,
          'subscribe_odom_info': subscribe_odom_info,
          'approx_sync': True,
          'wait_imu_to_init': True,
          'use_sim_time': use_sim_time,
          'database_path': database_path,
          
          # Optimization
          'queue_size': 20,
          'qos_image': 2,
          'qos_camera_info': 2,
    }]

    # --- REMAPPINGS for RTAB-Map ---
    # RTAB-Map listens to the "unzipped" RAW topics created by our nodes below
    remappings=[
          ('imu', '/mavros/imu/data'),
          ('rgb/image', '/camera/camera/color/image_raw/uncompressed'), # <--- Listening to our local unzip node
          ('rgb/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'), 
          ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw/uncompressed'), # <--- Listening to our local unzip node
          ('odom', '/mavros/local_position/odom')
    ] 

    return LaunchDescription([
        
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('subscribe_odom_info', default_value='true'),
        DeclareLaunchArgument('args', default_value=''),

        # ---------------------------------------------------------
        # DECOMPRESSOR 1: RGB (Reads Bag -> Publishes Raw)
        # ---------------------------------------------------------
        Node(
            package='image_transport',
            executable='republish',
            name='rgb_decompressor',
            output='screen',
            # REMOVED arguments=['compressed', 'raw'] (It was ignored)
            # ADDED specific parameters to FORCE the mode:
            parameters=[{
                'use_sim_time': use_sim_time,
                'in_transport': 'compressed',
                'out_transport': 'raw'
            }],
            remappings=[
                ('in/compressed', '/camera/camera/color/image_raw/compressed'),
                ('out', '/camera/camera/color/image_raw/uncompressed')
            ]
        ),

        # ---------------------------------------------------------
        # DECOMPRESSOR 2: DEPTH (Reads Bag -> Publishes Raw)
        # ---------------------------------------------------------
        Node(
            package='image_transport',
            executable='republish',
            name='depth_decompressor',
            output='screen',
            # Force 'compressedDepth' mode here:
            parameters=[{
                'use_sim_time': use_sim_time,
                'in_transport': 'compressedDepth',
                'out_transport': 'raw'
            }],
            remappings=[
                ('in/compressedDepth', '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth'),
                ('out', '/camera/camera/aligned_depth_to_color/image_raw/uncompressed')
            ]
        ),

        # ---------------------------------------------------------
        # MAIN NODE: RTAB-MAP
        # ---------------------------------------------------------
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d', args] 
        ),

        # ---------------------------------------------------------
        # VISUALIZER
        # ---------------------------------------------------------
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        ),     
    ])