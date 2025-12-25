import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- 1. CAPTURE CONFIGURATION FROM COMMAND LINE ---
    # These variables hold the values you pass with ":="
    use_sim_time = LaunchConfiguration('use_sim_time')
    subscribe_odom_info = LaunchConfiguration('subscribe_odom_info')
    args = LaunchConfiguration('args')

    # --- 2. DEFINE PARAMETERS ---
    # We insert the LaunchConfiguration variables here instead of hardcoded True/False
    parameters=[{
          'frame_id': 'base_link', 
          'subscribe_depth': True,
          'subscribe_odom_info': subscribe_odom_info, # <--- NOW DYNAMIC
          'approx_sync': True,
          'wait_imu_to_init': True,
          'use_sim_time': use_sim_time                # <--- CRITICAL FOR BAGS
    }]

    # --- 3. DEFINE REMAPPINGS ---
    remappings=[
          ('imu', '/mavros/imu/data'),
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'), 
          ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
          ('odom', '/mavros/local_position/odom')
    ] 

    return LaunchDescription([

        # --- 4. DECLARE ARGUMENTS ---
        # This allows the launch file to accept these flags
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Use simulation time? Set to true for bag playback.'),

        DeclareLaunchArgument(
            'subscribe_odom_info', 
            default_value='true',
            description='Subscribe to odom_info? Set to false if not in bag.'),
        
        DeclareLaunchArgument(
            'args', 
            default_value='',
            description='Extra arguments set to rtabmap nodes (e.g. --delete_db_on_start).'),

        # --- 5. LAUNCH NODES ---
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            # '-d' deletes the previous database so you don't merge old maps
            arguments=['-d', args] 
        ),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        ),     
    ])