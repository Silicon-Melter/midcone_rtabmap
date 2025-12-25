import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    
    # --- COMMON PARAMETERS ---
    common_params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'approx_sync': True,
        'wait_for_transform': 0.2
    }

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('midcone_bringup'), 'launch', 'urdf.launch.py')
        ]),
        launch_arguments={'model': 'drone.urdf'}.items()
    )
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[common_params, {
            'odom_frame_id': 'odom',
            'Reg/Strategy': '1', 
            'Reg/Force3DoF': 'false',
            'Odom/MaxTranslation': '2.0',
            'Odom/MaxRotation': '2.0',
            'Odom/ScanKeyFrameThr': '0.4',
            'Odom/GuessMotion': 'true',  
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ]
    )

    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[common_params, {
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_rgb': True,  
            
            'Reg/Strategy': '1',          
            'Reg/Force3DoF': 'true',      
            'Mem/IncrementalMemory': 'true',
            'sync_queue_size': 40,
            'topic_queue_size': 40,
            
            # Visuals
            'Grid/Sensor': '1',           
            'Grid/3D': 'true',            
        }],
        remappings=[
            ('odom',            '/odom'),
            ('rgb/image',       '/camera/camera/color/image_raw'),       
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw') 
        ],
        arguments=['--delete_db_on_start'] 
    )

    # --- 3. VISUALIZATION ---
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[common_params, {
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True
        }],
        remappings=[
            ('scan',            '/scan'),
            ('odom',            '/odom'),
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw')
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        icp_odometry_node,
        rtabmap_slam_node,
        rtabmap_viz_node
    ])