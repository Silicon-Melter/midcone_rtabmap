import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    common_params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'approx_sync': True,       # Critical: AI depth might have slightly different timestamps
        'wait_for_transform': 0.2
    }

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('midcone_bringup'), 'launch', 'urdf.launch.py')
        ]),
        launch_arguments={'model': 'drone.urdf'}.items()
    )


    qos_bridge_node = Node(
        package='midcone_rtabmap',
        executable='qos_bridge',
        output='screen'
    )

    depth_to_cloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        output='screen',
        parameters=[{'queue_size': 10}],
        remappings=[
            ('image_rect', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('camera_info', '/camera_info_fixed'), 
            ('points', '/camera/cloud_generated')
        ]
    )
    # --- 2. 3D ICP ODOMETRY ---
    # Takes the generated cloud and calculates movement.
    icp_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
            'odom_frame_id': 'odom',
            'Reg/Strategy': '1',
            'Odom/MaxTranslation': '2.0',
            'Odom/MaxRotation': '2.0',
            'Odom/ScanKeyFrameThr': '0.4',
            'Odom/GuessMotion': 'true',  
            'Odom/ResetCountdown': '1',
            'Icp/MaxTranslation': '2.0', 
            'Icp/MaxRotation': '2.0',
        }],
        remappings=[
            ('scan_cloud', '/camera/cloud_generated'),
            ('imu', '/mavros/imu/data'),
            ('odom', '/odom')
        ]
    )
    
    

    # --- 3. RTAB-MAP SLAM ---
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[common_params, {
            'map_frame_id': 'map',
            'subscribe_odom': True,       # Listen to rgbd_odometry
            'Mem/IncrementalMemory': 'true',
            
            # --- LIDAR MAPPING ---
            # We use Lidar to paint the walls, but rely on Camera for position
            'Reg/Strategy': '0',          # 0=Visual (Trust the Odom node)
            'RGBD/ProximityBySpace': 'true', # Find loop closures
        }],
        remappings=[
            ('odom',            '/odom'), # Connects to rgbd_odometry above
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw')
        ],
        arguments=['--delete_db_on_start'] # Starts fresh every time
    )

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
        qos_bridge_node,
        depth_to_cloud_node,
        icp_odom_node,
        rtabmap_slam_node,
        rtabmap_viz_node
    ])