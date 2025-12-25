import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    # 1. SHARED PARAMETERS
    common_params = [{
          'use_sim_time': True,
          'frame_id': 'base_link',
          'subscribe_depth': True,
          'subscribe_odom_info': True,
          'approx_sync': True,          
          'wait_imu_to_init': True,
          'wait_for_transform': 0.2
    }]

    # 2. SHARED REMAPPINGS
    common_remappings = [
          ('rgb/image',       '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'), 
          ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
          ('scan',            '/scan'),
          ('imu',             '/imu/data') 
    ]
    
    config_path = os.path.join(
        get_package_share_directory('midcone_rtabmap'),
        'config',
        'vo_icp_ekf.yaml'
    )
        
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('midcone_bringup'), 'launch', 'urdf.launch.py')
        ]),
        launch_arguments={'model': 'drone.urdf'}.items()
    )


    # 3. LIDAR ODOMETRY
    """ lidar_icp_node = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=common_params + [{
            'odom_frame_id': 'odom',
            'guess_frame_id': 'odom',         # FIX: Disable feedback loop
            'publish_tf': False,
            
            # --- FIX: ARTIFICIAL VARIANCE ---
            # Prevents the "0 covariance" error
            'Odom/Variance': '0.01',      

            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            'Odom/MaxTranslation': '2.0',
            'Odom/MaxRotation': '2.0',
            'Odom/ScanKeyFrameThr': '0.4',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
        }],
        remappings=common_remappings + [('odom', '/odom_lidar')],
        arguments=["--ros-args", "--log-level", 'icp_odometry:=warn']
    ) """

    lidar_icp_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'approx_sync': True,
            'publish_tf': False,
            'wait_for_transform': 0.2,
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
            ('odom', '/odom_lidar'),
        ]
    )

    # 4. VISUAL ODOMETRY
    rgbd_vo_node = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=common_params + [{
            'publish_tf': False,
            'odom_frame_id': 'odom',
        }],
        remappings=common_remappings + [('odom', '/odom_rgbd')]
    )

    # 5. EKF FUSION
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odom')]
    )

    # 6. SLAM
    rtabmap_slam_node = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=common_params + [{
            'subscribe_scan': True,
            'map_frame_id': 'map',
            'subscribe_odom': True,
            'Mem/NotLinkedNodesKept': 'false',
            'Icp/PointToPlaneMinComplexity': '0.04'
        }],
        remappings=common_remappings + [('odom', '/odom')],
        arguments=['--delete_db_on_start']
    )

    # 7. VISUALIZATION
    rtabmap_viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=common_params,
        remappings=common_remappings + [('odom', '/odom')]
    )

    # 8. IMU FILTER
    imu_filter = Node(
        package='imu_filter_madgwick', 
        executable='imu_filter_madgwick_node', 
        output='screen',
        parameters=[{
            'use_mag': False, 
            'world_frame': 'enu', 
            'publish_tf': False
        }],
        remappings=[
            ('imu/data_raw', '/mavros/imu/data'),
            ('imu/data', '/imu/data')
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        lidar_icp_node,
        rgbd_vo_node,
        ekf_node,
        rtabmap_slam_node,
        rtabmap_viz_node,
        imu_filter
    ])