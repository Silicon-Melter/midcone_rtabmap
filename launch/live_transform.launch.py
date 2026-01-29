from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. ODOMETRY BRIDGE
        # Converts /mavros/local_position/odom -> TF (odom -> base_link)
        Node(
            package='midcone_rtabmap',
            executable='odom_to_tf',
            output='screen',
            # We don't need sim_time because we are live!
            parameters=[{'use_sim_time': False}],
            remappings=[('odom', '/mavros/local_position/odom')]
        ),

        # 2. CAMERA POSITION (The Glue)
        # Publishes TF (base_link -> camera_link)
        # Verify your frame_id matches what the camera publishes!
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1', '--y', '0', '--z', '0', 
                         '--yaw', '-1.5707', '--pitch', '0', '--roll', '-1.5707', 
                         '--frame-id', 'base_link', 
                         '--child-frame-id', 'camera_link']
        ),
    ])