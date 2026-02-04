from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'midcone_rtabmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Install Config Files (CRITICAL FIX)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 3. Install Scripts (Optional, if you have standalone scripts in a 'scripts' folder)
        # (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='earth',
    maintainer_email='noppawit37815@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_converter = midcone_rtabmap.px4_converter:main',
            'odom_to_tf = midcone_rtabmap.odom_to_tf:main',
            'odom_to_tf_sim = midcone_rtabmap.odom_to_tf_sim:main',
            'ghost_tf = midcone_rtabmap.ghost_tf:main',
            'qos_bridge = midcone_rtabmap.qos_bridge:main',
            'lidar_gating_node = midcone_rtabmap.lidar_gating_node:main',
            'hybrid_odom_node = midcone_rtabmap.hybrid_odom_node:main',
            'path_plotter = midcone_rtabmap.path_plotter:main',
            'restamper = midcone_rtabmap.restamper:main',
        ],
    },
)