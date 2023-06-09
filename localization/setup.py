from setuptools import setup
import os
from glob import glob
package_name = 'localization'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        (os.path.join('share', package_name, "resource"), glob('resource/*.pcd')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed Baza',
    maintainer_email='ahmed.baza@skoltech.ru',
    description='building a ROS frame work for localization with pointcloud localization with ICP',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_loc = localization.icp_loc:main', 
            'map_pub = localization.map_pub:main', 
            'fixed_frames = localization.fixed_frames:main', 
            'robot_tfs = localization.robot_tfs:main', 
        ],
    },
)
