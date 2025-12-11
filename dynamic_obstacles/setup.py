from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dynamic_obstacles'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaxye2200',
    maintainer_email='your.email@example.com',
    description='Dynamic obstacles for TurtleBot3 navigation testing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_manager = dynamic_obstacles.obstacle_manager:main',
        ],
    },
)

