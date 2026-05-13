import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'trash_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davut',
    maintainer_email='davut@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'environment = trash_sim.environment_node:main',
            'robot = trash_sim.robot_node:main',
            'visualizer = trash_sim.visualizer_node:main',
            'rviz_publisher_node = trash_sim.rviz_publisher_node:main',
        ],
    },
)
