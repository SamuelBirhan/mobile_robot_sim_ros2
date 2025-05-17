from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mobile_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include config files
        ('share/' + package_name + '/config', ['config/controller_params.yaml', 'config/simulator_params.yaml']),
        
        # Include launch files
        ('share/' + package_name + '/launch', ['launch/launcher.py']),
        
        # Include RViz files (if you have them)
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='283492@student.pwr.edu.pl',
    description='Mobile Robot Simulation in ROS 2',
    license='Apache-2.0',  # Change if you have a different license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = mobile_robot_sim.simulator:main',  # Simulator node
            'controller = mobile_robot_sim.controller:main',  # Controller node
        ],
    },
)
