from setuptools import setup
from glob import glob
import os

package_name = 'r2_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.action_servers', f'{package_name}.topic_nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='R2 Team',
    maintainer_email='dev@r2.local',
    description='R2 hardware execution layer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'suspension_action_server = r2_hardware.action_servers.suspension_action_server:main',
            'arm_action_server = r2_hardware.action_servers.arm_action_server:main',
            'spear_action_server = r2_hardware.action_servers.spear_action_server:main',
            'odom_simulator = r2_hardware.topic_nodes.odom_simulator:main',
        ],
    },
)
