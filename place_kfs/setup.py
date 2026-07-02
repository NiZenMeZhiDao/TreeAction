from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'place_kfs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='R2 Team',
    maintainer_email='dev@r2.local',
    description='Top-level KFS placement action orchestration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'place_kfs_action_server = place_kfs.place_kfs_action_server:main',
        ],
    },
)
