from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'kit_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install calibration and grasp parameter files.
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join("share", package_name, "config"),glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='min',
    maintainer_email='alekdi8gm30@gmail.com',
    description='ROS 2 motion and perception controller for relief kit assembly',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = kit_robot.controller:main',
            'position_estimation = kit_robot.position_estimation:main',
        ],
    },
)
