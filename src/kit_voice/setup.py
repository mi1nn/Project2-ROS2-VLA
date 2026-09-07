from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'kit_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        # glob('resource/*') 는 점(.)으로 시작하는 파일을 안 잡는다 — .env 는 따로 챙긴다.
        (os.path.join('share', package_name, 'resource'), glob('resource/.env')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='min',
    maintainer_email='alekdi8gm30@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'get_command = kit_voice.get_keyword:main',
        ],
    },
)
