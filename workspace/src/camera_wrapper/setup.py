from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'camera_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # config yaml
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='artemkondratev5@gmail.com',
    description='Basler camera wrapper via pypylon',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'pylon_camera_node = camera_wrapper.pylon_camera_node:main',
        ],
    },
)
