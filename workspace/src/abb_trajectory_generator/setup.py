from glob import glob

from setuptools import setup

package_name = 'abb_trajectory_generator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Artem Kondratiev',
    maintainer_email='kondart.dev@gmail.com',
    description='Generates Cartesian trajectories (circle, line) and executes them through MoveIt',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_generator = abb_trajectory_generator.trajectory_generator_node:main',
        ],
    },
)
