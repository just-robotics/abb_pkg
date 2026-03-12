from setuptools import find_packages, setup
from glob import glob
import os

package_name = "point_cloud_emulator"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="you@todo.todo",
    description="Emulates 3D lidar point cloud with TF docking to manipulator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "point_cloud_emulator_node = point_cloud_emulator.point_cloud_emulator_node:main",
        ],
    },
)
