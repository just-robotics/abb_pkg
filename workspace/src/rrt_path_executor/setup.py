from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rrt_path_executor"


setup(
    name=package_name,
    version="0.0.0",
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
    maintainer_email="artemkondratev5@gmail.com",
    description="Execute nav_msgs/Path via send_pose.sh",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "path_executor_node = rrt_path_executor.path_executor_node:main",
        ],
    },
)
