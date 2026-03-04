import os

from setuptools import find_packages, setup
from glob import glob


package_name = "abb_lerobot"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="artemkondratev5@gmail.com",
    description="ABB + LeRobot dataset recorder package",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dataset_recorder = abb_lerobot.dataset_recorder:main",
        ],
    },
)

