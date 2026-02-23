"""Setup for rwifi_slam_ros2: Wi-Fi RTT SLAM launch and config."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rwifi_slam_ros2"

setup(
    name=package_name,
    version="0.13.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["scipy", "numpy", "setuptools", "g2opy", "pyoctomap"],
    zip_safe=True,
    maintainer="Rohan Dwivedi",
    maintainer_email="myemail.rohan@gmail.com",
    description="WiFi localisation and mapping for robots in ambient environments.",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "rwifi_slam_node = rwifi_slam_ros2.rwifi_slam_ros2_node:main",
        ],
    },
)
