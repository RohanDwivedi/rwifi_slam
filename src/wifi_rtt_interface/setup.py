"""Setuptools configuration for the wifi_rtt_interface ROS 2 package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "wifi_rtt_interface"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="myemail.rohan@gmail.com",
    description="Wi-Fi RTT publisher with simulation and Android API backends for ROS 2.",
    license="Apache 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "wifi_rtt_publisher = wifi_rtt_interface.wifi_rtt_publisher:main",
        ],
    },
)
