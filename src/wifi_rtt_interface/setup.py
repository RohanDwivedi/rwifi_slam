"""Setuptools configuration for the wifi_rtt_interface ROS 2 package."""

import os
from glob import glob
from xml.etree import ElementTree

from setuptools import find_packages, setup

package_name = "wifi_rtt_interface"
# package.xml is the single source of the version; scripts/bump-version.sh updates it.
_package_xml = os.path.join(os.path.dirname(os.path.realpath(__file__)), "package.xml")
version = ElementTree.parse(_package_xml).getroot().findtext("version")

setup(
    name=package_name,
    version=version,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="Rohan Dwivedi",
    maintainer_email="myemail.rohan@gmail.com",
    description="Publishes Wi-Fi RTT range observations from Isaac Sim (Android backend planned).",
    license="Apache-2.0",
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
