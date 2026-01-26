from setuptools import find_packages, setup

package_name = "wild_slam_ros2"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rohan Dwivedi",
    maintainer_email="myemail.rohan@gmail.com",
    description="WiLD-SLAM ROS2 package for Simultaneous Localization and Mapping",
    license="Apache 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
