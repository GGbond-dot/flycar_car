import os
from glob import glob

from setuptools import find_packages, setup


package_name = "wifi_ap_manager"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@example.com",
    description="Minimal ROS2 helper for starting an open Wi-Fi access point with NetworkManager.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ap_manager = wifi_ap_manager.ap_manager:main",
        ],
    },
)
