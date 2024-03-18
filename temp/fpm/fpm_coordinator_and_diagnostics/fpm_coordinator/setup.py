import os
from glob import glob

from setuptools import setup

package_name = "fpm_coordinator"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, f"{package_name}/actions"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ssl2lr",
    maintainer_email="ssl2lr@de.bosch.com",
    description="Coordinator module for fpm.",
    license="TBD",
    tests_require=["pytest", "bautiro_ros_interfaces"],
    entry_points={
        "console_scripts": [
            "coordinator = fpm_coordinator.coordinator:main",
        ],
    },
)
