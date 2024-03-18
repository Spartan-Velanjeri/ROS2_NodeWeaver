import os
from glob import glob

from setuptools import setup

package_name = "fpm_plc_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
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
    maintainer="die2si",
    maintainer_email="die2si@de.bosch.com",
    description="TODO: Package description",
    license="TBD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vacuum_driver = fpm_plc_driver.vacuum_driver:main",
            "lift_driver = fpm_plc_driver.lift_driver:main",
        ],
    },
)
