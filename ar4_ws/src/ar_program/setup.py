import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ar_program"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="sale2@eitlaser.com",
    description="Pre-programmed AR4 motions triggered over ROS 2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_program = ar_program.player:main",
            "teach = ar_program.teach:main",
        ],
    },
)
