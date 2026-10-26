from setuptools import find_packages, setup
import os
from glob import glob

package_name = "ar_mirror"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="nayzalovehrk@gmail.com",
    description="Mirrors sim joint states onto the real AR4 arm so both move together",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_mirror = ar_mirror.joint_mirror:main",
        ],
    },
)
