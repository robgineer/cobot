from setuptools import find_packages, setup
from glob import glob

package_name = "py_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Harbach",
    maintainer_email="robgineer@gmail.com",
    description="IK / FK demos using the MoveIt2 Python API",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "simple_ik_fk = py_demo.simple_ik_fk:main",
            "ik_reachable_set = py_demo.ik_reachable_set:main",
        ],
    },
)
