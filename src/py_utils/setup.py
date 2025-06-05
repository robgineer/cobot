from setuptools import find_packages, setup

package_name = "py_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Harbach",
    maintainer_email="robgineer@gmail.com",
    description="Helper functions for python based launch configurations.",
    license="BSD-3-Clause",
)
