from setuptools import find_packages, setup

package_name = 'perception_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="Robert Harbach",
    maintainer_email="robgineer@gmail.com",
    description="Python based object detection and recognition pipeline using a (simulated) RGBD camera.",
    license="BSD-3-Clause",
    entry_points={
        'console_scripts': [],
    },
)
