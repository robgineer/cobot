from setuptools import find_packages, setup

package_name = 'offline_hand_eye'

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
    maintainer='ubuntu',
    maintainer_email='thao.dang@hs-esslingen.de',
    description='A ROS2 package for offline hand-eye calibration, including data recording and calibration publishing tools.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_calib_data = offline_hand_eye.record_calib_data:main',
            'calib_publisher = offline_hand_eye.calib_publisher:main',
        ],
    },
)
