# SPDX-FileCopyrightText: 2025 Komiya Takumi
# SPDX-License-Identifier: Apache-2.0

from setuptools import find_packages, setup

package_name = 'ros2_system_watchdog'

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
    maintainer='Komiya Takumi',
    maintainer_email='s24c1052rg@s.chibakoudai.jp',
    description='ROS 2 node for monitoring system CPU usage.',
    license='Apache-2.0',
    tests_require=['pytest', 'launch_testing', 'launch_testing_ros'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'watchdog = ros2_system_watchdog.watchdog_node:main'
        ],
    },
)
