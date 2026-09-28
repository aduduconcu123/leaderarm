import os

from setuptools import find_packages, setup


package_name = 'feetech_driver'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            os.path.join('share', package_name),
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'calibration'),
            ['calibration/calibration.json']
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='phong',
    maintainer_email='phong@example.com',

    description='ROS 2 driver for Feetech STS3250 motors',

    license='Apache-2.0',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
            'feetech_node = feetech_driver.node:main',
        ],
    },
)
