from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'leader_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'experiments'),
         glob('config/experiments/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phong',
    maintainer_email='ctp1234231510@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'teleop = leader_controller.teleop:main',
            'calibrate_origin = leader_controller.calibrate_origin:main',
        ],
    },
)
