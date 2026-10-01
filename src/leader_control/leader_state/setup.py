import os

from setuptools import find_packages, setup


package_name = 'leader_state'

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
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='phong',
    maintainer_email='ctp1234231510@gmail.com',
    description='Leader arm state aggregator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'leader_state = leader_state.state:main',
        ],
    },
)
