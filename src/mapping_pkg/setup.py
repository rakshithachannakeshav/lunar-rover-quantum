from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mapping_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rakshitha Channakeshav',
    maintainer_email='rakshithachannakeshav@gmail.com',
    description='Occupancy grid mapping and terrain classification for the lunar rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_node = mapping_pkg.occupancy_grid_node:main',
            'terrain_classifier_node = mapping_pkg.terrain_classifier_node:main',
        ],
    },
)
