from setuptools import setup, find_packages

package_name = 'rover_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['rover_simulation', 'rover_simulation.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Monis',
    maintainer_email='monis@example.com',
    description='Lunar rover simulation package for ROS2 Jazzy + PyBullet',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pybullet_sim = rover_simulation.pybullet_sim:main',
            'odom_to_tf   = rover_simulation.odom_to_tf:main',
        ],
    },
)
