import os
from setuptools import find_packages, setup

package_name = "navigation_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            [
                "launch/navigation_pipeline.launch.py",
                "launch/path_executor.launch.py",
                "launch/quantum_path_executor.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rakshitha Channakeshav",
    maintainer_email="rakshithachannakeshav@gmail.com",
    description="Navigation and path execution for the lunar rover",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_executor = navigation_pkg.path_executor_node:main",
            "quantum_path_executor = navigation_pkg.quantum_path_executor_node:main",
        ],
    },
)
)