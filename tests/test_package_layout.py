#!/usr/bin/env python3
"""
tests/test_package_layout.py
----------------------------
Every ament_python package that installs console scripts needs a setup.cfg that
redirects the scripts to lib/<pkg>/. Without it pip puts them in bin/, where
`ros2 run` / `ros2 launch` do not look, and the node "executable is not found".
No ROS 2 dependency.
"""

import configparser
import os
import re

import pytest

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))


def _python_packages_with_scripts():
    packages = []
    for name in sorted(os.listdir(SRC_DIR)):
        pkg_dir = os.path.join(SRC_DIR, name)
        package_xml = os.path.join(pkg_dir, 'package.xml')
        setup_py = os.path.join(pkg_dir, 'setup.py')
        if not (os.path.isfile(package_xml) and os.path.isfile(setup_py)):
            continue
        with open(package_xml, encoding='utf-8') as f:
            if 'ament_python' not in f.read():
                continue
        with open(setup_py, encoding='utf-8') as f:
            # `console_scripts` followed by a non-empty list
            if re.search(r"console_scripts['\"]\s*:\s*\[\s*['\"]", f.read()):
                packages.append(name)
    return packages


PACKAGES = _python_packages_with_scripts()


def test_the_scan_finds_the_python_packages():
    # Guards the guard: if the scan ever finds nothing, the parametrised test below is vacuous.
    assert {'evaluation_pkg', 'mapping_pkg', 'navigation_pkg', 'planning_pkg'} <= set(PACKAGES)


@pytest.mark.parametrize('package', PACKAGES)
def test_scripts_install_into_lib_dir(package):
    cfg_path = os.path.join(SRC_DIR, package, 'setup.cfg')
    assert os.path.isfile(cfg_path), (
        f"{package} installs console scripts but has no setup.cfg; add one with "
        f"install_scripts=$base/lib/{package}"
    )

    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(cfg_path, encoding='utf-8')
    expected = f'$base/lib/{package}'
    assert cfg.get('install', 'install_scripts') == expected
    assert cfg.get('develop', 'script_dir') == expected  # used by `colcon build --symlink-install`
