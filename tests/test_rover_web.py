#!/usr/bin/env python3
"""
tests/test_rover_web.py
-----------------------
web/rover.html + web/js/rover_control.js: terrain drawing inputs, goal / route points on a
detected crater or rock are refused, and the laser yaw offset defaults to 180. The page script
runs under Node against a fake DOM and WebSocket (tests/js/rover_page_check.js); skipped when
Node is not installed. The HTML default is also checked here without Node.
"""

import os
import re
import shutil
import subprocess

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
HTML = os.path.join(ROOT, 'web', 'rover.html')


def test_laser_yaw_offset_defaults_to_180():
    html = open(HTML, encoding='utf-8').read()
    tag = re.search(r'<input[^>]*id="t-yaw"[^>]*>', html).group(0)
    assert 'value="180"' in tag


def test_page_script_has_terrain_and_refusal_hooks():
    js = open(os.path.join(ROOT, 'web', 'js', 'rover_control.js'), encoding='utf-8').read()
    for needle in ('/terrain', 'objectAt', 'drawTerrain', 'is on a detected',
                   '/terrain/objects', 'drawObjects', 'selectObject', '/terrain/preview/compressed'):
        assert needle in js
    html = open(HTML, encoding='utf-8').read()
    for el in ('id="objects"', 'id="preview"'):
        assert el in html


@pytest.mark.skipif(shutil.which('node') is None, reason='Node.js not installed')
def test_page_behaviour_under_node():
    r = subprocess.run(['node', os.path.join(ROOT, 'tests', 'js', 'rover_page_check.js')],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True,
                       timeout=60)
    assert r.returncode == 0, r.stdout
    assert 'FAIL' not in r.stdout
