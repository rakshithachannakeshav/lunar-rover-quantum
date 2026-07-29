#!/bin/bash
set -e

echo "=== PHASE E: PYTHON + QUANTUM STACK SETUP ==="

echo "--- Step 1: Install python3-pip and python3-venv ---"
sudo apt-get update -q
sudo apt-get install -y python3-pip python3-venv python3-dev

echo "--- Step 2: Create project directory and virtual environment ---"
mkdir -p ~/lunar-rover-quantum
cd ~/lunar-rover-quantum

if [ -d ".venv" ]; then
    echo "Virtual environment already exists, skipping creation."
else
    python3 -m venv .venv
    echo "Virtual environment created at ~/lunar-rover-quantum/.venv ✅"
fi

echo "--- Step 3: Activate venv and upgrade pip ---"
source .venv/bin/activate
pip install --upgrade pip setuptools wheel

echo "--- Step 4: Install all required Python packages ---"
pip install \
    "qiskit>=1.0.0" \
    "qiskit-aer>=0.14.0" \
    "qiskit-algorithms>=0.3.0" \
    "networkx>=3.0" \
    "numpy>=1.24.0" \
    "scipy>=1.10.0" \
    "matplotlib>=3.7.0" \
    "seaborn>=0.12.0" \
    "opencv-python-headless>=4.8.0" \
    "pyyaml>=6.0" \
    "tqdm>=4.65.0" \
    "pandas>=2.0.0"

echo "--- Step 5: Verify all imports ---"
python3 -c "
import sys
print('Python:', sys.version)

import qiskit
print('Qiskit version:', qiskit.__version__)

from qiskit_aer import AerSimulator
sim = AerSimulator()
print('AerSimulator:', sim)

import networkx as nx
G = nx.Graph()
G.add_edge('A','B', weight=1.5)
G.add_edge('B','C', weight=2.0)
path = nx.shortest_path(G, 'A', 'C', weight='weight')
print('NetworkX shortest path:', path)

import numpy as np
arr = np.array([1,2,3])
print('NumPy version:', np.__version__, '| Test array:', arr)

import scipy
print('SciPy version:', scipy.__version__)

import matplotlib
print('Matplotlib version:', matplotlib.__version__)

import seaborn
print('Seaborn version:', seaborn.__version__)

import cv2
print('OpenCV version:', cv2.__version__)

import yaml
print('PyYAML version:', yaml.__version__)

import tqdm
print('tqdm version:', tqdm.__version__)

import pandas as pd
print('Pandas version:', pd.__version__)

print('')
print('All quantum and Python packages working! ✅')
"

echo ""
echo "=== PHASE E COMPLETE ==="

echo "--- Configuring ~/.bashrc venv activation hint ---"
BASHRC="/home/monis/.bashrc"
VENV_COMMENT="# Lunar Rover Quantum: activate with: source ~/lunar-rover-quantum/.venv/bin/activate"
if ! grep -Fq "lunar-rover-quantum" "$BASHRC"; then
    echo "$VENV_COMMENT" >> "$BASHRC"
fi

echo "Done! To activate the venv later run:"
echo "  source ~/lunar-rover-quantum/.venv/bin/activate"
