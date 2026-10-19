"""
conftest.py

Author: Khanh Ngo
"""

import importlib.util
import sys
from pathlib import Path

import matplotlib
import pytest

matplotlib.use("Agg")  # never open a window during tests

ROOT = Path(__file__).absolute().parent.parent
sys.path.insert(0, str(ROOT / "src"))  # works without "pip install -e ." and on old pytest


@pytest.fixture
def load_simulation():
    """
    Import a simulation script by its path under simulations/ and turn off its plots
    """

    def load(relative_path):
        path = ROOT / "simulations" / relative_path
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.show_plot = False
        return module

    return load
