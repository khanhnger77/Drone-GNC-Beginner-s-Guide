#!/bin/bash
# Run all tests (the simulation tests set show_plot = False, so no window opens).
# Uses .venv when it exists. Plugin autoload is off, so pytest plugins from other
# environments on PYTHONPATH (e.g. ROS) cannot break the run.
cd "$(dirname "$0")"
PYTHON=python3
[ -x .venv/bin/python ] && PYTHON=.venv/bin/python
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$PYTHON" -m pytest -l --durations=0 "$@"
