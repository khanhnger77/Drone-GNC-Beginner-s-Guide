"""
history.py

Author: Khanh Ngo
"""

import numpy as np


class History:
    """
    Records values every step, like a tiny flight log (PX4: ulog).
    Drawing happens after the run, so the simulation itself is fast and testable.

    Usage:
        history.record(time_s=t, true_x_m=x)
        history["true_x_m"]  # numpy array over time
    """

    def __init__(self):
        self._data = {}

    def record(self, **values):
        for key, value in values.items():
            self._data.setdefault(key, []).append(np.copy(value))

    def __getitem__(self, key):
        return np.array(self._data[key])

    def __len__(self):
        return len(next(iter(self._data.values()), []))
