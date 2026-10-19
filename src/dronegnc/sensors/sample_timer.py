"""
sample_timer.py

Author: Khanh Ngo
"""


class SampleTimer:
    """
    Decides when a sensor produces a new sample. Real sensors run at different rates
    (e.g. IMU 200 Hz, barometer 50 Hz, GPS 10 Hz), and the estimator must handle that.
    """

    def __init__(self, rate_hz):
        """
        Constructor
        rate_hz: Output rate of the sensor [Hz]
        """

        self.period_s = 1.0 / rate_hz
        self.next_sample_time_s = 0.0

    def is_due(self, time_s):
        """
        True if a new sample should be produced at this time
        time_s: Current simulation time [s]
        """

        if time_s + 1e-9 < self.next_sample_time_s:
            return False
        self.next_sample_time_s += self.period_s
        return True
