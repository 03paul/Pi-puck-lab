from __future__ import annotations

import unittest

import numpy as np

from experiments.calibrate_travel_model import fit_travel_model


class CalibrationTests(unittest.TestCase):
    def test_fit_recovers_known_travel_model(self) -> None:
        rng = np.random.default_rng(7)
        a_true, v_true = 1.2, 0.08
        distances = np.linspace(0.15, 2.83, 20)
        times = a_true + distances / v_true + rng.normal(0.0, 0.05, size=distances.shape)

        a_fit, v_fit, r_squared = fit_travel_model(distances, times)

        self.assertAlmostEqual(a_fit, a_true, delta=0.15)
        self.assertAlmostEqual(v_fit, v_true, delta=0.005)
        self.assertGreater(r_squared, 0.99)

    def test_fit_rejects_non_increasing_travel_time(self) -> None:
        distances = np.array([0.2, 0.5, 1.0, 1.5])
        times = np.array([5.0, 4.0, 3.0, 2.0])  # travel time decreasing with distance
        with self.assertRaises(ValueError):
            fit_travel_model(distances, times)


if __name__ == "__main__":
    unittest.main()
