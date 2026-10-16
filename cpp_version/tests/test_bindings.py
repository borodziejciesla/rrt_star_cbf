import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build"))
import safe_cbf_rrt_py as scr  # noqa: E402


def test_rng_matches_numpy():
    for seed in (1, 2, 123):
        np.testing.assert_array_equal(scr.matlab_rand(seed, 100), np.random.RandomState(seed).rand(100))


def test_obstacles_match_example_m():
    circles = scr.generate_obstacles(2, 40, 0.06)
    expected = np.random.RandomState(2).rand(80).reshape(40, 2)
    np.testing.assert_array_equal(circles[:, :2], expected)
    assert np.all(circles[:, 2] == 0.06)


def test_safety_function_and_plan():
    h, dhdx, dhdy = scr.generate_safety_function(scr.generate_obstacles())
    assert h.shape == (100, 100)
    assert np.all(np.isfinite(h)) and h.max() > 0
    occ = (h <= 0).astype(float)
    params = scr.Params(maxIter=10000, v=1.0, stepSize=1, kappa=50, c=0.0, do_cbf=True, seed=3)
    res = scr.plan([10, 10], [90, 90], occ, h, dhdx, dhdy, params)
    assert res["found"]
    np.testing.assert_array_equal(res["path"][0], [10, 10])
    np.testing.assert_array_equal(res["path"][-1], [90, 90])
    assert 0 < res["mean_h_path"] <= 1
