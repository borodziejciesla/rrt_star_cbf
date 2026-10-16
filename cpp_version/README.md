# Safe CBF-RRT* (C++ port)

C++17 port of the MATLAB implementation in [`../rrt_star_cbf`](../rrt_star_cbf), with
pybind11 bindings for experiments and plotting. The library has no ROS dependency so it
can later be wrapped in a Nav2 `nav2_core::GlobalPlanner` plugin.

## Contents

| Path | Purpose |
| --- | --- |
| `include/safe_cbf_rrt/grid.hpp` | `Grid2D`, `BilinearInterpolant` (= `griddedInterpolant(..., 'linear', 'nearest')`), MATLAB `gradient`, `linspace` |
| `include/safe_cbf_rrt/matlab_rng.hpp` | MATLAB/numpy-compatible MT19937 `rand`, obstacle generator from `example.m` |
| `include/safe_cbf_rrt/poisson_safety.hpp` | Poisson safety function `h`, `dh/dx`, `dh/dy` (finite differences + Eigen) |
| `include/safe_cbf_rrt/planner.hpp` | `safeCbfRrtStar()` - 1:1 port of `SafeCBFRRTStar.m` |
| `python/bindings.cpp` | `safe_cbf_rrt_py` module |
| `scripts/example.py` | Python counterpart of `example.m`, writes comparison figures to `fig/` |
| `tests/` | gtest (`test_safe_cbf_rrt.cpp`) and pytest (`test_bindings.py`) |

## Build and run

Dependencies (all available as Ubuntu 22.04 packages): `libeigen3-dev`, `pybind11-dev`,
`libgtest-dev`, `python3-dev`, plus numpy, scipy, matplotlib for the scripts.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/test_safe_cbf_rrt            # C++ unit tests
python3 -m pytest -q tests           # binding tests
python3 scripts/example.py           # N=10 trials with and without CBF, figures in fig/
python3 scripts/example.py -N 50     # more trials
```

Python usage:

```python
import sys; sys.path.insert(0, "build")
import safe_cbf_rrt_py as scr

circles = scr.generate_obstacles(seed=2, n=40, r=0.06)   # (n, 3) [cx, cy, r] in [0,1]^2
h, dhdx, dhdy = scr.generate_safety_function(circles)    # 100 x 100 arrays, row = y
occ = (h <= 0).astype(float)
params = scr.Params(maxIter=10000, v=1.0, stepSize=1, kappa=50, c=0.0, do_cbf=True, seed=1)
res = scr.plan([10, 10], [90, 90], occ, h, dhdx, dhdy, params)
res["path"], res["path_length"], res["mean_h_path"], res["runtime_ms"]
```

Coordinates follow the MATLAB code: grids are `Ny x Nx`, points are `[x, y]` in 1-based
pixel coordinates (`x in [1, Nx]`, `y in [1, Ny]`).

## Notes on the port

- **Safety function without the PDE Toolbox.** In `GeneratePoissonSafetyFunction.m`,
  `bc_location` ends every iteration with `bcval(:,k) = [0.1; 0.1]`, overwriting the obstacle
  branch, so the first PDE has the constant solution `u = (0.1, 0.1)` and the source is
  `|u| = sqrt(0.02)`. The second solve is therefore `Laplace(h) = -sqrt(0.02)` on the unit
  square minus the circles with `h = 0` on all boundaries. The port solves this with a 5-point
  stencil on a grid 4x finer than the output (`fine` parameter; results converge from 4 up)
  and samples it onto the 100x100 grid.
- **Inside obstacles `h = 0`, not NaN.** MATLAB's `griddata` triangulates the convex hull of
  the FEM nodes, so it interpolates the zero boundary values across obstacle holes; the
  gradient therefore stays non-zero next to obstacles. Treating those cells as NaN (and later
  zero gradient) makes the CBF far too permissive near obstacles.
- **Planner quirks kept for parity:** neighbour set computed before inserting the new node,
  rewiring does not propagate costs to descendants, the search stops at the first goal
  connection, `rejection_ratio = (iter - accepted) / iter`, `mean_h` clamped to `[0, 1]`.
- **Reproducibility:** MATLAB calls `rng('shuffle')` inside the planner, so its trials are not
  reproducible. The port takes an explicit `seed` (`example.py` uses seeds `1..N`).

## Comparison with MATLAB

`scripts/example.py` produces:

| Figure | Content |
| --- | --- |
| `fig/base_cpp.png`, `fig/ours_cpp.png` | Same layout as `rrt_star_cbf/fig/base.png`, `ours.png` |
| `fig/base_mat.png`, `fig/ours_mat.png` | Same, C++ planner on the MATLAB fields from `example.mat` |
| `fig/distance_cpp.png` | Same layout as `rrt_star_cbf/fig/distance.png` |
| `fig/compare_base.png`, `fig/compare_ours.png` | MATLAB figure, C++ planner on C++ fields, C++ planner on MATLAB fields |
| `fig/compare_distance.png` | C++ profiles overlaid on curves digitized from `distance.png` |
| `fig/compare_distance_mat.png` | Per variant: C++ fields, MATLAB fields, digitized MATLAB curve |
| `fig/compare_fields.png` | `h` and `|grad h|`: MATLAB vs C++ and their relative difference |
| `fig/compare_occupancy.png` | C++ occupancy vs obstacle layout sampled from the MATLAB PNGs |
| `fig/h_field.png` | `h` and gradient directions |

### MATLAB field export (`example.mat`)

`rrt_star_cbf/example.mat` holds `h`, `dhdx`, `dhdy` saved from `example.m`
(`save example.mat h dhdx dhdy` after `GeneratePoissonSafetyFunction`). `example.py` loads it
by default (`--mat PATH` to point elsewhere, `--mat ''` to skip) and repeats every run on those
fields, which separates planner differences from Poisson-solver differences.

Agreement of the C++ safety function with `example.mat`:

| Quantity | Value |
| --- | --- |
| `max h` (MATLAB / C++) | 0.00125 / 0.00126 |
| Occupancy cells differing | 16 / 10000 |
| `h` relative error, median / 95th percentile | 2.7 % / 28.7 % (largest in the first cell next to obstacles) |
| `|grad h|` relative error, median | 1.1 % |
| `grad h` direction error, median | 0.39 deg |

### Results (N = 10, seeds 1..10)

| Metric | MATLAB | C++ planner, C++ fields | C++ planner, MATLAB fields |
| --- | --- | --- | --- |
| Mean obstacle distance, without CBF | 3.743 +/- 0.831 (README), 3.68 (`distance.png`) | 3.466 +/- 1.387 | 3.466 +/- 1.388 |
| Mean obstacle distance, with CBF | 4.658 +/- 0.326 (README), 5.80 (`distance.png`) | 5.988 +/- 0.363 | 6.063 +/- 0.247 |
| CBF paths taking the left-wall route | 10/10 (`ours.png`) | 1/10 | 8/10 |
| Runtime per plan, without / with CBF | - | ~14 ms / ~40 ms | ~14 ms / ~68 ms |

- The obstacle layout is reproduced exactly (MATLAB `rng(2); rand` == numpy `RandomState(2)`).
- Without CBF the paths and the distance profile match MATLAB closely with either field set.
- With CBF on the MATLAB fields, the C++ planner reproduces MATLAB's behaviour: most trees
  reach the goal along the left and top walls and the distance profile follows MATLAB's
  `distance.png`. The planner port is therefore faithful.
- With CBF on the C++ fields, most trees take the central corridor instead. The fields differ
  by only a few percent, mostly in the cells next to obstacles, but the CBF condition
  `v * grad(h) . d + kappa * h > 0` is very sensitive to `h / |grad h|` there, which tips the
  route choice.
- The README value 4.658 for the CBF run does not match MATLAB's own `distance.png`, whose
  curve averages 5.80; the C++ results (5.99 and 6.06) are close to the figure.
