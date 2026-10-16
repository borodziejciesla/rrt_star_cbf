#!/usr/bin/env python3
"""Python counterpart of rrt_star_cbf/example.m using the C++ Safe CBF-RRT* port.

Runs N planner trials without and with the CBF safety filter, prints the same
statistics as example.m and writes figures to ../fig. If the MATLAB export
rrt_star_cbf/example.mat (h, dhdx, dhdy) exists, every run is repeated with
those fields so the planner can be compared independently of the Poisson solver.

  base_cpp.png, ours_cpp.png      paths over the occupancy map (like fig/base.png, fig/ours.png)
  base_mat.png, ours_mat.png      same, C++ planner on the MATLAB fields
  distance_cpp.png                mean distance-to-obstacle profile (like fig/distance.png)
  compare_base.png, compare_ours.png
                                  MATLAB figure | C++ fields | MATLAB fields
  compare_distance.png            C++ profiles overlaid on curves digitized from fig/distance.png
  compare_distance_mat.png        per variant: C++ fields, MATLAB fields, digitized MATLAB curve
  compare_fields.png              h and |grad h|: MATLAB vs C++ and relative difference
  compare_occupancy.png           C++ occupancy vs the obstacle layout read from the MATLAB PNG
  h_field.png                     safety function h with its gradient field
"""

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import ndimage  # noqa: E402
from scipy.io import loadmat  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "build"))
import safe_cbf_rrt_py as scr  # noqa: E402

MATLAB_FIG_DIR = os.path.join(os.path.dirname(ROOT), "rrt_star_cbf", "fig")
OUT_DIR = os.path.join(ROOT, "fig")
DEFAULT_MAT = os.path.join(os.path.dirname(ROOT), "rrt_star_cbf", "example.mat")

# Values printed in rrt_star_cbf/README.md for the MATLAB implementation.
MATLAB_REFERENCE = {"base": (3.743, 0.831), "ours": (4.658, 0.326)}

# Pixel geometry of the MATLAB figures (560x420 PNGs, axis box detected from the
# dark-gray axis lines): x = 1..100 spans columns 118..461, y = 1..100 rows 374..31.
MAP_AXES = {"col0": 118, "col1": 461, "row0": 374, "row1": 31}

START = (10.0, 10.0)
GOAL = (90.0, 90.0)


def obstacle_distance_profile(path, distance_map, n=100):
    """Port of computeObstacleDistanceProfile from example.m."""
    if path is None or len(path) < 2:
        return np.full(n, np.nan)
    seg = np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1))
    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    total = cumulative[-1]
    if total <= 0:
        return np.full(n, np.nan)
    target = total * np.linspace(0.0, 1.0, n)
    xs = np.interp(target, cumulative, path[:, 0])
    ys = np.interp(target, cumulative, path[:, 1])
    # griddedInterpolant(..., 'linear', 'nearest') on 1-based pixel coordinates
    prof = ndimage.map_coordinates(distance_map, [ys - 1.0, xs - 1.0], order=1, mode="nearest")
    return np.maximum(0.0, prof)


def run_trials(label, params_kw, n_trials, occ, h, dhdx, dhdy, distance_map):
    stats = {k: [] for k in ("length", "mean_h", "rejection", "iterations", "distance", "runtime")}
    paths, profiles = [], []
    for i in range(1, n_trials + 1):
        params = scr.Params(seed=i, **params_kw)
        res = scr.plan(list(START), list(GOAL), occ, h, dhdx, dhdy, params)
        path = res["path"] if res["found"] else None
        prof = obstacle_distance_profile(path, distance_map)
        paths.append(path)
        profiles.append(prof)
        stats["length"].append(res["path_length"])
        stats["mean_h"].append(res["mean_h_path"])
        stats["rejection"].append(res["rejection_ratio"])
        stats["iterations"].append(res["iterations"])
        stats["distance"].append(np.mean(prof))
        stats["runtime"].append(res["runtime_ms"])
        if not res["found"]:
            print(f"[{label}] trial {i}: goal not reached in {res['iterations']} iterations")
    return paths, np.array(profiles).T, {k: np.array(v, dtype=float) for k, v in stats.items()}


def mean_std(values):
    v = values[np.isfinite(values)]
    if len(v) == 0:
        return float("nan"), float("nan")
    return float(np.mean(v)), float(np.std(v, ddof=1)) if len(v) > 1 else 0.0


def plot_paths(ax, occ, paths, title):
    """Same style as the MATLAB figure: imagesc(~occupancy), gray colormap, red paths."""
    ny, nx = occ.shape
    ax.imshow(1.0 - occ, cmap="gray", origin="lower", interpolation="nearest",
              extent=[0.5, nx + 0.5, 0.5, ny + 0.5], vmin=0, vmax=1)
    for path in paths:
        if path is not None:
            ax.plot(path[:, 0], path[:, 1], "r-", linewidth=0.8)
    ax.plot(*START, "o", color="g", markerfacecolor="g", markersize=5)
    ax.plot(*GOAL, "o", color="b", markerfacecolor="b", markersize=5)
    ax.set_xlim(1, nx)
    ax.set_ylim(1, ny)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title, fontweight="bold")


def save_paths_figure(occ, paths, title, filename):
    fig, ax = plt.subplots(figsize=(5.6, 4.2), dpi=100)
    plot_paths(ax, occ, paths, title)
    fig.savefig(os.path.join(OUT_DIR, filename))
    plt.close(fig)


def save_side_by_side(matlab_png, panels, filename):
    """MATLAB figure followed by one panel per (occupancy, paths, title) entry."""
    fig, axes = plt.subplots(1, 1 + len(panels), figsize=(6 * (1 + len(panels)), 5), dpi=100)
    if os.path.exists(matlab_png):
        axes[0].imshow(mpimg.imread(matlab_png))
    axes[0].set_title("MATLAB (" + os.path.basename(matlab_png) + ")")
    axes[0].axis("off")
    for ax, (occ, paths, title) in zip(axes[1:], panels):
        plot_paths(ax, occ, paths, title)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, filename))
    plt.close(fig)


def matlab_occupancy_from_png(png):
    """Sample the MATLAB map figure at every grid cell centre.

    Returns (occupancy, valid): occupancy is True on black pixels, valid masks
    out cells hidden under red paths or markers.
    """
    im = np.round(mpimg.imread(png)[..., :3] * 255).astype(int)
    a = MAP_AXES
    xs = np.arange(1, 101)
    ys = np.arange(1, 101)
    cols = np.round(a["col0"] + (xs - 1) * (a["col1"] - a["col0"]) / 99.0).astype(int)
    rows = np.round(a["row0"] + (ys - 1) * (a["row1"] - a["row0"]) / 99.0).astype(int)
    px = im[rows[:, None], cols[None, :]]
    black = np.all(px < 60, axis=2)
    white = np.all(px > 190, axis=2)
    return black, black | white


def save_occupancy_comparison(occ, filename):
    occ_cpp = occ > 0.5
    masks = []
    for name in ("base.png", "ours.png"):
        png = os.path.join(MATLAB_FIG_DIR, name)
        if os.path.exists(png):
            masks.append(matlab_occupancy_from_png(png))
    if not masks:
        print("MATLAB figures not found, skipping occupancy comparison")
        return None
    # Combine both MATLAB figures so paths drawn on one are filled in by the other.
    occ_m = np.zeros_like(occ_cpp)
    valid = np.zeros_like(occ_cpp)
    for black, ok in masks:
        occ_m = np.where(~valid & ok, black, occ_m)
        valid |= ok
    interior = np.zeros_like(valid)
    interior[1:-1, 1:-1] = True
    cmp_mask = valid & interior

    agree = np.mean(occ_cpp[cmp_mask] == occ_m[cmp_mask])
    inter = np.sum(occ_cpp & occ_m & cmp_mask)
    union = np.sum((occ_cpp | occ_m) & cmp_mask)
    iou = inter / union if union else float("nan")

    rgb = np.ones(occ.shape + (3,))
    rgb[occ_cpp & occ_m] = (0, 0, 0)
    rgb[occ_m & ~occ_cpp & cmp_mask] = (0.85, 0.1, 0.1)
    rgb[occ_cpp & ~occ_m & cmp_mask] = (0.1, 0.3, 0.9)
    rgb[~cmp_mask] = (0.8, 0.8, 0.8)

    fig, ax = plt.subplots(figsize=(6.5, 5.5), dpi=100)
    ax.imshow(rgb, origin="lower", extent=[0.5, 100.5, 0.5, 100.5], interpolation="nearest")
    ax.set_xlim(1, 100)
    ax.set_ylim(1, 100)
    ax.set_title(f"Occupancy: black=both, red=MATLAB only, blue=C++ only\n"
                 f"cell agreement {agree * 100:.2f}%, obstacle IoU {iou:.3f}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    fig.savefig(os.path.join(OUT_DIR, filename))
    plt.close(fig)
    return agree, iou


def digitize_matlab_distance(png):
    """Read the two curves (blue = Ours, orange = Base) out of fig/distance.png."""
    im = np.round(mpimg.imread(png)[..., :3] * 255).astype(int)
    axis = np.all(im == 38, axis=2)
    row0 = int(np.argmax(axis.sum(1)))           # x axis line (y = 0)
    col0 = int(np.argmax(axis.sum(0)))           # y axis line (x = 0)
    col1 = int(np.where(axis[row0])[0].max())    # x = 100
    row1 = int(np.where(axis[:, col0])[0].min())  # y = 12 (top of the MATLAB axes)
    # The legend box is outlined in the same dark gray inside the axes.
    inner = axis.copy()
    inner[row0 - 5:, :] = False
    inner[:, :col0 + 5] = False
    inner[:row1 + 5, :] = False
    lr, lc = np.where(inner)
    legend = np.zeros(axis.shape, dtype=bool)
    if len(lr):
        legend[lr.min():lr.max() + 1, lc.min():lc.max() + 1] = True
    curves = {}
    for name, color in (("ours", (0, 114, 189)), ("base", (217, 83, 25))):
        mask = np.all(np.abs(im - np.array(color)) < 30, axis=2) & ~legend
        xs, ys = [], []
        for col in range(col0, col1 + 1):
            rows = np.where(mask[:, col])[0]
            if len(rows):
                xs.append((col - col0) / (col1 - col0) * 100.0)
                ys.append((row0 - rows.mean()) / (row0 - row1) * 12.0)
        curves[name] = (np.array(xs), np.array(ys))
    return curves


def save_distance_figures(progress, prof_base, prof_ours):
    mean_base = np.nanmean(prof_base, axis=1)
    mean_ours = np.nanmean(prof_ours, axis=1)

    fig, ax = plt.subplots(figsize=(5.6, 4.2), dpi=100)
    ax.plot(progress, mean_ours, linewidth=1.5, color="#0072BD", label="Ours")
    ax.plot(progress, mean_base, linewidth=1.5, color="#D95319", label="Base")
    ax.set_xlim(0, 100)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.4)
    ax.legend(loc="upper left")
    ax.set_xlabel("Path progress [%]")
    ax.set_ylabel("Distance to closest obstacle")
    ax.set_title("Average distance to closest obstacle", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "distance_cpp.png"))
    plt.close(fig)

    png = os.path.join(MATLAB_FIG_DIR, "distance.png")
    if not os.path.exists(png):
        return
    curves = digitize_matlab_distance(png)
    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=100)
    ax.plot(progress, mean_ours, linewidth=1.8, color="#0072BD", label="Ours (C++)")
    ax.plot(progress, mean_base, linewidth=1.8, color="#D95319", label="Base (C++)")
    ax.plot(*curves["ours"], "--", linewidth=1.2, color="#0072BD", label="Ours (MATLAB, digitized)")
    ax.plot(*curves["base"], "--", linewidth=1.2, color="#D95319", label="Base (MATLAB, digitized)")
    ax.set_xlim(0, 100)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.4)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_xlabel("Path progress [%]")
    ax.set_ylabel("Distance to closest obstacle")
    ax.set_title("Average distance to closest obstacle: C++ vs MATLAB", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "compare_distance.png"))
    plt.close(fig)


def load_matlab_fields(mat_path):
    """h, dhdx, dhdy saved from example.m (`save example.mat h dhdx dhdy`)."""
    data = loadmat(mat_path)
    missing = [k for k in ("h", "dhdx", "dhdy") if k not in data]
    if missing:
        raise KeyError(f"{mat_path} is missing variables: {', '.join(missing)}")
    return tuple(np.nan_to_num(np.asarray(data[k], dtype=float)) for k in ("h", "dhdx", "dhdy"))


def compare_fields(cpp, mat):
    """Numeric agreement between the C++ and MATLAB safety fields."""
    h_c, dx_c, dy_c = cpp
    h_m, dx_m, dy_m = mat
    occ_c, occ_m = h_c <= 0, h_m <= 0
    free = ~occ_c & ~occ_m
    grad_c, grad_m = np.hypot(dx_c, dy_c), np.hypot(dx_m, dy_m)
    return {
        "h_max": (h_m.max(), h_c.max()),
        "occupancy_diff_cells": int(np.sum(occ_c != occ_m)),
        "h_rel_err_median": float(np.median(np.abs(h_c[free] - h_m[free]) / h_m[free])),
        "h_rel_err_p95": float(np.percentile(np.abs(h_c[free] - h_m[free]) / h_m[free], 95)),
        "grad_rel_err_median": float(np.median(np.abs(grad_c[free] - grad_m[free]) / grad_m[free].clip(1e-12))),
        "grad_angle_err_median_deg": float(np.degrees(np.median(np.abs(
            np.angle(np.exp(1j * (np.arctan2(dy_c, dx_c) - np.arctan2(dy_m, dx_m))))[free])))),
    }


def save_field_comparison(cpp, mat, filename):
    h_c, dx_c, dy_c = cpp
    h_m, dx_m, dy_m = mat
    ny, nx = h_c.shape
    ext = [0.5, nx + 0.5, 0.5, ny + 0.5]
    occ = (h_c <= 0) | (h_m <= 0)
    grad_c, grad_m = np.hypot(dx_c, dy_c), np.hypot(dx_m, dy_m)
    rows = [
        ("h", h_m, h_c),
        ("|grad h|", grad_m, grad_c),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(16, 9.5), dpi=100)
    for r, (name, m, c) in enumerate(rows):
        vmax = max(m.max(), c.max())
        for col, (label, field) in enumerate((("MATLAB (example.mat)", m), ("C++", c))):
            im = axes[r, col].imshow(np.where(occ, np.nan, field), origin="lower", extent=ext,
                                     cmap="viridis", vmin=0, vmax=vmax)
            axes[r, col].set_title(f"{name}: {label}")
            fig.colorbar(im, ax=axes[r, col], fraction=0.046)
        rel = np.where(occ, np.nan, (c - m) / np.where(m > 0, m, np.nan))
        im = axes[r, 2].imshow(rel, origin="lower", extent=ext, cmap="RdBu_r", vmin=-0.3, vmax=0.3)
        axes[r, 2].set_title(f"{name}: (C++ - MATLAB) / MATLAB")
        fig.colorbar(im, ax=axes[r, 2], fraction=0.046)
    for ax in axes.flat:
        ax.set_xlim(1, nx)
        ax.set_ylim(1, ny)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, filename))
    plt.close(fig)


def save_distance_comparison_mat(progress, profiles, filename):
    """Per variant: C++ planner on C++ fields, on MATLAB fields, and the digitized MATLAB curve."""
    png = os.path.join(MATLAB_FIG_DIR, "distance.png")
    curves = digitize_matlab_distance(png) if os.path.exists(png) else {}
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8), dpi=100)
    titles = {"base": "Without CBF (base)", "ours": "With CBF (ours)"}
    colors = {"base": "#D95319", "ours": "#0072BD"}
    for ax, variant in zip(axes, ("base", "ours")):
        ax.plot(progress, np.nanmean(profiles[variant]["cpp"], axis=1), "-", linewidth=1.8,
                color=colors[variant], label="C++ planner, C++ fields")
        ax.plot(progress, np.nanmean(profiles[variant]["mat"], axis=1), "-", linewidth=1.8,
                color="k", label="C++ planner, MATLAB fields (example.mat)")
        if variant in curves:
            ax.plot(*curves[variant], "--", linewidth=1.4, color="gray", label="MATLAB (digitized distance.png)")
        ax.set_xlim(0, 100)
        ax.set_ylim(bottom=0)
        ax.grid(True, alpha=0.4)
        ax.legend(loc="upper left", fontsize=8)
        ax.set_xlabel("Path progress [%]")
        ax.set_title(titles[variant], fontweight="bold")
    axes[0].set_ylabel("Distance to closest obstacle")
    fig.suptitle("Average distance to closest obstacle", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, filename))
    plt.close(fig)


def left_route_count(paths):
    """Paths passing between the left wall and the obstacle cluster at y ~ 60 (MATLAB's CBF route)."""
    return sum(p is not None and np.any((p[:, 1] > 55) & (p[:, 1] < 70) & (p[:, 0] < 12)) for p in paths)


def save_h_field(h, dhdx, dhdy, occ):
    ny, nx = h.shape
    fig, ax = plt.subplots(figsize=(6.5, 5.5), dpi=100)
    im = ax.imshow(np.where(occ > 0.5, np.nan, h), origin="lower", cmap="viridis",
                   extent=[0.5, nx + 0.5, 0.5, ny + 0.5])
    step = 4
    yy, xx = np.mgrid[1:ny + 1:step, 1:nx + 1:step]
    gx = dhdx[::step, ::step]
    gy = dhdy[::step, ::step]
    norm = np.hypot(gx, gy)
    norm[norm == 0] = 1.0
    ax.quiver(xx, yy, gx / norm, gy / norm, color="w", scale=40, width=0.002)
    fig.colorbar(im, ax=ax, label="h")
    ax.set_xlim(1, nx)
    ax.set_ylim(1, ny)
    ax.set_title("Poisson safety function h and grad h direction")
    fig.savefig(os.path.join(OUT_DIR, "h_field.png"))
    plt.close(fig)


def print_stats(name, stats, reference):
    def fmt(key, digits=3):
        m, s = mean_std(stats[key])
        return f"{m:.{digits}f} +/- {s:.{digits}f}"

    print(f"\n{name}")
    print(f"  path length              : {fmt('length')}")
    print(f"  mean h along path        : {fmt('mean_h', 4)}")
    print(f"  rejection ratio          : {fmt('rejection')}")
    print(f"  iterations               : {fmt('iterations', 1)}")
    print(f"  runtime per plan [ms]    : {fmt('runtime', 1)}")
    print(f"  distance to obstacle     : {fmt('distance')}"
          f"   (MATLAB README: {reference[0]:.3f} +/- {reference[1]:.3f})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-N", "--trials", type=int, default=10)
    parser.add_argument("--max-iter", type=int, default=10000)
    parser.add_argument("--fine", type=int, default=4, help="Poisson solver refinement factor")
    parser.add_argument("--mat", default=DEFAULT_MAT,
                        help="MATLAB h/dhdx/dhdy export (default: %(default)s); pass '' to skip")
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)

    circles = scr.generate_obstacles(2, 40, 0.06)
    field_sets = {"cpp": scr.generate_safety_function(circles, 100, 100, args.fine)}
    if args.mat:
        if os.path.exists(args.mat):
            field_sets["mat"] = load_matlab_fields(args.mat)
        else:
            print(f"{args.mat} not found, running with C++ fields only")

    common = dict(maxIter=args.max_iter, v=1.0, stepSize=1.0, kappa=50.0)
    variants = {
        "base": ("without CBF", dict(common, c=1.0, do_cbf=False)),
        "ours": ("with CBF", dict(common, c=0.0, do_cbf=True)),
    }
    occs, paths, profiles, stats = {}, {}, {}, {}
    for src, (h, dhdx, dhdy) in field_sets.items():
        occ = (h <= 0).astype(float)
        # bwdist(occupancy): distance from every cell to the nearest occupied cell
        distance_map = ndimage.distance_transform_edt(occ < 0.5)
        occs[src] = occ
        for variant, (label, params_kw) in variants.items():
            p, prof, st = run_trials(f"{label}, {src} fields", params_kw, args.trials,
                                     occ, h, dhdx, dhdy, distance_map)
            paths.setdefault(variant, {})[src] = p
            profiles.setdefault(variant, {})[src] = prof
            stats.setdefault(variant, {})[src] = st

    titles = {"base": "RRT* without CBF safety", "ours": "RRT* with CBF safety"}
    progress = np.linspace(0, 100, 100)
    for variant in variants:
        save_paths_figure(occs["cpp"], paths[variant]["cpp"], titles[variant], f"{variant}_cpp.png")
        panels = [(occs["cpp"], paths[variant]["cpp"], titles[variant] + "\n(C++ planner, C++ fields)")]
        if "mat" in field_sets:
            save_paths_figure(occs["mat"], paths[variant]["mat"], titles[variant], f"{variant}_mat.png")
            panels.append((occs["mat"], paths[variant]["mat"],
                           titles[variant] + "\n(C++ planner, MATLAB fields)"))
        save_side_by_side(os.path.join(MATLAB_FIG_DIR, f"{variant}.png"), panels, f"compare_{variant}.png")
    save_distance_figures(progress, profiles["base"]["cpp"], profiles["ours"]["cpp"])
    occ_cmp = save_occupancy_comparison(occs["cpp"], "compare_occupancy.png")
    save_h_field(*field_sets["cpp"], occs["cpp"])

    h = field_sets["cpp"][0]
    print(f"\nh: max {h.max():.5f}, occupied cells {int(occs['cpp'].sum())}/{h.size}")
    if occ_cmp is not None:
        print(f"occupancy vs MATLAB figure: agreement {occ_cmp[0] * 100:.2f}%, IoU {occ_cmp[1]:.3f}")
    if "mat" in field_sets:
        save_field_comparison(field_sets["cpp"], field_sets["mat"], "compare_fields.png")
        save_distance_comparison_mat(progress, profiles, "compare_distance_mat.png")
        fc = compare_fields(field_sets["cpp"], field_sets["mat"])
        print(f"C++ vs MATLAB fields ({os.path.basename(args.mat)}):")
        print(f"  h max                    : MATLAB {fc['h_max'][0]:.5f}, C++ {fc['h_max'][1]:.5f}")
        print(f"  occupancy cells differing: {fc['occupancy_diff_cells']}")
        print(f"  h relative error         : median {fc['h_rel_err_median'] * 100:.1f}%, "
              f"95th pct {fc['h_rel_err_p95'] * 100:.1f}%")
        print(f"  |grad h| relative error  : median {fc['grad_rel_err_median'] * 100:.1f}%")
        print(f"  grad h direction error   : median {fc['grad_angle_err_median_deg']:.2f} deg")

    names = {"cpp": "C++ fields", "mat": "MATLAB fields"}
    for variant, (label, _) in variants.items():
        for src in field_sets:
            print_stats(f"{label[0].upper()}{label[1:]} ({variant}), {names[src]}",
                        stats[variant][src], MATLAB_REFERENCE[variant])
            print(f"  left-wall route          : {left_route_count(paths[variant][src])}/{args.trials}")
    print(f"\nFigures written to {OUT_DIR}")


if __name__ == "__main__":
    main()
