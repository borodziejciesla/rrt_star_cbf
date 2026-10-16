#pragma once

#include <vector>

#include "safe_cbf_rrt/grid.hpp"
#include "safe_cbf_rrt/matlab_rng.hpp"

namespace safe_cbf_rrt {

struct SafetyFunction {
  Grid2D h;     ///< Ny x Nx safety values (h <= 0 is occupied)
  Grid2D dhdx;  ///< dh/dx in physical units (domain [0,1]^2)
  Grid2D dhdy;  ///< dh/dy in physical units
};

struct PoissonOptions {
  int nx = 100;               ///< output grid columns
  int ny = 100;               ///< output grid rows
  int fine = 4;               ///< solver refinement factor per output cell
  double source = 0.141421356237309515;  ///< |u| = sqrt(0.1^2 + 0.1^2)
};

/// Solve Laplace(h) = -source on [0,1]^2 minus the circles with h = 0 on the
/// outer box and inside the circles (5-point finite differences), resample to
/// an nx x ny grid, then compute the gradient like MATLAB `gradient(h, xq, yq)`.
/// Grid points inside a circle hold h = 0 rather than NaN: griddata in
/// GeneratePoissonSafetyFunction.m triangulates the convex hull of the FEM nodes,
/// so it interpolates the zero boundary values across obstacle holes and the
/// gradient stays non-zero next to obstacles.
SafetyFunction generateSafetyFunction(const std::vector<Circle>& circles,
                                      const PoissonOptions& opts = {});

/// occupancy = h <= 0 (1.0 occupied, 0.0 free).
Grid2D occupancyFromH(const Grid2D& h);

}  // namespace safe_cbf_rrt
