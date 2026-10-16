#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "safe_cbf_rrt/grid.hpp"

namespace safe_cbf_rrt {

using Point = std::array<double, 2>;  // [x, y] in 1-based pixel coordinates

struct Params {
  double c = 0.5;              ///< weight between length and safety (do_cbf only)
  int maxIter = 10000;
  double stepSize = 2.0;
  double neighborRadius = 10.0;
  int samplesPerEdge = 20;     ///< samples for collision & CBF checks
  double kappa = 5.0;          ///< CBF class-K gain
  double v = 0.0;              ///< speed used in the CBF derivative term
  bool doCbf = true;
  double goalBias = 0.05;
  std::uint32_t seed = 0;
};

struct Result {
  std::vector<Point> path;     ///< start -> goal, empty if not found
  double pathLength = 0.0;
  double meanHPath = 0.0;      ///< length-weighted mean h, NaN if no path
  double rejectionRatio = 0.0;
  int iterations = 0;
  int numNodes = 0;
  bool found = false;
  double runtimeMs = 0.0;
};

/// Port of SafeCBFRRTStar.m. Grids are Ny x Nx (row = y); occupancy uses
/// 1 = occupied. dhdx/dhdy are the gradient grids of h.
Result safeCbfRrtStar(const Point& start, const Point& goal, const Grid2D& occupancy,
                      const Grid2D& h, const Grid2D& dhdx, const Grid2D& dhdy,
                      const Params& params);

}  // namespace safe_cbf_rrt
