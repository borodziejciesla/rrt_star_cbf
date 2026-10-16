#include "safe_cbf_rrt/poisson_safety.hpp"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <stdexcept>

namespace safe_cbf_rrt {

namespace {

bool insideAnyCircle(double x, double y, const std::vector<Circle>& circles) {
  for (const auto& c : circles) {
    if (std::hypot(x - c.cx, y - c.cy) <= c.r) return true;
  }
  return false;
}

}  // namespace

SafetyFunction generateSafetyFunction(const std::vector<Circle>& circles,
                                      const PoissonOptions& opts) {
  if (opts.nx < 2 || opts.ny < 2 || opts.fine < 1) {
    throw std::invalid_argument("generateSafetyFunction: invalid grid options");
  }

  // Fine grid whose nodes include every output grid node.
  const int mx = (opts.nx - 1) * opts.fine + 1;
  const int my = (opts.ny - 1) * opts.fine + 1;
  const double dx = 1.0 / (mx - 1);
  const double dy = 1.0 / (my - 1);

  std::vector<int> index(static_cast<std::size_t>(mx) * my, -1);
  int n_unknowns = 0;
  for (int r = 1; r < my - 1; ++r) {
    for (int c = 1; c < mx - 1; ++c) {
      if (!insideAnyCircle(c * dx, r * dy, circles)) {
        index[static_cast<std::size_t>(r) * mx + c] = n_unknowns++;
      }
    }
  }

  // -Laplace(h) = source with homogeneous Dirichlet values on masked nodes.
  const double ax = 1.0 / (dx * dx);
  const double ay = 1.0 / (dy * dy);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(n_unknowns) * 5);
  Eigen::VectorXd rhs = Eigen::VectorXd::Constant(n_unknowns, opts.source);
  for (int r = 1; r < my - 1; ++r) {
    for (int c = 1; c < mx - 1; ++c) {
      const int i = index[static_cast<std::size_t>(r) * mx + c];
      if (i < 0) continue;
      triplets.emplace_back(i, i, 2.0 * ax + 2.0 * ay);
      const int nbr[4][3] = {{r, c - 1, 0}, {r, c + 1, 0}, {r - 1, c, 1}, {r + 1, c, 1}};
      for (const auto& n : nbr) {
        const int j = index[static_cast<std::size_t>(n[0]) * mx + n[1]];
        if (j >= 0) triplets.emplace_back(i, j, n[2] == 0 ? -ax : -ay);
      }
    }
  }
  Eigen::SparseMatrix<double> A(n_unknowns, n_unknowns);
  A.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(A);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("generateSafetyFunction: factorization failed");
  }
  const Eigen::VectorXd sol = solver.solve(rhs);

  // Sample onto the output grid (nodes coincide with fine-grid nodes).
  const std::vector<double> xq = linspace(0.0, 1.0, opts.nx);
  const std::vector<double> yq = linspace(0.0, 1.0, opts.ny);
  SafetyFunction out;
  out.h = Grid2D(opts.ny, opts.nx, 0.0);
  for (int r = 0; r < opts.ny; ++r) {
    for (int c = 0; c < opts.nx; ++c) {
      const int i = index[static_cast<std::size_t>(r * opts.fine) * mx + c * opts.fine];
      out.h.at(r, c) = i >= 0 ? sol[i] : 0.0;
    }
  }

  matlabGradient(out.h, xq, yq, out.dhdx, out.dhdy);
  return out;
}

Grid2D occupancyFromH(const Grid2D& h) {
  Grid2D occ(h.ny, h.nx, 0.0);
  for (std::size_t i = 0; i < h.data.size(); ++i) occ.data[i] = h.data[i] <= 0.0 ? 1.0 : 0.0;
  return occ;
}

}  // namespace safe_cbf_rrt
