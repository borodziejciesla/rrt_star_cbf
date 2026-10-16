#include <gtest/gtest.h>

#include <cmath>

#include "safe_cbf_rrt/grid.hpp"
#include "safe_cbf_rrt/matlab_rng.hpp"
#include "safe_cbf_rrt/planner.hpp"
#include "safe_cbf_rrt/poisson_safety.hpp"

using namespace safe_cbf_rrt;

TEST(MatlabRand, MatchesNumpyRandomState) {
  // numpy.random.RandomState(2).rand(4) == MATLAB rng(2); rand(1, 4)
  MatlabRand r2(2);
  EXPECT_NEAR(r2(), 0.43599490214200376, 1e-15);
  EXPECT_NEAR(r2(), 0.025926231827891333, 1e-15);
  EXPECT_NEAR(r2(), 0.5496624778787091, 1e-15);
  EXPECT_NEAR(r2(), 0.4353223926182769, 1e-15);
  MatlabRand r1(1);
  EXPECT_NEAR(r1(), 0.417022004702574, 1e-12);
  EXPECT_NEAR(r1(), 0.7203244934421581, 1e-12);
}

TEST(Grid, BilinearMatchesLinearFunctionAndClamps) {
  Grid2D g(5, 7);
  for (int r = 0; r < g.ny; ++r)
    for (int c = 0; c < g.nx; ++c) g.at(r, c) = 2.0 * (c + 1) + 3.0 * (r + 1);
  BilinearInterpolant f(g);
  EXPECT_NEAR(f(2.5, 3.25), 2.0 * 2.5 + 3.0 * 3.25, 1e-12);
  EXPECT_NEAR(f(7.0, 5.0), 2.0 * 7 + 3.0 * 5, 1e-12);
  // nearest extrapolation
  EXPECT_NEAR(f(-3.0, 2.0), 2.0 * 1 + 3.0 * 2, 1e-12);
  EXPECT_NEAR(f(10.0, 9.0), 2.0 * 7 + 3.0 * 5, 1e-12);
}

TEST(Grid, GradientMatchesMatlab) {
  const auto xs = linspace(0.0, 1.0, 11);
  const auto ys = linspace(0.0, 2.0, 9);
  Grid2D f(9, 11);
  for (int r = 0; r < 9; ++r)
    for (int c = 0; c < 11; ++c) f.at(r, c) = xs[c] * xs[c] + 3.0 * ys[r];
  Grid2D dx, dy;
  matlabGradient(f, xs, ys, dx, dy);
  // central differences are exact for quadratics in the interior
  EXPECT_NEAR(dx.at(4, 5), 2.0 * xs[5], 1e-12);
  // one-sided at the edges: (x1^2 - x0^2) / (x1 - x0) = x0 + x1
  EXPECT_NEAR(dx.at(0, 0), xs[0] + xs[1], 1e-12);
  EXPECT_NEAR(dx.at(0, 10), xs[9] + xs[10], 1e-12);
  EXPECT_NEAR(dy.at(3, 3), 3.0, 1e-12);
}

TEST(Poisson, EmptySquareMatchesSeriesSolution) {
  // Laplace(u) = -1 on the unit square, u = 0 on the boundary: u(0.5,0.5) ~ 0.0736713
  PoissonOptions opts;
  opts.nx = 101;
  opts.ny = 101;
  opts.fine = 2;
  opts.source = 1.0;
  const auto sf = generateSafetyFunction({}, opts);
  EXPECT_NEAR(sf.h.at(50, 50), 0.0736713, 2e-5);
  EXPECT_DOUBLE_EQ(sf.h.at(0, 37), 0.0);
  EXPECT_NEAR(sf.dhdx.at(50, 50), 0.0, 1e-9);
  EXPECT_GT(sf.dhdx.at(50, 10), 0.0);
}

TEST(Poisson, CirclesAreOccupied) {
  const std::vector<Circle> circles = {{0.5, 0.5, 0.1}};
  const auto sf = generateSafetyFunction(circles);
  const auto occ = occupancyFromH(sf.h);
  EXPECT_EQ(occ.at(49, 49), 1.0);   // inside the circle
  EXPECT_EQ(occ.at(0, 50), 1.0);    // outer boundary
  EXPECT_EQ(occ.at(20, 20), 0.0);   // free space
  EXPECT_EQ(sf.dhdx.at(49, 49), 0.0);
}

TEST(Planner, FindsCollisionFreePath) {
  const auto circles = generateObstacles(2, 40, 0.06);
  const auto sf = generateSafetyFunction(circles);
  const auto occ = occupancyFromH(sf.h);
  BilinearInterpolant focc(occ);

  for (bool do_cbf : {false, true}) {
    Params p;
    p.maxIter = 10000;
    p.v = 1.0;
    p.stepSize = 1.0;
    p.kappa = 50.0;
    p.c = do_cbf ? 0.0 : 1.0;
    p.doCbf = do_cbf;
    p.seed = 1;
    const auto r = safeCbfRrtStar({10, 10}, {90, 90}, occ, sf.h, sf.dhdx, sf.dhdy, p);
    ASSERT_TRUE(r.found) << "do_cbf=" << do_cbf;
    EXPECT_DOUBLE_EQ(r.path.front()[0], 10.0);
    EXPECT_DOUBLE_EQ(r.path.back()[1], 90.0);
    EXPECT_GT(r.pathLength, std::hypot(80.0, 80.0) - 1e-9);
    for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
      const auto xs = linspace(r.path[i][0], r.path[i + 1][0], p.samplesPerEdge);
      const auto ys = linspace(r.path[i][1], r.path[i + 1][1], p.samplesPerEdge);
      for (std::size_t k = 0; k < xs.size(); ++k) ASSERT_LE(focc(xs[k], ys[k]), 0.5);
    }

    // deterministic for a fixed seed
    const auto r2 = safeCbfRrtStar({10, 10}, {90, 90}, occ, sf.h, sf.dhdx, sf.dhdy, p);
    EXPECT_EQ(r.path.size(), r2.path.size());
    EXPECT_DOUBLE_EQ(r.pathLength, r2.pathLength);
  }
}
