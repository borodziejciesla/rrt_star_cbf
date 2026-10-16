#pragma once

#include <cstdint>
#include <random>
#include <vector>

namespace safe_cbf_rrt {

/// Reproduces MATLAB `rng(seed, 'twister'); rand` (and numpy
/// `RandomState(seed).rand()`): MT19937 seeded with init_genrand, doubles
/// built with genrand_res53.
class MatlabRand {
 public:
  explicit MatlabRand(std::uint32_t seed) : gen_(seed) {}

  double operator()() {
    const std::uint32_t a = gen_() >> 5;
    const std::uint32_t b = gen_() >> 6;
    return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
  }

 private:
  std::mt19937 gen_;
};

struct Circle {
  double cx;
  double cy;
  double r;
};

/// Same as example.m:
///   rng(seed); for i = 1:n, objects{i} = struct('c', rand(2,1), 'r', r); end
inline std::vector<Circle> generateObstacles(std::uint32_t seed, int n, double r) {
  MatlabRand rand(seed);
  std::vector<Circle> circles;
  circles.reserve(n);
  for (int i = 0; i < n; ++i) {
    const double cx = rand();
    const double cy = rand();
    circles.push_back({cx, cy, r});
  }
  return circles;
}

}  // namespace safe_cbf_rrt
