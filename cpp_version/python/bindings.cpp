#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "safe_cbf_rrt/matlab_rng.hpp"
#include "safe_cbf_rrt/planner.hpp"
#include "safe_cbf_rrt/poisson_safety.hpp"

namespace py = pybind11;
using namespace safe_cbf_rrt;

namespace {

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

Grid2D toGrid(const DoubleArray& arr) {
  if (arr.ndim() != 2) throw std::invalid_argument("expected a 2-D array (Ny x Nx)");
  Grid2D g(static_cast<int>(arr.shape(0)), static_cast<int>(arr.shape(1)));
  std::memcpy(g.data.data(), arr.data(), g.data.size() * sizeof(double));
  return g;
}

DoubleArray toArray(const Grid2D& g) {
  DoubleArray arr({g.ny, g.nx});
  std::memcpy(arr.mutable_data(), g.data.data(), g.data.size() * sizeof(double));
  return arr;
}

Point toPoint(const std::vector<double>& v) {
  if (v.size() != 2) throw std::invalid_argument("expected a point [x, y]");
  return {v[0], v[1]};
}

}  // namespace

PYBIND11_MODULE(safe_cbf_rrt_py, m) {
  m.doc() = "Safe CBF-RRT* (C++ port of the MATLAB implementation)";

  m.def(
      "matlab_rand",
      [](std::uint32_t seed, int n) {
        MatlabRand rand(seed);
        std::vector<double> out(n);
        for (double& v : out) v = rand();
        return out;
      },
      py::arg("seed"), py::arg("n"), "First n values of MATLAB rng(seed); rand");

  m.def(
      "generate_obstacles",
      [](std::uint32_t seed, int n, double r) {
        DoubleArray arr({n, 3});
        auto a = arr.mutable_unchecked<2>();
        const auto circles = generateObstacles(seed, n, r);
        for (int i = 0; i < n; ++i) {
          a(i, 0) = circles[i].cx;
          a(i, 1) = circles[i].cy;
          a(i, 2) = circles[i].r;
        }
        return arr;
      },
      py::arg("seed") = 2, py::arg("n") = 40, py::arg("r") = 0.06,
      "Circles as an (n, 3) array [cx, cy, r] in the unit square");

  m.def(
      "generate_safety_function",
      [](const DoubleArray& circles_arr, int nx, int ny, int fine) {
        if (circles_arr.ndim() != 2 || (circles_arr.shape(0) > 0 && circles_arr.shape(1) != 3)) {
          throw std::invalid_argument("circles must be an (n, 3) array [cx, cy, r]");
        }
        auto c = circles_arr.unchecked<2>();
        std::vector<Circle> circles;
        for (py::ssize_t i = 0; i < c.shape(0); ++i) circles.push_back({c(i, 0), c(i, 1), c(i, 2)});
        PoissonOptions opts;
        opts.nx = nx;
        opts.ny = ny;
        opts.fine = fine;
        SafetyFunction sf;
        {
          py::gil_scoped_release release;
          sf = generateSafetyFunction(circles, opts);
        }
        return py::make_tuple(toArray(sf.h), toArray(sf.dhdx), toArray(sf.dhdy));
      },
      py::arg("circles"), py::arg("nx") = 100, py::arg("ny") = 100, py::arg("fine") = 4,
      "Returns (h, dhdx, dhdy) as Ny x Nx arrays");

  py::class_<Params>(m, "Params")
      .def(py::init<>())
      .def(py::init([](py::kwargs kw) {
        Params p;
        py::object obj = py::cast(&p, py::return_value_policy::reference);
        for (auto item : kw) py::setattr(obj, item.first, item.second);
        return p;
      }))
      .def_readwrite("c", &Params::c)
      .def_readwrite("maxIter", &Params::maxIter)
      .def_readwrite("stepSize", &Params::stepSize)
      .def_readwrite("neighborRadius", &Params::neighborRadius)
      .def_readwrite("samplesPerEdge", &Params::samplesPerEdge)
      .def_readwrite("kappa", &Params::kappa)
      .def_readwrite("v", &Params::v)
      .def_readwrite("do_cbf", &Params::doCbf)
      .def_readwrite("goalBias", &Params::goalBias)
      .def_readwrite("seed", &Params::seed)
      .def("__repr__", [](const Params& p) {
        return "Params(c=" + std::to_string(p.c) + ", maxIter=" + std::to_string(p.maxIter) +
               ", stepSize=" + std::to_string(p.stepSize) +
               ", neighborRadius=" + std::to_string(p.neighborRadius) +
               ", samplesPerEdge=" + std::to_string(p.samplesPerEdge) +
               ", kappa=" + std::to_string(p.kappa) + ", v=" + std::to_string(p.v) +
               ", do_cbf=" + (p.doCbf ? "True" : "False") + ", seed=" + std::to_string(p.seed) +
               ")";
      });

  m.def(
      "plan",
      [](const std::vector<double>& start, const std::vector<double>& goal,
         const DoubleArray& occ, const DoubleArray& h, const DoubleArray& dhdx,
         const DoubleArray& dhdy, const Params& params) {
        const Grid2D g_occ = toGrid(occ), g_h = toGrid(h), g_dx = toGrid(dhdx), g_dy = toGrid(dhdy);
        Result r;
        {
          py::gil_scoped_release release;
          r = safeCbfRrtStar(toPoint(start), toPoint(goal), g_occ, g_h, g_dx, g_dy, params);
        }
        DoubleArray path({static_cast<py::ssize_t>(r.path.size()), static_cast<py::ssize_t>(2)});
        auto p = path.mutable_unchecked<2>();
        for (std::size_t i = 0; i < r.path.size(); ++i) {
          p(i, 0) = r.path[i][0];
          p(i, 1) = r.path[i][1];
        }
        py::dict d;
        d["path"] = path;
        d["path_length"] = r.pathLength;
        d["mean_h_path"] = r.meanHPath;
        d["rejection_ratio"] = r.rejectionRatio;
        d["iterations"] = r.iterations;
        d["num_nodes"] = r.numNodes;
        d["found"] = r.found;
        d["runtime_ms"] = r.runtimeMs;
        return d;
      },
      py::arg("start"), py::arg("goal"), py::arg("occupancy"), py::arg("h"), py::arg("dhdx"),
      py::arg("dhdy"), py::arg("params"),
      "Run Safe CBF-RRT*. Points are [x, y] in 1-based pixel coordinates.");
}
