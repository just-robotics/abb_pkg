#ifndef RRT_3D_PLANNER__TYPES_HPP_
#define RRT_3D_PLANNER__TYPES_HPP_

#include <cstddef>

namespace rrt_3d_planner {

struct VoxelKey {
  int ix;
  int iy;
  int iz;

  bool operator==(const VoxelKey & other) const {
    return ix == other.ix && iy == other.iy && iz == other.iz;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey & k) const {
    std::size_t h1 = static_cast<std::size_t>(k.ix) * 73856093u;
    std::size_t h2 = static_cast<std::size_t>(k.iy) * 19349663u;
    std::size_t h3 = static_cast<std::size_t>(k.iz) * 83492791u;
    return h1 ^ h2 ^ h3;
  }
};

struct Point3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Node3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  int parent_index{-1};
  double cost{0.0};
};

}  // namespace rrt_3d_planner

#endif  // RRT_3D_PLANNER__TYPES_HPP_
