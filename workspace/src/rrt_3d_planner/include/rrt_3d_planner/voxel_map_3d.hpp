#ifndef RRT_3D_PLANNER__VOXEL_MAP_3D_HPP_
#define RRT_3D_PLANNER__VOXEL_MAP_3D_HPP_

#include <unordered_set>

#include "rrt_3d_planner/types.hpp"

namespace rrt_3d_planner {

class VoxelMap3D {
public:
  VoxelMap3D(double voxel_size, double origin_x, double origin_y, double origin_z);

  VoxelKey worldToVoxel(double x, double y, double z) const;
  Point3D voxelToWorld(int ix, int iy, int iz) const;

  void clear();
  void insertPoint(double x, double y, double z);

  bool isOccupied(int ix, int iy, int iz) const;
  bool isOccupiedWorld(double x, double y, double z) const;

  double voxelSize() const;
  const std::unordered_set<VoxelKey, VoxelKeyHash> & occupiedVoxels() const;

private:
  double voxel_size_;
  double origin_x_;
  double origin_y_;
  double origin_z_;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels_;
};

}  // namespace rrt_3d_planner

#endif  // RRT_3D_PLANNER__VOXEL_MAP_3D_HPP_
