#include "rrt_3d_planner/voxel_map_3d.hpp"

#include <cmath>

namespace rrt_3d_planner {

VoxelMap3D::VoxelMap3D(double voxel_size, double origin_x, double origin_y, double origin_z)
: voxel_size_(voxel_size), origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z)
{
}

VoxelKey VoxelMap3D::worldToVoxel(double x, double y, double z) const
{
  return VoxelKey{
    static_cast<int>(std::floor((x - origin_x_) / voxel_size_)),
    static_cast<int>(std::floor((y - origin_y_) / voxel_size_)),
    static_cast<int>(std::floor((z - origin_z_) / voxel_size_))};
}

Point3D VoxelMap3D::voxelToWorld(int ix, int iy, int iz) const
{
  return Point3D{
    origin_x_ + (static_cast<double>(ix) + 0.5) * voxel_size_,
    origin_y_ + (static_cast<double>(iy) + 0.5) * voxel_size_,
    origin_z_ + (static_cast<double>(iz) + 0.5) * voxel_size_};
}

void VoxelMap3D::clear()
{
  occupied_voxels_.clear();
}

void VoxelMap3D::insertPoint(double x, double y, double z)
{
  occupied_voxels_.insert(worldToVoxel(x, y, z));
}

bool VoxelMap3D::isOccupied(int ix, int iy, int iz) const
{
  return occupied_voxels_.find(VoxelKey{ix, iy, iz}) != occupied_voxels_.end();
}

bool VoxelMap3D::isOccupiedWorld(double x, double y, double z) const
{
  const auto key = worldToVoxel(x, y, z);
  return isOccupied(key.ix, key.iy, key.iz);
}

double VoxelMap3D::voxelSize() const
{
  return voxel_size_;
}

const std::unordered_set<VoxelKey, VoxelKeyHash> & VoxelMap3D::occupiedVoxels() const
{
  return occupied_voxels_;
}

}  // namespace rrt_3d_planner
