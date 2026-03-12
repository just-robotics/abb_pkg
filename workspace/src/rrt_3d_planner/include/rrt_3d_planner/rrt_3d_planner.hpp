#ifndef RRT_3D_PLANNER__RRT_3D_PLANNER_HPP_
#define RRT_3D_PLANNER__RRT_3D_PLANNER_HPP_

#include <random>
#include <vector>

#include "rrt_3d_planner/types.hpp"
#include "rrt_3d_planner/voxel_map_3d.hpp"

namespace rrt_3d_planner {

struct Bounds3D {
  double x_min{-2.0};
  double x_max{2.0};
  double y_min{-2.0};
  double y_max{2.0};
  double z_min{0.0};
  double z_max{2.0};
};

struct PlannerParams {
  double step_size{0.2};
  double goal_bias{0.1};
  double goal_tolerance{0.2};
  int max_iterations{3000};
  double collision_step{0.05};
  double robot_radius{0.1};
  bool enable_smoothing{true};
  int smoothing_iterations{150};
};

class RRT3DPlanner {
public:
  RRT3DPlanner(const VoxelMap3D & map, const PlannerParams & params, const Bounds3D & bounds);

  bool plan(const Point3D & start, const Point3D & goal, std::vector<Point3D> & path_out);

  const std::vector<Node3D> & tree() const;

private:
  double distance(const Point3D & a, const Point3D & b) const;
  bool inBounds(const Point3D & p) const;
  Point3D sampleRandom(const Point3D & goal);
  int nearestIndex(const Point3D & sample) const;
  Point3D steer(const Point3D & from, const Point3D & to) const;
  bool isStateValid(const Point3D & p) const;
  bool isSegmentValid(const Point3D & a, const Point3D & b) const;
  std::vector<Point3D> extractPath(int last_index) const;
  void smoothPath(std::vector<Point3D> & path);

  const VoxelMap3D & map_;
  PlannerParams params_;
  Bounds3D bounds_;

  std::vector<Node3D> tree_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist01_;
};

}  // namespace rrt_3d_planner

#endif  // RRT_3D_PLANNER__RRT_3D_PLANNER_HPP_
