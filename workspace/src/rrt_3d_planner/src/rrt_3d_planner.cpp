#include "rrt_3d_planner/rrt_3d_planner.hpp"

#include <algorithm>
#include <cmath>

namespace rrt_3d_planner {

RRT3DPlanner::RRT3DPlanner(const VoxelMap3D & map, const PlannerParams & params, const Bounds3D & bounds)
: map_(map), params_(params), bounds_(bounds), rng_(std::random_device{}()), dist01_(0.0, 1.0)
{
}

bool RRT3DPlanner::plan(const Point3D & start, const Point3D & goal, std::vector<Point3D> & path_out)
{
  path_out.clear();
  tree_.clear();

  if (!isStateValid(start) || !isStateValid(goal)) {
    return false;
  }

  tree_.push_back(Node3D{start.x, start.y, start.z, -1, 0.0});

  for (int i = 0; i < params_.max_iterations; ++i) {
    Point3D x_rand = sampleRandom(goal);
    int idx_near = nearestIndex(x_rand);
    Point3D x_near{tree_[idx_near].x, tree_[idx_near].y, tree_[idx_near].z};
    Point3D x_new = steer(x_near, x_rand);

    if (!isStateValid(x_new)) {
      continue;
    }
    if (!isSegmentValid(x_near, x_new)) {
      continue;
    }

    const double new_cost = tree_[idx_near].cost + distance(x_near, x_new);
    tree_.push_back(Node3D{x_new.x, x_new.y, x_new.z, idx_near, new_cost});
    const int idx_new = static_cast<int>(tree_.size()) - 1;

    if (distance(x_new, goal) <= params_.goal_tolerance && isSegmentValid(x_new, goal)) {
      const double goal_cost = tree_[idx_new].cost + distance(x_new, goal);
      tree_.push_back(Node3D{goal.x, goal.y, goal.z, idx_new, goal_cost});
      path_out = extractPath(static_cast<int>(tree_.size()) - 1);
      if (params_.enable_smoothing) {
        smoothPath(path_out);
      }
      return true;
    }
  }

  return false;
}

const std::vector<Node3D> & RRT3DPlanner::tree() const
{
  return tree_;
}

double RRT3DPlanner::distance(const Point3D & a, const Point3D & b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool RRT3DPlanner::inBounds(const Point3D & p) const
{
  return p.x >= bounds_.x_min && p.x <= bounds_.x_max && p.y >= bounds_.y_min && p.y <= bounds_.y_max &&
         p.z >= bounds_.z_min && p.z <= bounds_.z_max;
}

Point3D RRT3DPlanner::sampleRandom(const Point3D & goal)
{
  if (dist01_(rng_) < params_.goal_bias) {
    return goal;
  }

  std::uniform_real_distribution<double> dx(bounds_.x_min, bounds_.x_max);
  std::uniform_real_distribution<double> dy(bounds_.y_min, bounds_.y_max);
  std::uniform_real_distribution<double> dz(bounds_.z_min, bounds_.z_max);

  return Point3D{dx(rng_), dy(rng_), dz(rng_)};
}

int RRT3DPlanner::nearestIndex(const Point3D & sample) const
{
  int best_idx = 0;
  double best_dist = distance(Point3D{tree_[0].x, tree_[0].y, tree_[0].z}, sample);

  for (int i = 1; i < static_cast<int>(tree_.size()); ++i) {
    const double d = distance(Point3D{tree_[i].x, tree_[i].y, tree_[i].z}, sample);
    if (d < best_dist) {
      best_dist = d;
      best_idx = i;
    }
  }
  return best_idx;
}

Point3D RRT3DPlanner::steer(const Point3D & from, const Point3D & to) const
{
  const double d = distance(from, to);
  if (d <= params_.step_size) {
    return to;
  }

  const double scale = params_.step_size / d;
  return Point3D{from.x + (to.x - from.x) * scale, from.y + (to.y - from.y) * scale,
                 from.z + (to.z - from.z) * scale};
}

bool RRT3DPlanner::isStateValid(const Point3D & p) const
{
  if (!inBounds(p)) {
    return false;
  }

  const auto center = map_.worldToVoxel(p.x, p.y, p.z);
  const int rv = static_cast<int>(std::ceil(params_.robot_radius / map_.voxelSize()));

  for (int dx = -rv; dx <= rv; ++dx) {
    for (int dy = -rv; dy <= rv; ++dy) {
      for (int dz = -rv; dz <= rv; ++dz) {
        const double dist_v = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * map_.voxelSize();
        if (dist_v > params_.robot_radius) {
          continue;
        }
        if (map_.isOccupied(center.ix + dx, center.iy + dy, center.iz + dz)) {
          return false;
        }
      }
    }
  }

  return true;
}

bool RRT3DPlanner::isSegmentValid(const Point3D & a, const Point3D & b) const
{
  const double d = distance(a, b);
  const int steps = std::max(1, static_cast<int>(std::ceil(d / std::max(1e-6, params_.collision_step))));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    Point3D p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
    if (!isStateValid(p)) {
      return false;
    }
  }

  return true;
}

std::vector<Point3D> RRT3DPlanner::extractPath(int last_index) const
{
  std::vector<Point3D> path;
  int idx = last_index;
  while (idx >= 0) {
    const auto & n = tree_[idx];
    path.push_back(Point3D{n.x, n.y, n.z});
    idx = n.parent_index;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void RRT3DPlanner::smoothPath(std::vector<Point3D> & path)
{
  if (path.size() < 3) {
    return;
  }

  std::uniform_int_distribution<int> pick(0, static_cast<int>(path.size()) - 1);

  for (int iter = 0; iter < params_.smoothing_iterations; ++iter) {
    if (path.size() < 3) {
      break;
    }

    int i = pick(rng_);
    int j = pick(rng_);
    if (i == j) {
      continue;
    }
    if (i > j) {
      std::swap(i, j);
    }
    if (j <= i + 1) {
      continue;
    }

    if (isSegmentValid(path[static_cast<size_t>(i)], path[static_cast<size_t>(j)])) {
      path.erase(path.begin() + i + 1, path.begin() + j);
      pick = std::uniform_int_distribution<int>(0, static_cast<int>(path.size()) - 1);
    }
  }
}

}  // namespace rrt_3d_planner
