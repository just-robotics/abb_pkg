#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "rrt_3d_planner/rrt_3d_planner.hpp"
#include "rrt_3d_planner/voxel_map_3d.hpp"

namespace rrt_3d_planner {

class RRT3DPlannerNode : public rclcpp::Node {
 public:
  RRT3DPlannerNode()
      : Node("rrt_3d_planner_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    points_topic_ =
        declare_parameter<std::string>("points_topic", "lidar/points");
    ee_pose_topic_ =
        declare_parameter<std::string>("ee_pose_topic", "/ee_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_point");
    path_topic_ = declare_parameter<std::string>("path_topic", "/planned_path");
    map_lock_topic_ =
        declare_parameter<std::string>("map_lock_topic", "/lock_map");
    locked_voxels_topic_ = declare_parameter<std::string>(
        "locked_voxels_topic", "/locked_voxel_markers");

    voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.10);

    step_size_ = declare_parameter<double>("step_size", 0.20);
    goal_bias_ = declare_parameter<double>("goal_bias", 0.10);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.20);
    max_iterations_ = declare_parameter<int>("max_iterations", 3000);
    collision_step_ = declare_parameter<double>("collision_step", 0.05);
    enable_smoothing_ = declare_parameter<bool>("enable_smoothing", true);
    smoothing_iterations_ = declare_parameter<int>("smoothing_iterations", 150);

    x_min_ = declare_parameter<double>("x_min", -2.0);
    x_max_ = declare_parameter<double>("x_max", 4.0);
    y_min_ = declare_parameter<double>("y_min", -2.0);
    y_max_ = declare_parameter<double>("y_max", 2.0);
    z_min_ = declare_parameter<double>("z_min", 0.0);
    z_max_ = declare_parameter<double>("z_max", 2.0);

    use_voxel_filter_ = declare_parameter<bool>("use_voxel_filter", false);
    voxel_filter_leaf_size_ =
        declare_parameter<double>("voxel_filter_leaf_size", 0.03);
    use_crop_box_ = declare_parameter<bool>("use_crop_box", true);
    max_voxel_marker_points_ =
        declare_parameter<int>("max_voxel_marker_points", 50000);

    voxel_map_ =
        std::make_unique<VoxelMap3D>(voxel_size_, x_min_, y_min_, z_min_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RRT3DPlannerNode::cloudCallback, this,
                  std::placeholders::_1));

    ee_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        ee_pose_topic_, 10,
        std::bind(&RRT3DPlannerNode::eePoseCallback, this,
                  std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        goal_topic_, 10,
        std::bind(&RRT3DPlannerNode::goalCallback, this,
                  std::placeholders::_1));
    map_lock_sub_ = create_subscription<std_msgs::msg::Bool>(
        map_lock_topic_, 10,
        std::bind(&RRT3DPlannerNode::mapLockCallback, this,
                  std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    voxel_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("/voxel_markers", 1);
    tree_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/rrt_tree_markers", 1);
    locked_voxel_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        locked_voxels_topic_, 1);

    RCLCPP_INFO(get_logger(), "rrt_3d_planner_node started. world_frame=%s",
                world_frame_.c_str());
  }

 private:
  bool lookupTransformToWorld(
      const std::string& source_frame,
      geometry_msgs::msg::TransformStamped& transform_out) {
    if (source_frame.empty() || source_frame == world_frame_) {
      return false;
    }

    try {
      transform_out = tf_buffer_.lookupTransform(world_frame_, source_frame,
                                                 tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "TF lookup failed (%s -> %s): %s",
          source_frame.c_str(), world_frame_.c_str(), ex.what());
      return false;
    }
  }

  Point3D transformPoint(
      const Point3D& p,
      const geometry_msgs::msg::TransformStamped& tf_msg) const {
    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);
    tf2::Vector3 v(p.x, p.y, p.z);
    tf2::Vector3 out = tf * v;
    return Point3D{out.x(), out.y(), out.z()};
  }

  bool pointStampedToWorld(const geometry_msgs::msg::PointStamped& in,
                           Point3D& out) {
    if (in.header.frame_id.empty() || in.header.frame_id == world_frame_) {
      out = Point3D{in.point.x, in.point.y, in.point.z};
      return true;
    }

    try {
      auto tf_msg = tf_buffer_.lookupTransform(world_frame_, in.header.frame_id,
                                               tf2::TimePointZero);
      geometry_msgs::msg::PointStamped transformed;
      tf2::doTransform(in, transformed, tf_msg);
      out = Point3D{transformed.point.x, transformed.point.y,
                    transformed.point.z};
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "Cannot transform point from %s to %s: %s",
                  in.header.frame_id.c_str(), world_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool poseStampedToWorld(const geometry_msgs::msg::PoseStamped& in,
                          Point3D& out) {
    if (in.header.frame_id.empty() || in.header.frame_id == world_frame_) {
      out = Point3D{in.pose.position.x, in.pose.position.y, in.pose.position.z};
      return true;
    }

    try {
      auto tf_msg = tf_buffer_.lookupTransform(world_frame_, in.header.frame_id,
                                               tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(in, transformed, tf_msg);
      out = Point3D{transformed.pose.position.x, transformed.pose.position.y,
                    transformed.pose.position.z};
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "Cannot transform pose from %s to %s: %s",
                  in.header.frame_id.c_str(), world_frame_.c_str(), ex.what());
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (map_locked_) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<int> index_map;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index_map);

    if (use_voxel_filter_) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(static_cast<float>(voxel_filter_leaf_size_),
                     static_cast<float>(voxel_filter_leaf_size_),
                     static_cast<float>(voxel_filter_leaf_size_));
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
          new pcl::PointCloud<pcl::PointXYZ>());
      vg.filter(*filtered);
      cloud = filtered;
    }

    voxel_map_->clear();

    geometry_msgs::msg::TransformStamped tf_msg;
    bool need_tf = lookupTransformToWorld(msg->header.frame_id, tf_msg);
    if (!msg->header.frame_id.empty() && msg->header.frame_id != world_frame_ &&
        !need_tf) {
      map_ready_ = false;
      return;
    }

    for (const auto& p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      Point3D wp{p.x, p.y, p.z};
      if (need_tf) {
        wp = transformPoint(wp, tf_msg);
      }

      if (use_crop_box_) {
        if (wp.x < x_min_ || wp.x > x_max_ || wp.y < y_min_ || wp.y > y_max_ ||
            wp.z < z_min_ || wp.z > z_max_) {
          continue;
        }
      }

      voxel_map_->insertPoint(wp.x, wp.y, wp.z);
    }

    map_ready_ = true;
    publishVoxelMarkers();
  }

  void mapLockCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      if (map_locked_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Map already locked");
        return;
      }

      if (!map_ready_) {
        RCLCPP_WARN(get_logger(),
                    "Cannot lock map: no valid cloud map received yet");
        return;
      }

      map_locked_ = true;
      publishLockedVoxelMarkers();
      RCLCPP_INFO(get_logger(),
                  "Map locked. Planning will use the last received cloud until "
                  "unlock or node restart");
      return;
    }

    if (!map_locked_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Map already unlocked");
      return;
    }

    map_locked_ = false;
    clearLockedVoxelMarkers();
    RCLCPP_INFO(get_logger(), "Map unlocked. Cloud updates resumed");
  }

  void eePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Point3D p;
    if (!poseStampedToWorld(*msg, p)) {
      return;
    }
    start_world_ = p;
    start_received_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    Point3D p;
    if (!pointStampedToWorld(*msg, p)) {
      return;
    }
    goal_world_ = p;
    goal_received_ = true;
    planIfReady();
  }

  void planIfReady() {
    if (!map_locked_ || !map_ready_ || !start_received_ || !goal_received_) {
      return;
    }

    PlannerParams planner_params;
    planner_params.step_size = step_size_;
    planner_params.goal_bias = goal_bias_;
    planner_params.goal_tolerance = goal_tolerance_;
    planner_params.max_iterations = max_iterations_;
    planner_params.collision_step = std::min(collision_step_, voxel_size_);
    planner_params.robot_radius = robot_radius_;
    planner_params.enable_smoothing = enable_smoothing_;
    planner_params.smoothing_iterations = smoothing_iterations_;

    Bounds3D bounds;
    bounds.x_min = x_min_;
    bounds.x_max = x_max_;
    bounds.y_min = y_min_;
    bounds.y_max = y_max_;
    bounds.z_min = z_min_;
    bounds.z_max = z_max_;

    RRT3DPlanner planner(*voxel_map_, planner_params, bounds);
    std::vector<Point3D> path;

    const bool ok = planner.plan(start_world_, goal_world_, path);

    publishTreeMarkers(planner.tree());

    if (!ok) {
      RCLCPP_WARN(get_logger(), "RRT failed to find path");
      publishPath({});
      return;
    }

    publishPath(path);
    RCLCPP_INFO(get_logger(), "Path planned: %zu points", path.size());
  }

  void publishPath(const std::vector<Point3D>& path) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp = now();

    for (const auto& p : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.position.z = p.z;
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);
  }

  void publishVoxelMarkers() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = now();
    marker.ns = "voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = voxel_size_;
    marker.scale.y = voxel_size_;
    marker.scale.z = voxel_size_;
    marker.color.r = 0.9f;
    marker.color.g = 0.1f;
    marker.color.b = 0.1f;
    marker.color.a = 0.5f;

    const auto& voxels = voxel_map_->occupiedVoxels();
    const std::size_t total = voxels.size();
    const std::size_t max_pts =
        static_cast<std::size_t>(std::max(1, max_voxel_marker_points_));
    const std::size_t stride = std::max<std::size_t>(1, total / max_pts);

    std::size_t idx = 0;
    for (const auto& v : voxels) {
      if (idx % stride != 0) {
        ++idx;
        continue;
      }
      const auto p = voxel_map_->voxelToWorld(v.ix, v.iy, v.iz);
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      marker.points.push_back(gp);
      ++idx;
    }

    voxel_pub_->publish(marker);
  }

  void publishLockedVoxelMarkers() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = now();
    marker.ns = "locked_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = voxel_size_;
    marker.scale.y = voxel_size_;
    marker.scale.z = voxel_size_;
    marker.color.r = 0.1f;
    marker.color.g = 0.3f;
    marker.color.b = 1.0f;
    marker.color.a = 0.7f;

    const auto& voxels = voxel_map_->occupiedVoxels();
    const std::size_t total = voxels.size();
    const std::size_t max_pts =
        static_cast<std::size_t>(std::max(1, max_voxel_marker_points_));
    const std::size_t stride = std::max<std::size_t>(1, total / max_pts);

    std::size_t idx = 0;
    for (const auto& v : voxels) {
      if (idx % stride != 0) {
        ++idx;
        continue;
      }
      const auto p = voxel_map_->voxelToWorld(v.ix, v.iy, v.iz);
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      marker.points.push_back(gp);
      ++idx;
    }

    locked_voxel_pub_->publish(marker);
  }

  void clearLockedVoxelMarkers() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = now();
    marker.ns = "locked_voxels";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    locked_voxel_pub_->publish(marker);
  }

  void publishTreeMarkers(const std::vector<Node3D>& tree) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = now();
    marker.ns = "rrt_tree";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.r = 0.0f;
    marker.color.g = 0.8f;
    marker.color.b = 0.2f;
    marker.color.a = 0.6f;

    for (std::size_t i = 0; i < tree.size(); ++i) {
      const auto& n = tree[i];
      if (n.parent_index < 0) {
        continue;
      }
      const auto& p = tree[static_cast<std::size_t>(n.parent_index)];
      geometry_msgs::msg::Point a;
      a.x = p.x;
      a.y = p.y;
      a.z = p.z;
      geometry_msgs::msg::Point b;
      b.x = n.x;
      b.y = n.y;
      b.z = n.z;
      marker.points.push_back(a);
      marker.points.push_back(b);
    }

    tree_pub_->publish(marker);
  }

  std::string world_frame_;
  std::string points_topic_;
  std::string ee_pose_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string map_lock_topic_;
  std::string locked_voxels_topic_;

  double voxel_size_;
  double robot_radius_;

  double step_size_;
  double goal_bias_;
  double goal_tolerance_;
  int max_iterations_;
  double collision_step_;
  bool enable_smoothing_;
  int smoothing_iterations_;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

  bool use_voxel_filter_;
  double voxel_filter_leaf_size_;
  bool use_crop_box_;
  int max_voxel_marker_points_;

  std::unique_ptr<VoxelMap3D> voxel_map_;
  bool map_ready_{false};
  bool map_locked_{false};
  bool start_received_{false};
  bool goal_received_{false};
  Point3D start_world_;
  Point3D goal_world_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_lock_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr voxel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      locked_voxel_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace rrt_3d_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rrt_3d_planner::RRT3DPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
