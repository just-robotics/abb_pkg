#include "abb_pkg/controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <stdexcept>

namespace abb_pkg {

Controller::Controller(const Config& config) : config_(config) {}

Controller::~Controller() {
  io_service_.stop();
  if (io_thread_.joinable()) io_thread_.join();
}

bool Controller::initialize() {
  std::cout << "[Controller] Starting initialization..." << std::endl;

  abb::egm::BaseConfiguration egm_config;
  egm_config.axes = abb::egm::Six;
  egm_config.use_demo_outputs = false;
  egm_config.use_velocity_outputs = true;

  std::cout << "[Controller] Creating EGM interface on port " << config_.port
            << "..." << std::endl;
  interface_ = std::make_unique<abb::egm::EGMControllerInterface>(
      io_service_, static_cast<unsigned short>(config_.port), egm_config);

  std::cout << "[Controller] Checking EGM interface initialization..."
            << std::endl;
  if (!interface_->isInitialized()) {
    std::cout << "[Controller] ERROR: EGM interface not initialized!"
              << std::endl;
    return false;
  }
  std::cout << "[Controller] EGM interface initialized successfully"
            << std::endl;

  std::cout << "[Controller] Starting IO service thread..." << std::endl;
  io_thread_ = std::thread([this]() { io_service_.run(); });

  std::cout << "[Controller] Initializing TRAC-IK solver..." << std::endl;
  ik_ = std::make_unique<TRAC_IK::TRAC_IK>(
      config_.base_link, config_.tip_link, config_.urdf_string,
      config_.ik_timeout, config_.ik_eps, TRAC_IK::Distance);

  KDL::JntArray lb, ub;

  std::cout << "[Controller] Getting KDL chain and limits..." << std::endl;
  if (!ik_->getKDLChain(chain_) || !ik_->getKDLLimits(lb, ub)) {
    std::cout << "[Controller] ERROR: TRAC-IK init failed!" << std::endl;
    throw std::runtime_error("TRAC-IK init failed");
  }
  std::cout << "[Controller] TRAC-IK initialized successfully, chain has "
            << chain_.getNrOfJoints() << " joints" << std::endl;

  seed_.resize(chain_.getNrOfJoints());
  seed_.data.setZero();

  std::cout << "[Controller] Initialization complete!" << std::endl;
  return true;
}

bool Controller::update() {
  static int update_count = 0;
  static int fail_count = 0;

  std::cout << "[Controller] update() called, waiting for message..."
            << std::endl;
  bool has_message = interface_->waitForMessage(0);
  std::cout << "[Controller] waitForMessage returned: "
            << (has_message ? "true" : "false") << std::endl;

  if (!has_message) {
    fail_count++;
    if (fail_count % 1000 == 0) {
      std::cout << "[Controller] Waiting for message... (failures: "
                << fail_count << ")" << std::endl;
    }
    return false;
  }

  std::cout << "[Controller] Reading message..." << std::endl;
  interface_->read(&inputs_);
  update_count++;
  std::cout << "[Controller] Message read successfully (count: " << update_count
            << ")" << std::endl;
  if (update_count % 250 == 0) {
    std::cout << "[Controller] Update successful (count: " << update_count
              << ")" << std::endl;
  }
  return true;
}

bool Controller::stepToward(const std::vector<double>& target,
                            abb::egm::wrapper::Output& output) {
  const auto& fb = inputs_.feedback().robot().cartesian().pose();

  const double cx = fb.position().x();
  const double cy = fb.position().y();
  const double cz = fb.position().z();

  const double dx = target[0] - cx;
  const double dy = target[1] - cy;
  const double dz = target[2] - cz;

  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double scale =
      (dist > config_.dlin && dist > 1e-9) ? (config_.dlin / dist) : 1.0;

  auto* pose = output.mutable_robot()->mutable_cartesian()->mutable_pose();

  pose->mutable_position()->set_x(cx + dx * scale);
  pose->mutable_position()->set_y(cy + dy * scale);
  pose->mutable_position()->set_z(cz + dz * scale);

  pose->mutable_euler()->set_x(target[3]);
  pose->mutable_euler()->set_y(target[4]);
  pose->mutable_euler()->set_z(target[5]);

  return dist < config_.dlin;
}

bool Controller::stepTowardJoints(const std::vector<double>& target_q,
                                  abb::egm::wrapper::Output& output,
                                  double dq_limit, double tolerance) {
  const auto& fb = inputs_.feedback().robot().joints().position();

  const int nj = fb.values_size();

  if ((int)target_q.size() != nj)
    throw std::runtime_error("Joint size mismatch");

  std::vector<double> next_q(nj);
  bool reached = true;

  double max_dq = 0.0;
  for (int i = 0; i < nj; ++i) {
    const double dq = std::abs(target_q[i] - fb.values(i));
    if (dq > max_dq) {
      max_dq = dq;
    }
  }

  // const double scale =
  //     (max_dq > dq_limit && max_dq > 1e-9) ? (dq_limit / max_dq) : 1.0;

  const double scale = 1.;

  for (int i = 0; i < nj; ++i) {
    const double q = fb.values(i);
    const double dq = target_q[i] - q;

    double step = dq * scale;
    next_q[i] = q + step;
  }

  reached = max_dq < tolerance;

  auto* joints = output.mutable_robot()->mutable_joints()->mutable_position();

  joints->clear_values();

  for (double q : next_q) joints->add_values(q);

  return reached;
}

void Controller::write(const abb::egm::wrapper::Output& output) {
  interface_->write(output);
}

std::vector<std::vector<double>> Controller::generateCircle() {
  const int n =
      std::max(1, static_cast<int>(std::round(2.0 * M_PI / config_.dtheta)));

  std::vector<std::vector<double>> trajectory;
  trajectory.reserve(static_cast<size_t>(n));

  for (int i = 0; i < n; ++i) {
    const double th = static_cast<double>(i) * config_.dtheta;
    trajectory.push_back({config_.x0 + config_.radius * std::cos(th),
                          config_.y0 + config_.radius * std::sin(th),
                          config_.z0, config_.rx0, config_.ry0, config_.rz0});
  }

  return trajectory;
}

std::vector<std::vector<double>> Controller::generateLine() {
  const int n = 10;

  std::vector<std::vector<double>> trajectory;
  trajectory.reserve(static_cast<size_t>(n));

  double dp = 1 / static_cast<double>(n);
  double p = 1.;

  for (int i = 0; i < n; ++i) {
    double x = p * config_.x0 + (1 - p) * config_.x1;
    double y = p * config_.y0 + (1 - p) * config_.y1;
    p -= dp;

    trajectory.push_back(
        {x, y, config_.z0, config_.rx0, config_.ry0, config_.rz0});
  }

  return trajectory;
}

std::vector<std::vector<double>> Controller::trajectoryToJoints(
    const std::vector<std::vector<double>>& traj) {
  std::vector<std::vector<double>> result;

  const unsigned nj = chain_.getNrOfJoints();
  if (nj == 0) {
    throw std::runtime_error("IK chain not initialized");
  }

  static constexpr double deg2rad = M_PI / 180.0;
  static constexpr double rad2deg = 180.0 / M_PI;

  KDL::JntArray lb, ub;
  bool has_limits = ik_->getKDLLimits(lb, ub);

  KDL::JntArray seed = seed_;

  try {
    const auto& fb = inputs_.feedback().robot().joints().position();
    if (fb.values_size() == static_cast<int>(nj)) {
      for (unsigned i = 0;
           i < nj && i < static_cast<unsigned>(fb.values_size()); ++i) {
        seed(i) = fb.values(i) * deg2rad;
      }
    }
  } catch (...) {
    if (has_limits) {
      for (unsigned i = 0; i < nj; ++i) {
        seed(i) = (lb(i) + ub(i)) / 2.0;
      }
    }
  }

  KDL::JntArray q_out(nj);
  KDL::JntArray last_successful_seed = seed;
  bool has_successful_solution = false;

  result.reserve(traj.size());

  std::cout << "[Controller] Starting IK computation for " << traj.size()
            << " points..." << std::endl;

  for (size_t traj_idx = 0; traj_idx < traj.size(); ++traj_idx) {
    if (traj_idx % 10 == 0 || traj_idx == traj.size() - 1) {
      std::cout << "[Controller] Processing point " << (traj_idx + 1) << "/"
                << traj.size() << std::endl;
    }

    const auto& p = traj[traj_idx];
    if (p.size() != 6) {
      result.emplace_back();
      continue;
    }

    double x_m = p[0] * 1e-3;
    double y_m = p[1] * 1e-3;
    double z_m = p[2] * 1e-3;

    KDL::Frame frame(
        KDL::Rotation::RPY(p[3] * deg2rad, p[4] * deg2rad, p[5] * deg2rad),
        KDL::Vector(x_m, y_m, z_m));

    int rc = ik_->CartToJnt(seed, frame, q_out, KDL::Twist::Zero());

    if (rc < 0 && has_successful_solution) {
      rc = ik_->CartToJnt(last_successful_seed, frame, q_out,
                          KDL::Twist::Zero());
    }

    if (rc < 0 && has_limits) {
      KDL::JntArray midpoint_seed(nj);
      for (unsigned i = 0; i < nj; ++i) {
        midpoint_seed(i) = (lb(i) + ub(i)) / 2.0;
      }
      rc = ik_->CartToJnt(midpoint_seed, frame, q_out, KDL::Twist::Zero());
    }

    if (rc < 0 && has_limits) {
      if (has_successful_solution) {
        for (int attempt = 0; attempt < 5 && rc < 0; ++attempt) {
          KDL::JntArray random_seed(nj);
          for (unsigned i = 0; i < nj; ++i) {
            double range = (ub(i) - lb(i)) * 0.2;
            double offset =
                (static_cast<double>(rand()) / RAND_MAX - 0.5) * range;
            random_seed(i) = last_successful_seed(i) + offset;
            if (random_seed(i) < lb(i)) random_seed(i) = lb(i);
            if (random_seed(i) > ub(i)) random_seed(i) = ub(i);
          }
          rc = ik_->CartToJnt(random_seed, frame, q_out, KDL::Twist::Zero());
        }
      }

      if (rc < 0) {
        KDL::JntArray midpoint_seed(nj);
        for (unsigned i = 0; i < nj; ++i) {
          midpoint_seed(i) = (lb(i) + ub(i)) / 2.0;
        }
        for (int attempt = 0; attempt < 5 && rc < 0; ++attempt) {
          KDL::JntArray random_seed(nj);
          for (unsigned i = 0; i < nj; ++i) {
            double range = (ub(i) - lb(i)) * 0.3;
            double offset =
                (static_cast<double>(rand()) / RAND_MAX - 0.5) * range;
            random_seed(i) = midpoint_seed(i) + offset;
            if (random_seed(i) < lb(i)) random_seed(i) = lb(i);
            if (random_seed(i) > ub(i)) random_seed(i) = ub(i);
          }
          rc = ik_->CartToJnt(random_seed, frame, q_out, KDL::Twist::Zero());
        }
      }

      if (rc < 0) {
        for (int attempt = 0; attempt < 10 && rc < 0; ++attempt) {
          KDL::JntArray random_seed(nj);
          for (unsigned i = 0; i < nj; ++i) {
            double range = ub(i) - lb(i);
            random_seed(i) =
                lb(i) + (static_cast<double>(rand()) / RAND_MAX) * range;
          }
          rc = ik_->CartToJnt(random_seed, frame, q_out, KDL::Twist::Zero());
        }
      }
    }

    if (rc >= 0) {
      std::vector<double> q(nj);
      for (unsigned i = 0; i < nj; ++i) q[i] = q_out(i) * rad2deg;

      result.push_back(q);

      seed = q_out;
      last_successful_seed = q_out;
      has_successful_solution = true;
    } else {
      result.emplace_back();
      if (traj_idx < 5 || traj_idx == traj.size() - 1) {
        std::cout << "[Controller] IK failed for point " << traj_idx
                  << " (x=" << p[0] << ", y=" << p[1] << ", z=" << p[2] << ")"
                  << std::endl;
      }
      if (has_successful_solution) {
        seed = last_successful_seed;
      }
    }
  }

  size_t success_count = 0;
  for (const auto& r : result) {
    if (!r.empty()) success_count++;
  }
  std::cout << "[Controller] IK computation finished: " << success_count << "/"
            << traj.size() << " points solved" << std::endl;

  return result;
}

Controller::State Controller::getCurrentState() const {
  const auto& fb = inputs_.feedback().robot().cartesian().pose();
  State state;
  state.x = fb.position().x();
  state.y = fb.position().y();
  state.z = fb.position().z();
  state.rx = fb.euler().x();
  state.ry = fb.euler().y();
  state.rz = fb.euler().z();
  return state;
}

bool Controller::isInitialized() const {
  return interface_ != nullptr && interface_->isInitialized();
}

void Controller::testFKvsEGM() const {
  std::cout << "\n========== FK vs EGM Test ==========" << std::endl;

  try {
    const auto& joint_fb = inputs_.feedback().robot().joints().position();
    const auto& cart_fb = inputs_.feedback().robot().cartesian().pose();

    const unsigned nj = chain_.getNrOfJoints();
    if (joint_fb.values_size() != static_cast<int>(nj)) {
      std::cout << "[FK Test] ERROR: Joint count mismatch! EGM: "
                << joint_fb.values_size() << ", Chain: " << nj << std::endl;
      return;
    }

    KDL::JntArray q(nj);
    static constexpr double deg2rad = M_PI / 180.0;
    for (unsigned i = 0;
         i < nj && i < static_cast<unsigned>(joint_fb.values_size()); ++i) {
      q(i) = joint_fb.values(i) * deg2rad;
    }

    std::cout << "[FK Test] Joint angles from EGM (deg):" << std::endl;
    for (unsigned i = 0; i < nj; ++i) {
      std::cout << "  Joint " << (i + 1) << ": " << joint_fb.values(i)
                << std::endl;
    }

    KDL::ChainFkSolverPos_recursive fk(chain_);
    KDL::Frame fk_frame;
    int fk_result = fk.JntToCart(q, fk_frame);

    if (fk_result < 0) {
      std::cout << "[FK Test] ERROR: FK computation failed!" << std::endl;
      return;
    }

    KDL::Vector fk_pos = fk_frame.p;
    double fk_roll, fk_pitch, fk_yaw;
    fk_frame.M.GetRPY(fk_roll, fk_pitch, fk_yaw);

    static constexpr double rad2deg = 180.0 / M_PI;
    double fk_x_mm = fk_pos.x() * 1000.0;
    double fk_y_mm = fk_pos.y() * 1000.0;
    double fk_z_mm = fk_pos.z() * 1000.0;
    double fk_rx_deg = fk_roll * rad2deg;
    double fk_ry_deg = fk_pitch * rad2deg;
    double fk_rz_deg = fk_yaw * rad2deg;

    double egm_x = cart_fb.position().x();
    double egm_y = cart_fb.position().y();
    double egm_z = cart_fb.position().z();
    double egm_rx = cart_fb.euler().x();
    double egm_ry = cart_fb.euler().y();
    double egm_rz = cart_fb.euler().z();

    std::cout << "\n[FK Test] Comparison:" << std::endl;
    std::cout << "  Position X: FK=" << fk_x_mm << " mm, EGM=" << egm_x
              << " mm, diff=" << std::abs(fk_x_mm - egm_x) << " mm"
              << std::endl;
    std::cout << "  Position Y: FK=" << fk_y_mm << " mm, EGM=" << egm_y
              << " mm, diff=" << std::abs(fk_y_mm - egm_y) << " mm"
              << std::endl;
    std::cout << "  Position Z: FK=" << fk_z_mm << " mm, EGM=" << egm_z
              << " mm, diff=" << std::abs(fk_z_mm - egm_z) << " mm"
              << std::endl;
    std::cout << "  Rotation RX: FK=" << fk_rx_deg << " deg, EGM=" << egm_rx
              << " deg, diff=" << std::abs(fk_rx_deg - egm_rx) << " deg"
              << std::endl;
    std::cout << "  Rotation RY: FK=" << fk_ry_deg << " deg, EGM=" << egm_ry
              << " deg, diff=" << std::abs(fk_ry_deg - egm_ry) << " deg"
              << std::endl;
    std::cout << "  Rotation RZ: FK=" << fk_rz_deg << " deg, EGM=" << egm_rz
              << " deg, diff=" << std::abs(fk_rz_deg - egm_rz) << " deg"
              << std::endl;

    const double pos_tolerance = 10.0;
    const double rot_tolerance = 5.0;

    bool pos_match = (std::abs(fk_x_mm - egm_x) < pos_tolerance &&
                      std::abs(fk_y_mm - egm_y) < pos_tolerance &&
                      std::abs(fk_z_mm - egm_z) < pos_tolerance);

    bool rot_match = (std::abs(fk_rx_deg - egm_rx) < rot_tolerance &&
                      std::abs(fk_ry_deg - egm_ry) < rot_tolerance &&
                      std::abs(fk_rz_deg - egm_rz) < rot_tolerance);

    std::cout << "\n[FK Test] FK Frame:" << std::endl;
    std::cout << "  Position: [" << fk_pos.x() << ", " << fk_pos.y() << ", "
              << fk_pos.z() << "]" << std::endl;
    std::cout << "  Rotation (RPY): [" << fk_roll << ", " << fk_pitch << ", "
              << fk_yaw << "] rad" << std::endl;
    std::cout << "  Rotation (RPY): [" << fk_rx_deg << ", " << fk_ry_deg << ", "
              << fk_rz_deg << "] deg" << std::endl;

    if (pos_match && rot_match) {
      std::cout << "\n[FK Test]  SUCCESS: FK and EGM match within tolerance!"
                << std::endl;
      std::cout << "  Position tolerance: " << pos_tolerance << " mm"
                << std::endl;
      std::cout << "  Rotation tolerance: " << rot_tolerance << " deg"
                << std::endl;
    } else {
      std::cout << "\n[FK Test] FAILED: FK and EGM do NOT match!" << std::endl;
      if (!pos_match) {
        std::cout << "   Position mismatch detected!" << std::endl;
      }
      if (!rot_match) {
        std::cout << "   Rotation mismatch detected!" << std::endl;
      }
      std::cout << "  This indicates URDF/frames do NOT correspond to EGM!"
                << std::endl;
    }

    std::cout << "=====================================\n" << std::endl;

  } catch (const std::exception& e) {
    std::cout << "[FK Test] ERROR: Exception caught: " << e.what() << std::endl;
  }
}

}  // namespace abb_pkg
