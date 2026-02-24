#include "abb_pkg/controller.hpp"

#include <cmath>
#include <kdl_parser/kdl_parser.hpp>
#include <stdexcept>

namespace abb_pkg {

Controller::Controller(const Config& config) : config_(config) {}

Controller::~Controller() {
  io_service_.stop();
  if (io_thread_.joinable()) io_thread_.join();
}

bool Controller::initialize() {
  abb::egm::BaseConfiguration egm_config;
  egm_config.axes = abb::egm::Six;
  egm_config.use_demo_outputs = false;
  egm_config.use_velocity_outputs = true;

  interface_ = std::make_unique<abb::egm::EGMControllerInterface>(
      io_service_, static_cast<unsigned short>(config_.port), egm_config);

  if (!interface_->isInitialized()) {
    return false;
  }

  io_thread_ = std::thread([this]() { io_service_.run(); });

  ik_ = std::make_unique<TRAC_IK::TRAC_IK>(
      config_.base_link, config_.tip_link, config_.urdf_string,
      config_.ik_timeout, config_.ik_eps, TRAC_IK::Distance);

  KDL::JntArray lb, ub;
  if (!ik_->getKDLChain(chain_) || !ik_->getKDLLimits(lb, ub)) {
    throw std::runtime_error("TRAC-IK init failed");
  }

  seed_.resize(chain_.getNrOfJoints());
  seed_.data.setZero();
  return true;
}

bool Controller::update() {
  if (!interface_->waitForMessage(0)) {
    return false;
  }
  interface_->read(&inputs_);
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

void Controller::write(const abb::egm::wrapper::Output& output) {
  interface_->write(output);
}

}  // namespace abb_pkg
