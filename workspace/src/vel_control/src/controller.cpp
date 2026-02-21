#include "vel_control/controller.hpp"

#include <iostream>

namespace vel_control {

Controller::Controller(const Config& config) : config_(config) {}

Controller::~Controller() {
  io_service_.stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
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
  return true;
}

bool Controller::update() {
  if (!interface_->waitForMessage(0)) {
    return false;
  }
  interface_->read(&inputs_);
  return true;
}

void Controller::write(const abb::egm::wrapper::Output& output) {
  interface_->write(output);
}

bool Controller::isInitialized() const {
  return interface_ != nullptr && interface_->isInitialized();
}

}  // namespace vel_control
