#ifndef VEL_CONTROL_CONTROLLER_HPP
#define VEL_CONTROL_CONTROLLER_HPP

#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_controller_interface.h>
#include <abb_libegm/egm_wrapper.pb.h>

#include <boost/asio/io_service.hpp>
#include <memory>
#include <thread>

namespace vel_control {

class Controller {
 public:
  struct Config {
    int port = 6515;
  };

 private:
  Config config_;
  boost::asio::io_service io_service_;
  std::thread io_thread_;
  std::unique_ptr<abb::egm::EGMControllerInterface> interface_;
  abb::egm::wrapper::Input inputs_;

 public:
  explicit Controller(const Config& config);
  ~Controller();

  bool initialize();
  bool update();
  void write(const abb::egm::wrapper::Output& output);
  abb::egm::wrapper::Input getInputs() const { return inputs_; }
  bool isInitialized() const;
};

}  // namespace vel_control

#endif  // VEL_CONTROL_CONTROLLER_HPP
