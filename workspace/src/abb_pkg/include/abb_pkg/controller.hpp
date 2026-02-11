#ifndef ABB_PKG_CONTROLLER_HPP
#define ABB_PKG_CONTROLLER_HPP

#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_controller_interface.h>
#include <abb_libegm/egm_wrapper.pb.h>

#include <boost/asio/io_service.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <string>
#include <thread>
#include <trac_ik/trac_ik.hpp>
#include <vector>

namespace abb_pkg {

class Controller {
 public:
  struct Config {
    int port;
    double dlin;
    double radius;
    double dtheta;
    double x0;
    double y0;
    double x1;
    double y1;
    double z0;
    double rx0;
    double ry0;
    double rz0;
    std::string base_link;
    std::string tip_link;
    std::string urdf_string;
    double ik_timeout;
    double ik_eps;
    double dq_limit;
    double joint_tolerance;
  };

  struct State {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
  };

 private:
  Config config_;

  boost::asio::io_service io_service_;
  std::thread io_thread_;

  std::unique_ptr<abb::egm::EGMControllerInterface> interface_;
  abb::egm::wrapper::Input inputs_;

  KDL::Chain chain_;
  KDL::JntArray seed_;
  std::unique_ptr<TRAC_IK::TRAC_IK> ik_;

 public:
  explicit Controller(const Config& config);
  ~Controller();

  bool initialize();
  bool update();

  bool stepToward(const std::vector<double>& target,
                  abb::egm::wrapper::Output& output);
  bool stepTowardJoints(const std::vector<double>& target_q,
                        abb::egm::wrapper::Output& output, double dq_limit,
                        double tolerance);

  void write(const abb::egm::wrapper::Output& output);

  std::vector<std::vector<double>> generateCircle();
  std::vector<std::vector<double>> generateLine();

  std::vector<std::vector<double>> trajectoryToJoints(
      const std::vector<std::vector<double>>& traj);

  State getCurrentState() const;
  abb::egm::wrapper::Input getInputs() const { return inputs_; }

  bool isInitialized() const;

  // Test function: compare FK with EGM feedback
  void testFKvsEGM() const;
};

}  // namespace abb_pkg

#endif  // ABB_PKG_CONTROLLER_HPP
