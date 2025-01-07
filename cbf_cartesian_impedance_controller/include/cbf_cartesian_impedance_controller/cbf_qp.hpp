#ifndef CBF_QP_HPP
#define CBF_QP_HPP
#include "qp_wrapper.hpp"
namespace cbf {

// control barrier function
double h(Eigen::Vector<real_t, 3>& x, Eigen::Vector<real_t, 3>& n,
         Eigen::Vector<real_t, 3>& p) {
  return n.dot(x - p);
}
// derivative of control barrier function
Eigen::Vector3d dot_h(Eigen::Vector3d& n) { return n.transpose(); }

std::vector<double> cbfFilterReference(KDL::Frame& filtered_target_frame,
                                       KDL::Frame& current_frame,
                                       KDL::Frame& new_target_frame,
                                       KDL::Frame& old_target_frame,
                                       std::vector<Eigen::Vector3d>& n,
                                       std::vector<Eigen::Vector3d>& p,
                                       double dt) {
  std::vector<double> logs;
  // setup problem
  qpOASES::Options qpOptions;
  qpOptions.printLevel = qpOASES::PL_LOW;
  const int n_constraints = n.size();
  const double gamma = 0.0;
  qpOASES::QProblem min_problem(3, 1, qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(qpOptions);

  // compute u nominal
  KDL::Frame error;
  error.p = new_target_frame.p - old_target_frame.p;
  const Eigen::Vector<real_t, 3> u_nominal = {error.p.x(), error.p.y(),
                                              error.p.z()};
  const Eigen::Vector<real_t, 3> g = -u_nominal;

  // constraints
  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);

  // fill A matrix
  for (int i = 0; i < n_constraints; ++i) {
    A.row(i) = dot_h(n[i]);
  }

  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  Eigen::Vector<real_t, 3> x = {old_target_frame.p.x(), old_target_frame.p.y(),
                                old_target_frame.p.z()};
  for (int i = 0; i < n_constraints; ++i) {
    A_lb(i) = -gamma * h(x, n[i], p[i]);
    // std::cout << "h(x): " << h(x, n[i], p[i]) << std::endl;
  }

  int nWSR = 200;

  min_problem.init(0, g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);
  Eigen::Vector<real_t, 3> filtered_error;
  auto status = min_problem.getPrimalSolution(filtered_error.data());
  if (status != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "Error in init: "
              << qpOASES::MessageHandling::getErrorCodeMessage(status)
              << std::endl;
    exit(1);
  }
  // std::cout << "result: " << filtered_error << std::endl;
  filtered_target_frame.p =
      old_target_frame.p +
      dt * KDL::Vector(filtered_error(0), filtered_error(1), filtered_error(2));
  filtered_target_frame.M = old_target_frame.M;
  old_target_frame = filtered_target_frame;
  logs.push_back(filtered_target_frame.p.z());  // 0
  logs.push_back(new_target_frame.p.z());       // 1
  logs.push_back(old_target_frame.p.z());       // 2
  logs.push_back(filtered_error(2));            // 3
  std::cout << x[2] << std::endl;
  std::cout << h(x, n[0], p[0]) << "\n---------" << std::endl;
  logs.push_back(h(x, n[0], p[0]));  // 4
  return logs;
}

}  // namespace cbf
#endif  // CBF_QP_HPP
