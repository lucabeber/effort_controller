#ifndef PLANES_CBF_QP_HPP
#define PLANES_CBF_QP_HPP
#include "qp_wrapper.hpp"
namespace planes_cbf {

// control barrier function that ensures that the robot stays above a plane
double h(Eigen::Vector<real_t, 3>& x, Eigen::Vector<real_t, 3>& n,
         Eigen::Vector<real_t, 3>& p) {
  return n.dot(x - p);
}
// lie derivative of h with respect to g
Eigen::Vector3d L_gh(Eigen::Vector3d& n) { return n.transpose(); }

std::vector<double> cbfPositionFilter(KDL::Frame& filtered_target_frame,
                                      KDL::Frame& target_frame,
                                      std::vector<Eigen::Vector3d>& n,
                                      std::vector<Eigen::Vector3d>& p) {
  std::vector<double> logs;
  // setup problem
  qpOASES::Options qpOptions;
  qpOptions.printLevel = qpOASES::PL_LOW;
  const int n_constraints = n.size();
  // gamma = 1.0 is the optimal value that reduces the constraint to n.T u >=
  // n.T (x - p) which is geometrically correct
  const double gamma = 1.0;
  // setup problem, Hessian is identity
  qpOASES::QProblem min_problem(3, 1, qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(qpOptions);

  // compute u nominal as
  KDL::Frame error;
  error.p = target_frame.p - filtered_target_frame.p;
  const Eigen::Vector<real_t, 3> u_nominal = {error.p.x(), error.p.y(),
                                              error.p.z()};
  const Eigen::Vector<real_t, 3> g = -u_nominal;

  // constraints
  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  Eigen::Vector<real_t, 3> x = {filtered_target_frame.p.x(),
                                filtered_target_frame.p.y(),
                                filtered_target_frame.p.z()};

  // fill A matrix
  for (int i = 0; i < n_constraints; ++i) {
    A.row(i) = L_gh(n[i]);
    A_lb(i) = -gamma * h(x, n[i], p[i]);
  }

  int nWSR = 200;

  min_problem.init(0, g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);
  Eigen::Vector<real_t, 3> u;
  auto status = min_problem.getPrimalSolution(u.data());
  if (status != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "Error in init: "
              << qpOASES::MessageHandling::getErrorCodeMessage(status)
              << std::endl;
    exit(1);
  }
  // std::cout << "result: " << u << std::endl;
  filtered_target_frame.p =
      filtered_target_frame.p + KDL::Vector(u(0), u(1), u(2));
  logs.push_back(filtered_target_frame.p.z());  // 0
  logs.push_back(u(2));                         // 1
  logs.push_back(error.p.z());                  // 2
  std::cout << x[2] << std::endl;
  std::cout << h(x, n[0], p[0]) << "\n---------" << std::endl;
  logs.push_back(h(x, n[0], p[0]));  // 3
  return logs;
}

}  // namespace planes_cbf
#endif  // PLANES_CBF_QP_HPP
