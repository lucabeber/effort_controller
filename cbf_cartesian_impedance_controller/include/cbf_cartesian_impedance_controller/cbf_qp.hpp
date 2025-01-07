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

void cbfFilterReference(KDL::Frame& current_frame, KDL::Frame& target_frame,
                        std::vector<Eigen::Vector3d>& n,
                        std::vector<Eigen::Vector3d>& p) {
  // setup problem
  qpOASES::Options myOptions;
  myOptions.printLevel = qpOASES::PL_LOW;
  const int n_constraints = 1;
  const double gamma = 10.0;
  qpOASES::QProblem min_problem(1, 1);
  min_problem.setOptions(myOptions);

  // objective hessian
  KDL::Frame error;
  error.p = target_frame.p - current_frame.p;
  const Eigen::Vector<real_t, 3> delta_x = {error.p.x(), error.p.y(),
                                            error.p.z()};
  const Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor> H =
      Eigen::Matrix<real_t, 3, 3, Eigen::RowMajor>::Identity();
  const Eigen::Vector<real_t, 3> g = -2 * delta_x;
  // constraints
  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);

  // fill A matrix
  for (int i = 0; i < n_constraints; ++i) {
    A.row(i) = dot_h(n[i]);
  }

  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  for (int i = 0; i < n_constraints; ++i) {
    Eigen::Vector<real_t, 3> x = {current_frame.p.x(), current_frame.p.y(),
                                  current_frame.p.z()};
    A_lb(i) = -gamma * h(x, n[i], p[i]);
  }

  int nWSR = 200;
  min_problem.init(H.data(), g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);
  Eigen::Vector<real_t, 3> filtered_error;
  min_problem.getPrimalSolution(filtered_error.data());
  // // std::cout << "delta_x: " << delta_x << std::endl;

  target_frame.p =
      current_frame.p +
      KDL::Vector(filtered_error(0), filtered_error(1), filtered_error(2));
}

}  // namespace cbf
#endif  // CBF_QP_HPP
