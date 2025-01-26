#ifndef PLANES_CBF_QP_HPP
#define PLANES_CBF_QP_HPP
#include "pseudo_inversion.h"
#include "qp_wrapper.hpp"
namespace planes_hocbf {

// control barrier function that ensures that the robot stays above a plane
double h(const Eigen::Vector3d& x, const Eigen::Vector3d& n,
         const Eigen::Vector3d& p) {
  return n.dot(x - p);
}

double dot_h(const Eigen::Vector3d& dot_x1, const Eigen::Vector3d& n) {
  return n.dot(dot_x1);
}
double ddot_h(const Eigen::Vector3d& dot_x2, const Eigen::Vector3d& n) {
  return n.dot(dot_x2);
}
// full hocbf debug function
// double log_psi_2_quadratic(const Eigen::Vector3d& tau_nominal,
//                            const Eigen::MatrixXd& Lambda,
//                            const Eigen::MatrixXd& J, const Eigen::VectorXd&
//                            mu, const Eigen::Vector3d& x_des, const
//                            Eigen::Vector3d& x, const Eigen::Vector3d& dot_x1,
//                            double dt, const Eigen::Vector3d& n, const
//                            Eigen::Vector3d& p) {
//   // void(mu);
//   Eigen::MatrixXd J_tran_pinv;
//   pseudoInverse(J.transpose(), &J_tran_pinv);
//   // ignore coriolis and gravity, and setting Lambda_des = Lambda cancels out
//   // F^{ext}
//   Eigen::Vector3d dot_x2 =
//       (-Lambda.inverse() * J_tran_pinv * tau_nominal).head(3);
//   const double h_ = h(x, n, p);
//   const double dot_h_ = dot_h(dot_x1, n);
//   const double ddot_h_ = ddot_h(dot_x2, n);
//   return ddot_h_ + 2 * dot_h_ * h_ + std::pow(dot_h_, 2) + std::pow(h_, 4) +
//          2 * dot_h_ * pow(h_, 2);
// }
// psi_2 b vector
// double quadratic_psi2_b(const Eigen::MatrixXd& Lambda,
//                         const Eigen::VectorXd& mu, const Eigen::Vector3d& x,
//                         const Eigen::Vector3d& dot_x, const Eigen::Vector3d&
//                         n, const Eigen::Vector3d& p) {
//   const double h_ = h(x, n, p);
//   const double dot_h_ = dot_h(dot_x, n);
//   return -2 * dot_h_ * h_ - std::pow(dot_h_, 2) - std::pow(h_, 4) -
//          2 * dot_h_ * pow(h_, 2);
// }

double log_psi2_linear(const Eigen::VectorXd& F_u,
                       const Eigen::MatrixXd& Lambda, const Eigen::MatrixXd& J,
                       const Eigen::VectorXd& mu, const Eigen::Vector3d& x,
                       const Eigen::Vector3d& dot_x1, const Eigen::Vector3d& n,
                       const Eigen::Vector3d& p, const double k) {
  // ignore coriolis and gravity,
  //     and setting Lambda_des =
  //         Lambda cancels out F ^ { ext }
  Eigen::Vector3d dot_x2 = (Lambda.inverse() * F_u).head(3);
  const double h_ = h(x, n, p);
  const double dot_h_ = dot_h(dot_x1, n);
  const double ddot_h_ = ddot_h(dot_x2, n);
  // return ddot_h_ + k * dot_h_ + k * std::sqrt(dot_h_ + k * h_);
  return ddot_h_ + 2 * k * dot_h_ + k * k * h_;
}

// psi_2 A matrix
Eigen::Vector3d psi2_A(const Eigen::MatrixXd& Lambda, const Eigen::Vector3d n) {
  Eigen::MatrixXd Lambda_inv = Lambda.inverse();
  return n.transpose() * Lambda_inv.block(0, 0, 3, 3);
  // auto Lambda_inv = Lambda.inverse();
  // return n.dot(Lambda_inv.block(0, 0, 3, 3));
}
double linear_psi2_b(const Eigen::MatrixXd& Lambda, const Eigen::VectorXd& mu,
                     const Eigen::Vector3d& x, const Eigen::Vector3d& dot_x,
                     const Eigen::Vector3d& n, const Eigen::Vector3d& p,
                     const double k) {
  const double h_ = h(x, n, p);
  const double dot_h_ = dot_h(dot_x, n);
  return -2 * k * dot_h_ - k * k * h_;
}

std::vector<double> hocbfPositionFilter(Eigen::VectorXd& F_u,
                                        const Eigen::MatrixXd& Lambda,
                                        const Eigen::MatrixXd& J,
                                        const Eigen::VectorXd& mu,
                                        const KDL::Frame& current_frame,
                                        const Eigen::Vector3d& dot_x, double dt,
                                        const std::vector<Eigen::Vector3d>& n,
                                        const std::vector<Eigen::Vector3d>& p) {
  std::vector<double> logs;
  // setup problem
  qpOASES::Options qpOptions;
  qpOptions.printLevel = qpOASES::PL_LOW;
  const int n_constraints = n.size();
  // setup problem, Hessian is identity
  qpOASES::QProblem min_problem(3, n_constraints,
                                qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(qpOptions);

  const double k = 50.0;
  Eigen::Vector3d x(current_frame.p.x(), current_frame.p.y(),
                    current_frame.p.z());
  // Eigen::MatrixXd J_tran_pinv;
  // pseudoInverse(J.transpose(), &J_tran_pinv, false);

  // std::cout << "J_tran_pinv * J_tran = \n"
  //           << (J_tran_pinv * J.transpose()) << std::endl;
  // std::cout << "-----" << std::endl;

  // Eigen::VectorXd F_u = J_tran_pinv * tau_nominal;

  Eigen::Vector<real_t, 3> F_u_pos = {F_u(0), F_u(1), F_u(2)};
  Eigen::Vector<real_t, 3> g = -F_u_pos;
  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
  for (int i = 0; i < n_constraints; ++i) {
    A.row(i) = psi2_A(Lambda, n[i]);
  }
  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  for (int i = 0; i < n_constraints; ++i) {
    A_lb(i) = linear_psi2_b(Lambda, mu, x, dot_x, n[i], p[i], k);
  }
  int nWSR = 200;

  min_problem.init(0, g.data(), A.data(), 0, 0, A_lb.data(), 0, nWSR);

  Eigen::Vector<real_t, 3> F_u_pos_star;
  auto status = min_problem.getPrimalSolution(F_u_pos_star.data());
  if (status != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "Error in init: "
              << qpOASES::MessageHandling::getErrorCodeMessage(status)
              << std::endl;
    exit(1);
  }

  // Eigen::VectorXd tau_tmp = tau_nominal;
  F_u.head(3) = F_u_pos_star;
  // std::cout << "tau_error: " << (tau_nominal - tau_tmp).transpose()
  //           << std::endl;
  // std::cout << "tau_nominal: " << tau_nominal.transpose() << std::endl;
  // std::cout << "Lambda: " << Lambda << std::endl;
  // std::cout << "J: " << J << std::endl;
  // std::cout << "mu: " << mu.transpose() << std::endl;
  // std::cout << "x: " << x.transpose() << std::endl;
  // std::cout << "dot_x: " << dot_x.transpose() << std::endl;
  // std::cout << "n: " << n[0].transpose() << std::endl;
  // std::cout << "p: " << p[0].transpose() << std::endl;
  // std::cout << "k: " << k << std::endl;

  double psi_2 = log_psi2_linear(F_u, Lambda, J, mu, x, dot_x, n[0], p[0], k);
  logs.push_back(psi_2);
  return logs;
}
}  // namespace planes_hocbf
#endif  // PLANES_CBF_QP_HPP
