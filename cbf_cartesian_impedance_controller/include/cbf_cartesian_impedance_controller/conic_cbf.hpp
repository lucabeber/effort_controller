#ifndef CONIC_CBF_QP_HPP
#define CONIC_CBF_QP_HPP
#include <unsupported/Eigen/MatrixFunctions>

#include "qp_wrapper.hpp"

namespace conic_cbf {

// control barrier function that ensures that the orientation stays within a
// conic barriers

// skew matrix from vector omega
Eigen::Matrix<real_t, 3, 3> skew(Eigen::Vector3d& omega) {
  Eigen::Matrix<real_t, 3, 3> omega_skew;
  omega_skew << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1),
      omega(0), 0;
  return omega_skew;
}

// def h(R, e_i, theta_i):
//     return e_i.T @ R @ e_i - np.cos(theta_i)
// control barrier function that ensures that the robot's end effector
// orientation stays within a conic
inline double h(Eigen::Matrix<real_t, 3, 3>& R, Eigen::Vector3d& e_i,
                double theta_i) {
  return e_i.transpose() * R * e_i - std::cos(theta_i);
}
// def cbf_constraint(u, R, e_i, theta, alpha):
//     e_i_skew = skew(e_i)
//     h_val = h(R, e_i, theta)
//     return -e_i.T @ R @ e_i_skew @ u + alpha * h_val

// lie derivative of h with respect to g
inline Eigen::Vector3d L_gh(Eigen::Matrix<real_t, 3, 3>& R,
                            Eigen::Vector3d& e_i, double theta, double gamma) {
  return -e_i.transpose() * R * skew(e_i);
}

Eigen::Matrix<real_t, 3, 3> computeSkewSymmetric(
    const Eigen::Matrix<real_t, 3, 3>& R_t,
    const Eigen::Matrix<real_t, 3, 3>& R_t_minus_1, double delta_t) {
  // Compute the relative rotation matrix (Delta R)
  Eigen::Matrix<real_t, 3, 3> delta_R = R_t * R_t_minus_1.transpose();

  // Compute the skew-symmetric matrix Omega
  Eigen::Matrix<real_t, 3, 3> Omega =
      (delta_R - Eigen::Matrix<real_t, 3, 3>::Identity()) / delta_t;

  // Return the skew-symmetric matrix Omega
  return Omega;
}

std::vector<double> cbfOrientFilter(KDL::Frame& filtered_target_frame,
                                    KDL::Frame& target_frame,
                                    Eigen::Vector3d& thetas, double dt) {
  std::vector<double> logs;
  // setup problem
  qpOASES::Options qpOptions;
  qpOptions.printLevel = qpOASES::PL_LOW;
  const int n_constraints = 3;
  const double gamma = 1.0;
  // setup problem, Hessian is identity
  qpOASES::QProblem min_problem(3, 1, qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(qpOptions);

  // compute u nominal
  // compute omega
  Eigen::Matrix<real_t, 3, 3> R_new(target_frame.M.data);
  Eigen::Matrix<real_t, 3, 3> R(filtered_target_frame.M.data);
  auto omega = computeSkewSymmetric(R_new, R, dt);
  // extract omega from error
  Eigen::Vector3d u_nominal = {omega(2, 1), omega(0, 2), omega(1, 0)};
  const Eigen::Vector<real_t, 3> g = -u_nominal;

  // constraints
  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  // fill A matrix
  for (int i = 0; i < n_constraints; ++i) {
    // Eigen::Vector3d e_i = Eigen::Vector3d::Unit(i);

    // createrot matrix from quaternion
    Eigen::Quaterniond q(0, 1, 0, 0);
    Eigen::MatrixXd e_i_rot = q.toRotationMatrix();
    Eigen::Vector3d e_i = e_i_rot.col(i);

    A.row(i) = L_gh(R, e_i, thetas(i), gamma).transpose();
    A_lb(i) = -gamma * h(R, e_i, thetas(i));
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
  // R = R @ expm(dt * skew(u))
  Eigen::Vector3d omega_opt(u(0), u(1), u(2));
  Eigen::MatrixXd skew_u = skew(omega_opt) * dt;
  Eigen::MatrixXd skew_u_exp = skew_u.exp();
  Eigen::MatrixXd R_opt = R;
  R_opt = R_opt * skew_u_exp;
  filtered_target_frame.M = KDL::Rotation(
      R_opt(0, 0), R_opt(0, 1), R_opt(0, 2), R_opt(1, 0), R_opt(1, 1),
      R_opt(1, 2), R_opt(2, 0), R_opt(2, 1), R_opt(2, 2));

  return logs;
}

}  // namespace conic_cbf
#endif  // CONIC_CBF_QP_HPP
