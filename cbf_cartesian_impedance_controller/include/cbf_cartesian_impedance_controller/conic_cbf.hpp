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

// # Define the CBF
// def h(R, e_w, e_i, theta_i):
//     return e_w.T @ R.T @ e_i - np.cos(theta_i)
inline double h(Eigen::Matrix<real_t, 3, 3>& R, Eigen::Vector3d& e_w,
                Eigen::Vector3d& e_ref, double theta_i) {
  return e_w.transpose() * R.transpose() * e_ref - std::cos(theta_i);
}

// # Define the CBF constraint
// def cbf_constraint(u, R, e_w, e_i, theta, alpha):
//     # return -e_i.T @ R @ skew(e_i) @ u + alpha * h(R, e_i, theta)
//     e_w_skew = skew(e_w)
//     h_val = h(R, e_w, e_i, theta)
//     return -e_i.T @ R @ e_w_skew @ u + alpha * h_val

// lie derivative of h with respect to g
inline Eigen::Vector3d L_gh(Eigen::Matrix<real_t, 3, 3>& R,
                            Eigen::Vector3d& e_w, Eigen::Vector3d& e_ref) {
  return -e_ref.transpose() * R * skew(e_w);
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

std::vector<double> cbfOrientFilter(KDL::Frame& ref_frame,
                                    KDL::Frame& filtered_target_frame,
                                    KDL::Frame& target_frame,
                                    Eigen::Vector3d& thetas, double dt) {
  std::vector<double> logs;
  // setup problem
  qpOASES::Options qpOptions;
  qpOptions.printLevel = qpOASES::PL_LOW;
  const int n_constraints = 3;
  const double gamma = 1.0;
  // setup problem, Hessian is identity
  qpOASES::QProblem min_problem(3, n_constraints,
                                qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(qpOptions);

  Eigen::Matrix<real_t, 3, 3> R_ref(ref_frame.M.data);
  Eigen::Matrix<real_t, 3, 3> R_new(target_frame.M.data);
  Eigen::Matrix<real_t, 3, 3> R(filtered_target_frame.M.data);
  // auto omega = computeSkewSymmetric(R_new, R, dt);
  // extract omega from error
  // Eigen::Vector3d u_nominal = {omega(2, 1), omega(0, 2), omega(1, 0)};
  Eigen::Vector3d u_nominal = {1.0, 1.2, -0.3};
  const Eigen::Vector<real_t, 3> g = -u_nominal;

  // constraints
  Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor> A(n_constraints, 3);
  Eigen::Vector<real_t, Eigen::Dynamic> A_lb(n_constraints);

  // fill A matrix
  for (int i = 0; i < n_constraints; ++i) {
    Eigen::Vector3d e_ref = R_ref.col(i);
    Eigen::Vector3d e_w = Eigen::Vector3d::Unit(i);

    A.row(i) = L_gh(R, e_w, e_ref).transpose();
    A_lb(i) = -gamma * h(R, e_w, e_ref, thetas(i));
    logs.push_back(h(R, e_w, e_ref, thetas(i)));  // 0 - 1 - 2
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
  logs.push_back(u(0));  // 3
  logs.push_back(u(1));  // 4
  logs.push_back(u(2));  // 5
  Eigen::MatrixXd skew_u = dt * skew(omega_opt);
  Eigen::MatrixXd skew_u_exp = skew_u.exp();
  Eigen::MatrixXd R_opt = R * skew_u_exp;

  // Ensure the resulting matrix is still a valid rotation matrix (orthonormal)
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd(
  //     R_opt, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // R_opt = svd.matrixU() * svd.matrixV().transpose();

  // filtered_target_frame.p = target_frame.p;
  double* kdlData = filtered_target_frame.M.data;
  for (int i = 0; i < 9; ++i) {
    kdlData[i] = R_opt(i);
  }

  return logs;
}

}  // namespace conic_cbf
#endif  // CONIC_CBF_QP_HPP
