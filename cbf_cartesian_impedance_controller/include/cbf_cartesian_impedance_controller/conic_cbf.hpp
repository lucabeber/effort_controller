#ifndef CONIC_CBF_QP_HPP
#define CONIC_CBF_QP_HPP
#include <unsupported/Eigen/MatrixFunctions>

#include "qp_wrapper.hpp"

namespace conic_cbf {

// control barrier function that ensures that the orientation stays within a
// conic barriers

// skew matrix from vector omega
Eigen::Matrix3d skew(Eigen::Vector3d &omega) {
  Eigen::Matrix3d omega_skew;
  omega_skew << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1),
      omega(0), 0;
  return omega_skew;
}
double theta_from_matrix(Eigen::Matrix3d &R) {
  return std::acos((R.trace() - 1) / 2);
}
// exponential map from vector omega
Eigen::Matrix3d exp_map(Eigen::Vector3d &omega) {
  double theta = omega.norm();
  if (theta == 0) {
    return Eigen::Matrix3d::Identity();
  }
  Eigen::Matrix3d omega_skew = skew(omega);
  return Eigen::Matrix3d::Identity() + std::sin(theta) / theta * omega_skew +
         (1 - std::cos(theta)) / (theta * theta) * omega_skew * omega_skew;
}
// logaritmic map from rotation matrix
Eigen::Vector3d log_map(Eigen::Matrix3d &R) {
  double theta = theta_from_matrix(R);
  if (theta == 0) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Matrix3d log_R = theta / (2 * std::sin(theta)) * (R - R.transpose());
  return Eigen::Vector3d(log_R(2, 1), log_R(0, 2), log_R(1, 0));
}
// # Define the CBF
inline double h(Eigen::Matrix3d &R, Eigen::Vector3d &e_w,
                Eigen::Vector3d &e_ref, double theta_i) {
  return e_w.transpose() * R.transpose() * e_ref - std::cos(theta_i);
}

// # Define the CBF constraint
// lie derivative of h with respect to g
inline Eigen::Vector3d L_gh(Eigen::Matrix3d &R, Eigen::Vector3d &e_w,
                            Eigen::Vector3d &e_ref) {
  return -e_ref.transpose() * R * skew(e_w);
}

std::vector<double> cbfOrientFilter(KDL::Frame &ref_frame,
                                    KDL::Frame &filtered_target_frame,
                                    KDL::Frame &target_frame,
                                    Eigen::Vector3d &thetas, double dt) {
  std::vector<double> logs;
  // setup problem
  qpOASES::Options qpOptions;
  qpOptions.printLevel = qpOASES::PL_LOW;
  const int n_constraints = 3;
  const double gamma = 100.0;
  // setup problem, Hessian is identity
  qpOASES::QProblem min_problem(3, n_constraints,
                                qpOASES::HessianType::HST_IDENTITY);
  min_problem.setOptions(qpOptions);

  // orientation used to center the conic barrier
  Eigen::Matrix3d R_ref(ref_frame.M.data);
  // new target orientation
  Eigen::Matrix3d R_new(target_frame.M.data);
  // current target orientation
  Eigen::Matrix3d R(filtered_target_frame.M.data);
  Eigen::Matrix3d delta_R = R * R_new.transpose();
  Eigen::Vector3d omega = log_map(delta_R);
  // extract omega from error
  Eigen::Vector3d u_nominal = {omega(0), omega(1), omega(2)};
  logs.push_back(u_nominal(0)); // 0
  logs.push_back(u_nominal(1)); // 1
  logs.push_back(u_nominal(2)); // 2
  // Eigen::Vector3d u_nominal = {1.0, 1.2, -0.3};
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
    logs.push_back(h(R, e_w, e_ref, thetas(i))); // 0 - 1 - 2
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
  logs.push_back(u(0)); // 3
  logs.push_back(u(1)); // 4
  logs.push_back(u(2)); // 5
  Eigen::Vector3d angle = omega_opt * dt;
  Eigen::Matrix3d R_opt = R * exp_map(angle);

  // filtered_target_frame.p = target_frame.p;
  // double* kdlData = filtered_target_frame.M.data;
  // for (int i = 0; i < 9; ++i) {
  //   kdlData[i] = R_opt(i);
  // }
  filtered_target_frame.M = KDL::Rotation(R_opt(0, 0), R_opt(0, 1), R_opt(0, 2),
                                   R_opt(1, 0), R_opt(1, 1), R_opt(1, 2),
                                   R_opt(2, 0), R_opt(2, 1), R_opt(2, 2));

  return logs;
}

} // namespace conic_cbf
#endif // CONIC_CBF_QP_HPP
