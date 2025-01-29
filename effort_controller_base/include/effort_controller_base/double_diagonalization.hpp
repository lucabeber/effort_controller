#include <Eigen/Dense>
#include <iostream>
#pragma once
// This function computes the damping matrix D_d as in A. Albu-Schaffer, C. Ott,
// U. Frese and G. Hirzinger, "Cartesian impedance control of redundant robots:
// recent results with the DLR-light-weight-arms," 2003 IEEE International
// Conference on Robotics and Automation
Eigen::MatrixXd compute_correct_damping(const Eigen::MatrixXd &Lambda,
                                        const Eigen::MatrixXd &K_d,
                                        const double csi = 1.0) {
  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(K_d, Lambda);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigen decomposition failed!");
  }
  const Eigen::MatrixXd Q = solver.eigenvectors().transpose().inverse();
  //   std::cout << "Lambda \n"
  //             << Lambda << std::endl
  //             << "Q*Q.t \n"
  //             << Q * Q.transpose() << std::endl;
  //   Eigen::SelfAdjointEigenSolver<const Eigen::MatrixXd>
  //   eigenSolverLambda(Lambda); std::cout << "Lambda eigenvalues: \n"
  //             << eigenSolverLambda.eigenvalues() << std::endl;

  const Eigen::MatrixXd K_d0 = solver.eigenvalues().asDiagonal();
  //   std::cout << "K_d " << K_d << std::endl;
  //   std::cout << "Q * K_d0 * Q.t " << Q * K_d0 * Q.transpose() << std::endl;

  //   Eigen::SelfAdjointEigenSolver<const Eigen::MatrixXd> eigenSolver1(K_d0);
  //   std::cout << "K_d0 eigenvalues: \n"
  //             << eigenSolver1.eigenvalues() << std::endl;

  const Eigen::MatrixXd D_d0 = 2.0 * csi * K_d0.cwiseSqrt();
  //   std::cout << "D_d0: \n" << D_d0 << std::endl;
  //   Eigen::SelfAdjointEigenSolver<const Eigen::MatrixXd> eigenSolver2(D_d0);
  //   std::cout << "D_d0 eigenvalues: \n"
  //             << eigenSolver2.eigenvalues() << std::endl;

  const Eigen::MatrixXd D_d = Q * D_d0 * Q.transpose();
  //   Eigen::SelfAdjointEigenSolver<const Eigen::MatrixXd> eigenSolver3(D_d);
  //   std::cout << "D_d eigenvalues: \n" << eigenSolver3.eigenvalues() <<
  //   std::endl;

  return D_d;
}