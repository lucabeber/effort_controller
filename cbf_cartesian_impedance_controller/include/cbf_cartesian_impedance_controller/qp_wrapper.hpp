#ifndef QPROS2_QPOASES_EIGEN_WRAPPER_HPP
#define QPROS2_QPOASES_EIGEN_WRAPPER_HPP
#ifdef __GNUC__
// Avoid warnings
#pragma GCC system_header
#endif
#include <Eigen/Dense>

#include "qpOASES/MessageHandling.hpp"
#include "qpOASES/SQProblem.hpp"
#include "qpOASES/Types.hpp"
using real_t = qpOASES::real_t;
using qpH_t = Eigen::Matrix<real_t, 1, 1, Eigen::RowMajor>;
using qpx_t = Eigen::Vector<real_t, 3>;  // deltax
using qpg_t = qpx_t;                     // -2 deltaX_des
using qpxBounds_t = qpx_t;               // -inf +inf

#endif  // QPROS2_QPOASES_EIGEN_WRAPPER_HPP