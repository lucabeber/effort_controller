/*
    Computes the Jacobian time derivative
    Copyright (C) 2015  Antoine Hoarau <hoarau [at] isir.upmc.fr>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
   USA
*/

#ifndef KDL_CHAINJNTTOJACDOTSOLVER_HPP
#define KDL_CHAINJNTTOJACDOTSOLVER_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/solveri.hpp>

namespace KDL {

class ChainJntToJacDotSolver : public SolverI {
public:
  static const int E_JAC_DOT_FAILED = -100;
  static const int E_JACSOLVER_FAILED = -101;
  static const int E_FKSOLVERPOS_FAILED = -102;

  // Hybrid representation ref Frame: base, ref Point: end-effector
  static const int HYBRID = 0;
  // Body-fixed representation ref Frame: end-effector, ref Point: end-effector
  static const int BODYFIXED = 1;
  // Inertial representation ref Frame: base, ref Point: base
  static const int INERTIAL = 2;

  explicit ChainJntToJacDotSolver(const Chain &chain);
  virtual ~ChainJntToJacDotSolver();
  virtual int JntToJacDot(const KDL::JntArrayVel &q_in,
                          KDL::Twist &jac_dot_q_dot, int seg_nr = -1);
  virtual int JntToJacDot(const KDL::JntArrayVel &q_in, KDL::Jacobian &jdot,
                          int seg_nr = -1);
  int setLockedJoints(const std::vector<bool> &locked_joints);

  void setHybridRepresentation() { setRepresentation(HYBRID); }
  void setBodyFixedRepresentation() { setRepresentation(BODYFIXED); }
  void setInertialRepresentation() { setRepresentation(INERTIAL); }
  void setRepresentation(const int &representation);

  virtual void updateInternalDataStructures();

  virtual const char *strError(const int error) const;

protected:
  const Twist &getPartialDerivativeHybrid(const Jacobian &bs_J_ee,
                                          const unsigned int &joint_idx,
                                          const unsigned int &column_idx);
  const Twist &getPartialDerivativeBodyFixed(const Jacobian &ee_J_ee,
                                             const unsigned int &joint_idx,
                                             const unsigned int &column_idx);
  const Twist &getPartialDerivativeInertial(const Jacobian &bs_J_bs,
                                            const unsigned int &joint_idx,
                                            const unsigned int &column_idx);
  const Twist &getPartialDerivative(const Jacobian &J,
                                    const unsigned int &joint_idx,
                                    const unsigned int &column_idx,
                                    const int &representation);

private:
  const Chain &chain;
  std::vector<bool> locked_joints_;
  unsigned int nr_of_unlocked_joints_;
  ChainJntToJacSolver jac_solver_;
  Jacobian jac_;
  Jacobian jac_dot_;
  int representation_;
  ChainFkSolverPos_recursive fk_solver_;
  Frame F_bs_ee_;
  Twist jac_dot_k_;
  Twist jac_j_, jac_i_;
  Twist t_djdq_;
};

} // namespace KDL
#endif