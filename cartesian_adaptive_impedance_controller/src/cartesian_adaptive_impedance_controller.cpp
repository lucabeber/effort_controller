////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_adaptive_impedance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "effort_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <cartesian_adaptive_impedance_controller/cartesian_adaptive_impedance_controller.h>


namespace cartesian_adaptive_impedance_controller
{

CartesianAdaptiveImpedanceController::CartesianAdaptiveImpedanceController()
// Base constructor won't be called in diamond inheritance, so call that
// explicitly
: Base::EffortControllerBase(),
  ImpedanceBase::CartesianImpedanceController()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianAdaptiveImpedanceController::on_init()
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (ImpedanceBase::on_init() != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  auto_declare<std::string>("compliance_ref_link", "");

  constexpr double default_stiff_max = 1000.0;
  auto_declare<double>("stiffness_max.trans_x", default_stiff_max);
  auto_declare<double>("stiffness_max.trans_y", default_stiff_max);
  auto_declare<double>("stiffness_max.trans_z", default_stiff_max);

  constexpr double default_stiff_min = 200.0;
  auto_declare<double>("stiffness_min.trans_x", default_stiff_min);
  auto_declare<double>("stiffness_min.trans_y", default_stiff_min);
  auto_declare<double>("stiffness_min.trans_z", default_stiff_min);

  constexpr double default_force_max = 60.0;
  auto_declare<double>("force_max.trans_x", default_force_max);
  auto_declare<double>("force_max.trans_y", default_force_max);
  auto_declare<double>("force_max.trans_z", default_force_max);

  constexpr double default_Q = 3200;
  auto_declare<double>("Q", default_Q);

  constexpr double default_R = 1;
  auto_declare<double>("R", default_R);

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianAdaptiveImpedanceController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (ImpedanceBase::on_configure(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // Make sure compliance link is part of the robot chain
  m_compliance_ref_link = get_node()->get_parameter("compliance_ref_link").as_string();
  if(!Base::robotChainContains(m_compliance_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_compliance_ref_link << " is not part of the kinematic chain from "
                                              << Base::m_robot_base_link << " to "
                                              << Base::m_end_effector_link);
    return TYPE::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  ImpedanceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

  // Set max stiffness
  m_stiffness_max(0) = get_node()->get_parameter("stiffness_max.trans_x").as_double();
  m_stiffness_max(1) = get_node()->get_parameter("stiffness_max.trans_y").as_double();
  m_stiffness_max(2) = get_node()->get_parameter("stiffness_max.trans_z").as_double();

  // Set min stiffness
  m_stiffness_min(0) = get_node()->get_parameter("stiffness_min.trans_x").as_double();
  m_stiffness_min(1) = get_node()->get_parameter("stiffness_min.trans_y").as_double();
  m_stiffness_min(2) = get_node()->get_parameter("stiffness_min.trans_z").as_double();

  // Set max force
  m_force_max(0) = get_node()->get_parameter("force_max.trans_x").as_double();
  m_force_max(1) = get_node()->get_parameter("force_max.trans_y").as_double();
  m_force_max(2) = get_node()->get_parameter("force_max.trans_z").as_double();

  // Set Q matrix
  m_Q_matrix(0, 0) = get_node()->get_parameter("Q").as_double();
  m_Q_matrix(1, 1) = get_node()->get_parameter("Q").as_double();
  m_Q_matrix(2, 2) = get_node()->get_parameter("Q").as_double();

  // Set R matrix
  m_R_matrix(0, 0) = get_node()->get_parameter("R").as_double();
  m_R_matrix(1, 1) = get_node()->get_parameter("R").as_double();
  m_R_matrix(2, 2) = get_node()->get_parameter("R").as_double();

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianAdaptiveImpedanceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Base::on_activation(..) will get called twice,
  // but that's fine.
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (ImpedanceBase::on_activate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // Reset external forces, desired forces and desired stiffness
  ctrl::Vector3D v = Eigen::Vector3d::Zero();
  ctrl::Matrix3D m = Eigen::Matrix3d::Zero();

  for(size_t i = 0; i < m_window_length; i++)
  {
    m_external_forces.push_back(v);
    m_desired_forces.push_back(v);
    m_desired_stiffness.push_back(m);
  }

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianAdaptiveImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (ImpedanceBase::on_deactivate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}


controller_interface::return_type CartesianAdaptiveImpedanceController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
{
  USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up QProblem object. */
	QProblem example( 2,1 );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	/* Get and print solution of first QP. */
	real_t xOpt[2];
	real_t yOpt[2+1];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
	
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

	example.printOptions();
  // // Synchronize the internal model and the real robot
  // Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // // Control the robot motion in such a way that the resulting net force
  // // vanishes. This internal control needs some simulation time steps.
  // for (int i = 0; i < Base::m_iterations; ++i)
  // {
  //   // The internal 'simulation time' is deliberately independent of the outer
  //   // control cycle.
  //   auto internal_period = rclcpp::Duration::from_seconds(0.02);

  //   // Compute the net force
  //   ctrl::Vector6D error = computeComplianceError();

  //   // Turn Cartesian error into joint motion
  //   Base::computeJointControlCmds(error,internal_period);
  // }

  // // Write final commands to the hardware interface
  // Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

// ctrl::Vector6D CartesianAdaptiveImpedanceController::computeComplianceError()
// {
//   ctrl::Vector6D tmp;
//   tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
//   tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
//   tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
//   tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
//   tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
//   tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

//   m_stiffness = tmp.asDiagonal();

//   ctrl::Vector6D net_force =

//     // Spring force in base orientation
//     Base::displayInBaseLink(m_stiffness,m_compliance_ref_link) * MotionBase::computeMotionError()

//     // Sensor and target force in base orientation
//     + ImpedanceBase::computeForceError();

//   return net_force;
// }

} // namespace


// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_adaptive_impedance_controller::CartesianAdaptiveImpedanceController, controller_interface::ControllerInterface)
