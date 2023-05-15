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
/*!\file    cartesian_impedance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "effort_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <cartesian_impedance_controller/cartesian_impedance_controller.h>

#include <cartesian_impedance_controller/pseudo_inversion.h>

namespace cartesian_impedance_controller
{

CartesianImpedanceController::CartesianImpedanceController()
: Base::EffortControllerBase(), m_hand_frame_control(true)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if(!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link << " is not part of the kinematic chain from "
                                             << Base::m_robot_base_link << " to "
                                             << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Set stiffness
  ctrl::Vector6D tmp;
  tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  m_cartesian_stiffness = tmp.asDiagonal();

  // Set damping
  tmp[0] = 2 * 0.707 * sqrt(tmp[0]);
  tmp[1] = 2 * 0.707 * sqrt(tmp[1]);
  tmp[2] = 2 * 0.707 * sqrt(tmp[2]);
  tmp[3] = 2 * 0.707 * sqrt(tmp[3]);
  tmp[4] = 2 * 0.707 * sqrt(tmp[4]);
  tmp[5] = 2 * 0.707 * sqrt(tmp[5]);

  m_cartesian_damping = tmp.asDiagonal();

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"),
    10,
    std::bind(&CartesianImpedanceController::targetWrenchCallback, this, std::placeholders::_1));

  // m_ft_sensor_wrench_subscriber =
  //   get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     get_node()->get_name() + std::string("/ft_sensor_wrench"),
  //     10,
  //     std::bind(&CartesianImpedanceController::ftSensorWrenchCallback, this, std::placeholders::_1));

  m_target_frame_subscriber = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        get_node()->get_name() + std::string("/target_frame"),
        3,
        std::bind(&CartesianImpedanceController::targetFrameCallback, this, std::placeholders::_1));

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  // Update joint states
  Base::updateJointStates();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  Base::writeJointEffortCmds();
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
{
    // Update joint states
  Base::updateJointStates();

  // Find the desired joints positions
  Base::computeNullSpace(m_target_frame, period)
  m_null_space_pose = Base::m_simulated_joint_motion;

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions, Base::m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::MatrixND jac = Base::m_jacobian.data;
  ctrl::MatrixND jac_pseudo_inverse;
  pseudoInverse(jac.transpose(), &jac_pseudo_inverse);

  // Redefine joints velocities in Eigen format
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  
  // Compute the motion error
  ctrl::Vector6D motion_error = computeMotionError();
  
  ctrl::VectorND tau_task(Base::m_joint_number), tau_null(Base::m_joint_number), tau_ext(Base::m_joint_number);

  // Torque calculation for task space
  tau_task = jac.transpose() * (m_cartesian_stiffness * motion_error + m_cartesian_damping * (jac * q_dot));

  // Torque calculation for null space
  tau_null = (ctrl::MatrixND::Identity(Base::m_joint_number) - jac * jac_pseuodo_inverse) *
                * (m_null_space_stiffness * (- Base::m_joint_positions + m_null_space_pose) - m_null_space_damping * Base::m_joint_velocities);

  // Torque calculation for external wrench
  tau_ext = jac.transpose() * m_target_wrench;

  // Final torque calculation
  ctrl::VectorND tau_tot = - tau_task + tau_null + tau_ext;

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();
  
  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianImpedanceController::computeForceError()
{
  ctrl::Vector6D target_wrench;
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control) // Assume end-effector frame by convention
  {
    target_wrench = Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link);
  }
  else // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench,m_new_ft_sensor_ref) + target_wrench;
}

ctrl::Vector6D CartesianImpedanceController::computeMotionError()
{
  // Redefine eigen vectors in kdl format
  KDL::Frame target_frame_kdl, current_frame_kdl;


  // Transformation from target -> current corresponds to error = target - current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle    = error_kdl.M.GetRotAngle(rot_axis);   // rot_axis is normalized

  rot_axis = rot_axis * angle;

  // Reassign values
  ctrl::Vector6D error;
  error(0) = error_kdl.p.x();
  error(1) = error_kdl.p.y();
  error(2) = error_kdl.p.z();
  error(3) = rot_axis(0);
  error(4) = rot_axis(1);
  error(5) = rot_axis(2);

  return error;
}
// void CartesianImpedanceController::setFtSensorReferenceFrame(const std::string& new_ref)
// {
//   // Compute static transform from the force torque sensor to the new reference
//   // frame of interest.
//   m_new_ft_sensor_ref = new_ref;

//   // Joint positions should cancel out, i.e. it doesn't matter as long as they
//   // are the same for both transformations.
//   KDL::JntArray jnts(Base::m_ik_solver->getPositions());

//   KDL::Frame sensor_ref;
//   Base::m_forward_kinematics_solver->JntToCart(
//       jnts,
//       sensor_ref,
//       m_ft_sensor_ref_link);

//   KDL::Frame new_sensor_ref;
//   Base::m_forward_kinematics_solver->JntToCart(
//       jnts,
//       new_sensor_ref,
//       m_new_ft_sensor_ref);

//   m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
// }

void CartesianImpedanceController::targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianImpedanceController::targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target)
{
  if (target->header.frame_id != Base::m_robot_base_link)
  {
    auto& clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
        clock, 3000,
        "Got target pose in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(),
        target->header.frame_id.c_str());
    return;
  }

  m_target_frame = KDL::Frame(
      KDL::Rotation::Quaternion(
        target->pose.orientation.x,
        target->pose.orientation.y,
        target->pose.orientation.z,
        target->pose.orientation.w),
      KDL::Vector(
        target->pose.position.x,
        target->pose.position.y,
        target->pose.position.z));
}
// void CartesianImpedanceController::ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
// {
//   KDL::Wrench tmp;
//   tmp[0] = wrench->wrench.force.x;
//   tmp[1] = wrench->wrench.force.y;
//   tmp[2] = wrench->wrench.force.z;
//   tmp[3] = wrench->wrench.torque.x;
//   tmp[4] = wrench->wrench.torque.y;
//   tmp[5] = wrench->wrench.torque.z;

//   // Compute how the measured wrench appears in the frame of interest.
//   tmp = m_ft_sensor_transform * tmp;

//   m_ft_sensor_wrench[0] = tmp[0];
//   m_ft_sensor_wrench[1] = tmp[1];
//   m_ft_sensor_wrench[2] = tmp[2];
//   m_ft_sensor_wrench[3] = tmp[3];
//   m_ft_sensor_wrench[4] = tmp[4];
//   m_ft_sensor_wrench[5] = tmp[5];
// }

}

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceController, controller_interface::ControllerInterface)
