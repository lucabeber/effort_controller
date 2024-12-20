#include "controller_interface/controller_interface.hpp"
#include "effort_controller_base/Utility.h"
#include <joint_impedance_controller/joint_impedance_controller.h>

#include <joint_impedance_controller/pseudo_inversion.h>

namespace joint_impedance_controller {

JointImpedanceController::JointImpedanceController()
    : Base::EffortControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);
  auto_declare<double>("nullspace_stiffness", 0.0);

  constexpr double default_joint_stiff = 100.0;

  for (size_t i = 1; i <= 7; i++) {
    auto_declare<double>("stiffness.joint" + std::to_string(i),
                         default_joint_stiff);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link =
      get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link
                            << " is not part of the kinematic chain from "
                            << Base::m_robot_base_link << " to "
                            << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }

  // Set stiffness
  ctrl::VectorND tmp(Base::m_joint_number);
  for (size_t i = 1; i <= Base::m_joint_number; i++) {
    tmp(i - 1) = get_node()
                     ->get_parameter("stiffness.joint" + std::to_string(i))
                     .as_double();
  }

  m_joint_stiffness = ctrl::VectorND::Zero(Base::m_joint_number);
  m_joint_stiffness = tmp;

  // Set damping
  m_joint_damping = ctrl::VectorND::Zero(Base::m_joint_number);
  m_joint_damping = 2 * m_joint_stiffness.cwiseSqrt();

  // Set nullspace stiffness
  m_null_space_stiffness =
      get_node()->get_parameter("nullspace_stiffness").as_double();
  RCLCPP_INFO(get_node()->get_logger(), "Postural task stiffness: %f",
              m_null_space_stiffness);

  // Set nullspace damping
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

  // Set the identity matrix with dimension of the joint space
  m_identity = ctrl::MatrixND::Identity(m_joint_number, m_joint_number);

  // Make sure sensor wrenches are interpreted correctly
  // setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/target_wrench"), 10,
          std::bind(&JointImpedanceController::targetWrenchCallback, this,
                    std::placeholders::_1));

  // m_ft_sensor_wrench_subscriber =
  //   get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     get_node()->get_name() + std::string("/ft_sensor_wrench"),
  //     10,
  //     std::bind(&JointImpedanceController::ftSensorWrenchCallback, this,
  //     std::placeholders::_1));

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 3,
          std::bind(&JointImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set the target frame to the current frame
  m_target_frame = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  m_q_desired = Base::m_joint_positions.data;
  m_q_starting_pose = Base::m_joint_positions.data;

  // Initialize the old torque to zero
  m_tau_old = ctrl::VectorND::Zero(Base::m_joint_number);

  m_old_rot_error = ctrl::Vector3D::Zero();

  m_old_vel_error = ctrl::VectorND::Zero(Base::m_joint_number);

  m_target_wrench = ctrl::Vector6D::Zero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  Base::writeJointEffortCmds();
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type
JointImpedanceController::update(const rclcpp::Time &time,
                                 const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  // Compute the torque to applay at the joints
  ctrl::VectorND tau_tot = computeTorque();

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D JointImpedanceController::computeMotionError() {
  // Compute the cartesian error between the current and the target frame

  // Transformation from target -> current corresponds to error = target -
  // current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = error_kdl.M.GetRotAngle(rot_axis); // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle = std::clamp(angle, -max_angle, max_angle);
  distance = std::clamp(distance, -max_distance, max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error.head<3>() << error_kdl.p.x(), error_kdl.p.y(), error_kdl.p.z();
  error.tail<3>() << rot_axis(0), rot_axis(1), rot_axis(2);

  return error;
}

ctrl::VectorND JointImpedanceController::computeTorque() {
  // Find the desired joints positions
  // Base::computeNullSpace(m_target_frame);

  // Compute the inverse kinematics
  Base::computeIKSolution(m_target_frame, m_q_desired);

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions,
                                      Base::m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::MatrixND jac = Base::m_jacobian.data;
  ctrl::MatrixND jac_tran_pseudo_inverse;

  pseudoInverse(jac.transpose(), &jac_tran_pseudo_inverse);

  // Compute the desired joint positions

  // Redefine joints velocities in Eigen format
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  // ctrl::VectorND q_null_space = Base::m_simulated_joint_motion.data;

  // // Compute the motion error
  // ctrl::Vector6D motion_error = computeMotionError();

  // // Initialize the torque vectors
  // ctrl::VectorND tau_task(Base::m_joint_number),
  // tau_null(Base::m_joint_number),
  //     tau_ext(Base::m_joint_number);

  // q_dot = m_alpha * q_dot + (1 - m_alpha) * m_old_vel_error;

  // q_dot(0) = std::round(q_dot(0) * 1000) / 1000;
  // q_dot(1) = std::round(q_dot(1) * 1000) / 1000;
  // q_dot(2) = std::round(q_dot(2) * 1000) / 1000;
  // q_dot(3) = std::round(q_dot(3) * 1000) / 1000;
  // q_dot(4) = std::round(q_dot(4) * 1000) / 1000;
  // q_dot(5) = std::round(q_dot(5) * 1000) / 1000;
  // q_dot(6) = std::round(q_dot(6) * 1000) / 1000;

  // m_old_vel_error = q_dot;

  // // Compute the stiffness and damping in the base link
  // const auto base_link_stiffness =
  // Base::displayInBaseLink(m_cartesian_stiffness, Base::m_end_effector_link);
  // const auto base_link_damping = Base::displayInBaseLink(m_cartesian_damping,
  // Base::m_end_effector_link);

  // // Compute the task torque
  // tau_task = jac.transpose() * (base_link_stiffness * motion_error -
  // (base_link_damping * (jac * q_dot)));

  // // Compute the null space torque
  // q_null_space = m_q_starting_pose;
  // tau_null = (m_identity - jac.transpose() * jac_tran_pseudo_inverse) *
  //            (m_null_space_stiffness * (-q + q_null_space) -
  //             m_null_space_damping * q_dot);

  // // Compute the torque to achieve the desired force
  // tau_ext = jac.transpose() * m_target_wrench;

  ctrl::VectorND tau_task(Base::m_joint_number);

  // Compute the desired joint torques
  for (size_t i = 0; i < Base::m_joint_number; i++) {
    tau_task(i) = m_joint_stiffness(i) * (m_q_desired(i) - q(i)) -
                  m_joint_damping(i) * q_dot(i);
  }
  return tau_task;
}

void JointImpedanceController::targetWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  // Parse the target wrench
  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;

  // Check if the wrench is given in the base frame
  if (wrench->header.frame_id != Base::m_robot_base_link) {
    // Transform the wrench to the base frame
    m_target_wrench =
        Base::displayInBaseLink(m_target_wrench, wrench->header.frame_id);
  }
}

void JointImpedanceController::targetFrameCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr target) {
  if (target->header.frame_id != Base::m_robot_base_link) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "Got target pose in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  m_target_frame =
      KDL::Frame(KDL::Rotation::Quaternion(
                     target->pose.orientation.x, target->pose.orientation.y,
                     target->pose.orientation.z, target->pose.orientation.w),
                 KDL::Vector(target->pose.position.x, target->pose.position.y,
                             target->pose.position.z));
}
} // namespace joint_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(joint_impedance_controller::JointImpedanceController,
                       controller_interface::ControllerInterface)
