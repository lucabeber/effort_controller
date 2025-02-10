#include <cartesian_impedance_controller/cartesian_impedance_controller.h>

namespace cartesian_impedance_controller {

CartesianImpedanceController::CartesianImpedanceController()
    : Base::EffortControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);
  auto_declare<double>("nullspace_stiffness", 0.0);
  auto_declare<bool>("compensate_dJdq", false);

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_configure(
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
  ctrl::Vector6D tmp;
  tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  m_cartesian_stiffness = tmp.asDiagonal();

  // Set damping
  tmp[0] = 2 * sqrt(tmp[0]);
  tmp[1] = 2 * sqrt(tmp[1]);
  tmp[2] = 2 * sqrt(tmp[2]);
  tmp[3] = 2 * sqrt(tmp[3]);
  tmp[4] = 2 * sqrt(tmp[4]);
  tmp[5] = 2 * sqrt(tmp[5]);

  // m_cartesian_damping = tmp.asDiagonal();

  // Set nullspace stiffness
  m_null_space_stiffness =
      get_node()->get_parameter("nullspace_stiffness").as_double();
  RCLCPP_INFO(get_node()->get_logger(), "Postural task stiffness: %f",
              m_null_space_stiffness);

  m_compensate_dJdq = get_node()->get_parameter("compensate_dJdq").as_bool();
  RCLCPP_INFO(get_node()->get_logger(), "Compensate dJdq: %d",
              m_compensate_dJdq);
  // Set nullspace damping
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

  // Set the identity matrix with dimension of the joint space
  m_identity = ctrl::MatrixND::Identity(m_joint_number, m_joint_number);
  // Make sure sensor wrenches are interpreted correctly
  // setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/target_wrench"), 10,
          std::bind(&CartesianImpedanceController::targetWrenchCallback, this,
                    std::placeholders::_1));

  // m_ft_sensor_wrench_subscriber =
  //   get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     get_node()->get_name() + std::string("/ft_sensor_wrench"),
  //     10,
  //     std::bind(&CartesianImpedanceController::ftSensorWrenchCallback, this,
  //     std::placeholders::_1));

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 3,
          std::bind(&CartesianImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));
  m_data_publisher = get_node()->create_publisher<debug_msg::msg::Debug>(
      get_node()->get_name() + std::string("/data"), 1);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set the target frame to the current frame
  m_target_frame = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  m_target_joint_position = m_q_starting_pose = Base::m_joint_positions.data;

  m_target_wrench = ctrl::Vector6D::Zero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  // Base::writeJointEffortCmds(ctrl::VectorND::Zero());
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  // Compute the torque to applay at the joints
  // ctrl::VectorND tau_tot = computeTorque();

  // Saturation of the torque
  // Base::computeJointEffortCmds(tau_tot);

  computeTargetPos();
  // Write final commands to the hardware interface
  Base::writeJointEffortCmds(m_target_joint_position);

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianImpedanceController::computeMotionError() {
  // Compute the cartesian error between the current and the target frame

  // Transformation from target -> current corresponds to error = target -
  // current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = error_kdl.M.GetRotAngle(rot_axis);  // rot_axis is normalized
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
void CartesianImpedanceController::computeTargetPos() {
  ctrl::VectorND desired_joint_position;
  Base::computeIKSolution(m_target_frame, desired_joint_position);
  // filter result with exp filter
  double const alpha = 0.8;
  ctrl::VectorND current_position = Base::m_joint_positions.data;
  m_target_joint_position =
      alpha * current_position + (1.0 - alpha) * desired_joint_position;
}

ctrl::VectorND CartesianImpedanceController::computeTorque() {
  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  debug_msg::msg::Debug debug_msg;

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions,
                                      Base::m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::MatrixND jac = Base::m_jacobian.data;
  ctrl::MatrixND jac_tran_pseudo_inverse;

  pseudoInverse(jac.transpose(), &jac_tran_pseudo_inverse);

  KDL::JntSpaceInertiaMatrix M(Base::m_joint_number);
  m_dyn_solver->JntToMass(Base::m_joint_positions, M);

  Eigen::MatrixXd Lambda = (jac * M.data.inverse() * jac.transpose()).inverse();

  // Redefine joints velocities in Eigen format
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  ctrl::VectorND q_null_space(Base::m_joint_number);

  // Compute the motion error
  ctrl::Vector6D motion_error = computeMotionError();

  // Initialize the torque vectors
  ctrl::VectorND tau_task(Base::m_joint_number), tau_null(Base::m_joint_number),
      tau_ext(Base::m_joint_number), tau_task_old(Base::m_joint_number),
      tau(Base::m_joint_number);

  // init tau to zero
  tau.setZero();
  tau_ext.setZero();
  tau_task_old.setZero();
  tau_task.setZero();
  tau_null.setZero();

  // Compute the stiffness and damping in the base link
  const auto base_link_stiffness =
      Base::displayInBaseLink(m_cartesian_stiffness, Base::m_end_effector_link);

  Eigen::MatrixXd K_d = base_link_stiffness;
  Eigen::VectorXd damping_correction = 5.0 * Eigen::VectorXd::Ones(6);
  auto D_d = compute_correct_damping(Lambda, K_d, 1.0);

  // add a small damping correction to the diagonal of D_d to account for model
  // inaccuracies, remove this loop if you face strange behavior
  for (int i = 3; i < 6; i++) {
    D_d(i, i) = D_d(i, i) + damping_correction(3);
  }
  Eigen::VectorXd stiffness_torque = jac.transpose() * (K_d * motion_error);
  Eigen::VectorXd damping_torque = jac.transpose() * -(D_d * (jac * q_dot));

  // Compute the task torque
  tau_task = stiffness_torque + damping_torque;

  KDL::JntArray tau_coriolis(Base::m_joint_number),
      tau_gravity(Base::m_joint_number);

  if (m_compensate_gravity) {
    Base::m_dyn_solver->JntToGravity(Base::m_joint_positions, tau_gravity);
    tau = tau + tau_gravity.data;
  }
  if (m_compensate_coriolis) {
    Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions,
                                      Base::m_joint_velocities, tau_coriolis);
    tau = tau + tau_coriolis.data;
  }
  // Computes the Jacobian derivative * q_dot, negligible for most of the robot
  if (m_compensate_dJdq) {
    KDL::JntArrayVel q_in(Base::m_joint_positions, Base::m_joint_velocities);
    KDL::Twist jac_dot_q_dot;
    Base::m_jnt_to_jac_dot_solver->JntToJacDot(q_in, jac_dot_q_dot);
    // convert KDL::Twist to Eigen::VectorXd
    Eigen::VectorXd jac_dot_q_dot_eigen(6);
    jac_dot_q_dot_eigen.head(3) << jac_dot_q_dot.vel.x(), jac_dot_q_dot.vel.y(),
        jac_dot_q_dot.vel.z();
    jac_dot_q_dot_eigen.tail(3) << jac_dot_q_dot.rot.x(), jac_dot_q_dot.rot.y(),
        jac_dot_q_dot.rot.z();
    Eigen::VectorXd j_tran_lambda_jdot_qdot =
        jac.transpose() * Lambda * jac_dot_q_dot_eigen;
    tau = tau + j_tran_lambda_jdot_qdot;
  }

  // Compute the null space torque
  q_null_space = m_q_starting_pose;
  if (m_null_space_stiffness > 1e-6) {
    // Compute dynamically consistent null space projector
    tau_null =
        (m_identity - jac.transpose() * Lambda * jac * M.data.inverse()) *
        (m_null_space_stiffness * (-q + q_null_space) -
         m_null_space_damping * q_dot);
  } else {
    tau_null = ctrl::VectorND::Zero(Base::m_joint_number);
  }

#if DEBUG
  Eigen::VectorXd Force = K_d * motion_error - D_d * (jac * q_dot);
  for (int i = 0; i < 7; i++) {
    debug_msg.stiffness_torque[i] = stiffness_torque(i);
    debug_msg.damping_torque[i] = damping_torque(i);
    debug_msg.coriolis_torque[i] = tau_coriolis(i);
    debug_msg.nullspace_torque[i] = tau_null(i);
    if (i < 6) {
      debug_msg.impedance_force[i] = Force(i);
    }
  }
  m_data_publisher->publish(debug_msg);
#endif
  // Compute the torque to achieve the desired force
  tau_ext = jac.transpose() * m_target_wrench;

  tau += tau_task + tau_null + tau_ext;

  return tau;
}

void CartesianImpedanceController::targetWrenchCallback(
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

void CartesianImpedanceController::targetFrameCallback(
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
}  // namespace cartesian_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    cartesian_impedance_controller::CartesianImpedanceController,
    controller_interface::ControllerInterface)
