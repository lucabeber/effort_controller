#include <kuka_cartesian_impedance_controller/kuka_cartesian_impedance_controller.h>

namespace kuka_cartesian_impedance_controller {

KukaCartesianImpedanceController::KukaCartesianImpedanceController()
    : Base::EffortControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaCartesianImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaCartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }


  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 3,
          std::bind(&KukaCartesianImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));

  m_data_publisher = get_node()->create_publisher<debug_msg::msg::Debug>(
      get_node()->get_name() + std::string("/data"), 1);

#if LOGGING
  XBot::MatLogger2::Options opt;
  opt.default_buffer_size = 5e8; // set default buffer size
  opt.enable_compression = true;
  RCLCPP_INFO(get_node()->get_logger(), "\n\nCreating logger\n\n");
  m_logger = XBot::MatLogger2::MakeLogger("/tmp/cart_impedance.mat", opt);
  // m_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
  m_logger_appender = XBot::MatAppender::MakeInstance();
  m_logger_appender->add_logger(m_logger);
  m_logger_appender->start_flush_thread();

  // subscribe to lbr_fri_idl/msg/LBRState
  m_state_subscriber =
      get_node()->create_subscription<lbr_fri_idl::msg::LBRState>(
          "/lbr/state", 1,
          std::bind(&KukaCartesianImpedanceController::stateCallback, this,
                    std::placeholders::_1));
#endif

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}
#if LOGGING
void KukaCartesianImpedanceController::stateCallback(
    const lbr_fri_idl::msg::LBRState::SharedPtr state) {
  m_state = *state;
  // m_logger->add("commanded_torque:", state->commanded_torque.data);
  // m_logger->
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaCartesianImpedanceController::on_activate(
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

  m_last_time = get_node()->get_clock()->now().seconds();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaCartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  // call logger destructor
#if LOGGING
  // m_logger.reset();
  RCLCPP_WARN(get_node()->get_logger(), "\n\nFlushing logger\n\n");
#endif
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  // Base::writeJointEffortCmds(ctrl::VectorND::Zero());
  Base::on_deactivate(previous_state);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type
KukaCartesianImpedanceController::update(const rclcpp::Time &time,
                                     const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  computeTargetPos();
  // Write final commands to the hardware interface
  Base::writeJointEffortCmds(m_target_joint_position);

  return controller_interface::return_type::OK;
}

ctrl::Vector6D KukaCartesianImpedanceController::computeMotionError() {
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
void KukaCartesianImpedanceController::computeTargetPos() {
  Eigen::MatrixXd JJt;
  KDL::Jacobian J(Base::m_joint_number);

  KDL::JntArray q(Base::m_joint_number);
  q.data = Base::m_joint_positions.data;

  KDL::JntArray dq(Base::m_joint_number);
  dq.data.setZero();

  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 0.1;
  const double damp = 1e-12;

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;

  for (int i = 0;; i++) {
    // Compute forward kinematics
    Base::m_fk_solver->JntToCart(q, m_current_frame);
    // Compute error (logarithm map from desired frame to current)
    KDL::Twist delta_twist = KDL::diff(m_current_frame, m_target_frame);
    for (int j = 0; j < 6; ++j) {
      err(j) = delta_twist[j];
    }

    // Check for convergence
    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= IT_MAX) {
      success = false;
      break;
    }

    // Compute the Jacobian at the current confiation
    Base::m_jnt_to_jac_solver->JntToJac(q, J);

    // Compute damped least squares inverse
    JJt = J.data * J.data.transpose();
    JJt.diagonal().array() += damp;

    // Compute velocity update considering also the nullspace
    // dq = J^T(JJ^T)^-1 * e + (I - J^T(JJ^T)^-1 * J) * dt(q_starting_pose - q)
    dq.data = J.data.transpose() * JJt.ldlt().solve(err); //+ (m_identity - J.data.transpose() * JJt.ldlt().solve(J.data)) * (m_q_starting_pose - q.data) * DT;
    // Integrate joint velocities (Euler integration)
    for (int j = 0; j < Base::m_joint_number; j++) {
      q(j) += dq.data(j) * DT;
    }

    // Logging every 10 iterations
    if (i % 10 == 0) {
      std::cout << i << ": error = " << err.transpose()
                << " | err norm = " << err.norm() << std::endl;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Error: %f", err.norm());
  }

  m_target_joint_position = q.data;

#if LOGGING
  m_logger->add("time_sec", get_node()->get_clock()->now().seconds());
  m_logger->add("time_nsec", get_node()->get_clock()->now().nanoseconds());
  // add cartesian traj
  m_logger->add("cart_target_x", m_target_frame.p.x());
  m_logger->add("cart_target_y", m_target_frame.p.y());
  m_logger->add("cart_target_z", m_target_frame.p.z());
  Eigen::Matrix<double, 3, 3> rot_des(m_target_frame.M.data);
  m_logger->add("cart_target_M", rot_des);
  m_logger->add("cart_current_x", m_current_frame.p.x());
  m_logger->add("cart_current_y", m_current_frame.p.y());
  m_logger->add("cart_current_z", m_current_frame.p.z());
  Eigen::Matrix<double, 3, 3> M_c(m_current_frame.M.data);
  m_logger->add("cart_current_M", M_c);
  // solver outcome
  m_logger->add("joint_current", Base::m_joint_positions.data);
  m_logger->add("joint_target", m_target_joint_position);

  std::vector<double> measured_torque(std::begin(m_state.measured_torque),
                                      std::end(m_state.measured_torque));
  m_logger->add("measured_torque", measured_torque);
  std::vector<double> commanded_torque(std::begin(m_state.commanded_torque),
                                       std::end(m_state.commanded_torque));
  m_logger->add("commanded_torque", commanded_torque);

  auto compute_condition_number = [](const Eigen::MatrixXd &matrix) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
    Eigen::VectorXd singular_values = svd.singularValues();
    return singular_values(0) / singular_values(singular_values.size() - 1);
  };
  m_logger->add("condition_number_jac", compute_condition_number(J.data));
  m_logger->flush_available_data();
#endif
}


void KukaCartesianImpedanceController::targetFrameCallback(
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
} // namespace cartesian_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    kuka_cartesian_impedance_controller::KukaCartesianImpedanceController,
    controller_interface::ControllerInterface)
