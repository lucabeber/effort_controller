#include "cbf_cartesian_impedance_controller/cbf_cartesian_impedance_controller.hpp"

#include "cbf_cartesian_impedance_controller/pseudo_inversion.h"
#include "controller_interface/controller_interface.hpp"
#include "effort_controller_base/Utility.h"

namespace cbf_cartesian_impedance_controller {

CBFCartesianImpedanceController::CBFCartesianImpedanceController()
    : Base::EffortControllerBase() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CBFCartesianImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<double>("nullspace_stiffness", 0.0);

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
  ;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CBFCartesianImpedanceController::on_configure(
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

  m_cartesian_damping = tmp.asDiagonal();

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
          std::bind(&CBFCartesianImpedanceController::targetWrenchCallback,
                    this, std::placeholders::_1));

  // m_ft_sensor_wrench_subscriber =
  //   get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     get_node()->get_name() + std::string("/ft_sensor_wrench"),
  //     10,
  //     std::bind(&CBFCartesianImpedanceController::ftSensorWrenchCallback,
  //     this, std::placeholders::_1));

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 3,
          std::bind(&CBFCartesianImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));
  m_logger_publisher =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/cbf_log",
                                                                     10);

  m_marker_pub = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization_marker", 1);
  // enable shared memory
  m_visualizer =
      std::make_shared<Visualizer>(m_marker_pub, Base::m_robot_base_link, 1.0);


  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CBFCartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set the target frame to the current frame
  m_target_frame = m_old_target_frame = m_filtered_target = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  m_q_starting_pose = Base::m_joint_positions.data;

  // Initialize the old torque to zero
  m_tau_old = ctrl::VectorND::Zero(Base::m_joint_number);

  m_old_rot_error = ctrl::Vector3D::Zero();

  m_old_vel_error = ctrl::VectorND::Zero(Base::m_joint_number);

  m_target_wrench = ctrl::Vector6D::Zero();

  m_last_time = get_node()->get_clock()->now();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CBFCartesianImpedanceController::on_deactivate(
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
CBFCartesianImpedanceController::update(const rclcpp::Time &time,
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

ctrl::Vector6D CBFCartesianImpedanceController::computeMotionError() {
  // Compute the cartesian error between the current and the target frame

  // Transformation from target -> current corresponds to error = target -
  // current
  KDL::Frame error_kdl;
  error_kdl.M = m_filtered_target.M * m_current_frame.M.Inverse();
  error_kdl.p = m_filtered_target.p - m_current_frame.p;

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

ctrl::VectorND CBFCartesianImpedanceController::computeTorque() {
  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions,
                                      Base::m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::MatrixND jac = Base::m_jacobian.data;
  ctrl::MatrixND jac_tran_pseudo_inverse;

  pseudoInverse(jac.transpose(), &jac_tran_pseudo_inverse);

  // Redefine joints velocities in Eigen format
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  ctrl::VectorND q_null_space = Base::m_simulated_joint_motion.data;

  auto current_time = get_node()->get_clock()->now();
  double dt = (current_time - m_last_time).seconds();

  // filter position
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> constraint_planes;

  constraint_planes.push_back(std::make_pair(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ() * 0.3));
  Eigen::Vector3d plane(0.0, 1.0, 0.0);
  Eigen::Vector3d point(0.0, -0.2, 0.0);
  constraint_planes.push_back(std::make_pair(plane, point));
  plane << 0.0, -1.0, 0.0;
  point << 0.0, 0.2, 0.0;
  constraint_planes.push_back(std::make_pair(plane, point));

  std::vector<Eigen::Vector3d> n;
  std::vector<Eigen::Vector3d> p;
  for (size_t i = 0; i < constraint_planes.size(); i++) {
    n.push_back(constraint_planes[i].first);
    p.push_back(constraint_planes[i].second);
  }

  std::vector<double> logs;
  auto logs1 =
      planes_cbf::cbfPositionFilter(m_filtered_target, m_target_frame, n, p);

  // update every 0.02s
  if (vis_iter == 0) {
    Eigen::Vector3d target(m_target_frame.p.x(), m_target_frame.p.y(),
                           m_target_frame.p.z());
    Eigen::Vector3d filtered_target(m_filtered_target.p.x(),
                                    m_filtered_target.p.y(),
                                    m_filtered_target.p.z());
    m_visualizer->draw_scene(constraint_planes, target, filtered_target, current_time, 0.015);
  }
  vis_iter = (vis_iter + 1) % 20;

  // set limits (radiants) from initial orientation
  Eigen::Vector3d thetas(0.4, 0.4, 0.4);
  if (m_received_initial_frame) {
    logs = conic_cbf::cbfOrientFilter(m_initial_frame, m_filtered_target,
                                      m_target_frame, thetas, dt);
  }
  m_last_time = current_time;
  // logs.push_back(current_time.seconds());  // 4
  std_msgs::msg::Float64MultiArray msg;
  msg.data = logs;
  m_logger_publisher->publish(msg);

  // Compute the motion error
  ctrl::Vector6D motion_error = computeMotionError();

  // Initialize the torque vectors
  ctrl::VectorND tau_task(Base::m_joint_number), tau_null(Base::m_joint_number),
      tau_ext(Base::m_joint_number);

  // Filter the velocity errorm_old_vel_error
  // q_dot = m_alpha * q_dot + (1 - m_alpha) * m_old_vel_error;
  // for (int i = 0; i < q_dot.size(); i++) {
  //   q_dot(i) = std::round(q_dot(i) * 1000) / 1000;
  // }
  // m_old_vel_error = q_dot;

  // Compute the stiffness and damping in the base link
  const auto base_link_stiffness =
      Base::displayInBaseLink(m_cartesian_stiffness, Base::m_end_effector_link);
  const auto base_link_damping =
      Base::displayInBaseLink(m_cartesian_damping, Base::m_end_effector_link);

  // Compute the task torque
  tau_task = jac.transpose() * (base_link_stiffness * motion_error -
                                (base_link_damping * (jac * q_dot)));

  // Compute the null space torque
  q_null_space = m_q_starting_pose;
  tau_null = (m_identity - jac.transpose() * jac_tran_pseudo_inverse) *
             (m_null_space_stiffness * (-q + q_null_space) -
              m_null_space_damping * q_dot);

  // Compute the torque to achieve the desired force
  tau_ext = jac.transpose() * m_target_wrench;

  ctrl::VectorND tau = tau_task + tau_null + tau_ext;

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
  return tau;
}

void CBFCartesianImpedanceController::targetWrenchCallback(
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

void CBFCartesianImpedanceController::targetFrameCallback(
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
  if (!m_received_initial_frame) {
    m_initial_frame = m_target_frame;
    m_filtered_target = m_target_frame;
    m_received_initial_frame = true;
  }
}
} // namespace cbf_cartesian_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    cbf_cartesian_impedance_controller::CBFCartesianImpedanceController,
    controller_interface::ControllerInterface)
