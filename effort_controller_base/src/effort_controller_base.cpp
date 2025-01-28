#include <effort_controller_base/effort_controller_base.h>

namespace effort_controller_base {

EffortControllerBase::EffortControllerBase() {}

RobotDescriptionListener::RobotDescriptionListener(
    std::shared_ptr<std::string> robot_description_ptr,
    const std::string &topic_name)
    : Node("RobotDescriptionListener") {
  m_robot_description_ptr_ = robot_description_ptr;
  // Lamda callback for robot description
  auto descriptionCB =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void {
    *m_robot_description_ptr_ = msg->data;
    m_description_sub_.reset();
    RCLCPP_INFO(get_logger(), "Received robot description");
    m_description_received_ = true;
  };

  // set to qos to be RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
  auto durability_policy = rmw_qos_profile_default;
  durability_policy.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  auto qos = rclcpp::QoS(rclcpp::KeepAll(), durability_policy);
  m_description_sub_ = create_subscription<std_msgs::msg::String>(
      topic_name, qos, descriptionCB);
}

controller_interface::InterfaceConfiguration
EffortControllerBase::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * m_cmd_interface_types.size());
  for (const auto &type : m_cmd_interface_types) {
    for (const auto &joint_name : m_joint_names) {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
EffortControllerBase::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * m_state_interface_types.size());
  for (const auto &type : m_state_interface_types) {
    for (const auto &joint_name : m_joint_names) {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EffortControllerBase::on_init() {
  if (!m_initialized) {
    auto_declare<std::string>("ik_solver", "forward_dynamics");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_base_link", "");
    auto_declare<std::string>("compliance_ref_link", "");
    auto_declare<std::string>("end_effector_link", "");
    auto_declare<bool>("kuka_hw", false);
    auto_declare<bool>("compensate_gravity", false);
    auto_declare<bool>("compensate_coriolis", false);
    auto_declare<double>("delta_tau_max", 1.0);

    auto_declare<std::vector<std::string>>("joints",
                                           std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces",
                                           {hardware_interface::HW_IF_EFFORT});
    auto_declare<std::vector<std::string>>("state_interfaces",
                                           {hardware_interface::HW_IF_POSITION,
                                            hardware_interface::HW_IF_VELOCITY,
                                            hardware_interface::HW_IF_EFFORT});
    auto_declare<double>("solver.error_scale", 1.0);
    auto_declare<int>("solver.iterations", 1);
    m_initialized = true;
    std::string topic_name;
    RCLCPP_INFO(get_node()->get_logger(), "Namespace: %s",
                get_node()->get_namespace());
    // append namespace to robot_description topic
    if (std::string(get_node()->get_namespace()).compare("/") == 0) {
      topic_name = "robot_description";
      RCLCPP_INFO(get_node()->get_logger(), "A");

    } else {
      topic_name =
          std::string(get_node()->get_namespace()) + "/robot_description";
      RCLCPP_INFO(get_node()->get_logger(), "B");
    }
    // create shared pointer to robot description
    auto robot_description_ptr = std::make_shared<std::string>();
    auto robot_description_listener =
        std::make_shared<RobotDescriptionListener>(robot_description_ptr,
                                                   topic_name);
    // create executor to wait for robot description
    auto executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(robot_description_listener);
    while (!robot_description_listener->m_description_received_) {
      executor->spin_some();
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000, "Waiting for robot description");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    m_robot_description = *robot_description_ptr;
  }

  m_commanded_torque_publisher =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          get_node()->get_name() + std::string("/commanded_torque"), 1);
  m_measured_torque_publisher =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          get_node()->get_name() + std::string("/measured_torque"), 1);

  m_lbr_state_subscriber =
      get_node()->create_subscription<lbr_fri_idl::msg::LBRState>(
          std::string("state"), 1,
          [this](const lbr_fri_idl::msg::LBRState::SharedPtr msg) {
            std_msgs::msg::Float64MultiArray datas;
            for (size_t i = 0; i < m_joint_number; i++) {
              datas.data.push_back(-msg->commanded_torque[i]);
            }
            m_commanded_torque_publisher->publish(datas);
            datas.data.clear();
            for(size_t i = 0; i < m_joint_number; i++) {
              datas.data.push_back(-msg->measured_torque[i]);
            }
            m_measured_torque_publisher->publish(datas);
          });
  

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EffortControllerBase::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  if (m_configured) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  m_compensate_gravity =
      get_node()->get_parameter("compensate_gravity").as_bool();
  m_compensate_coriolis =
      get_node()->get_parameter("compensate_coriolis").as_bool();

  RCLCPP_WARN_STREAM(get_node()->get_logger(), "Gravity compensation set to "
                                                   << std::boolalpha
                                                   << m_compensate_gravity);
  RCLCPP_WARN_STREAM(get_node()->get_logger(), "Coriolis compensation set to "
                                                   << std::boolalpha
                                                   << m_compensate_coriolis);
  // // Get delta tau maximum
  m_delta_tau_max = get_node()->get_parameter("delta_tau_max").as_double();
  if (m_delta_tau_max < 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "delta_tau_max must be greater than 1.0 Nm");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }

  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree robot_tree;

  m_robot_base_link = get_node()->get_parameter("robot_base_link").as_string();
  if (m_robot_base_link.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  m_end_effector_link =
      get_node()->get_parameter("end_effector_link").as_string();
  if (m_end_effector_link.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  m_compliance_ref_link =
      get_node()->get_parameter("compliance_ref_link").as_string();
  if (m_compliance_ref_link.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "compliance_ref_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(m_robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to parse urdf model from 'robot_description'");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to parse KDL tree from urdf model");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(m_robot_base_link, m_end_effector_link,
                           m_robot_chain)) {
    const std::string error = ""
                              "Failed to parse robot chain from urdf model. "
                              "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(get_node()->get_logger(), error.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  if (!robotChainContains(m_compliance_ref_link)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_compliance_ref_link
                            << " is not part of the kinematic chain from "
                            << m_robot_base_link << " to "
                            << m_end_effector_link);
    return CallbackReturn::ERROR;
  }

  // Get names of actuated joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }

  // Initialize joint number
  m_joint_number = m_joint_names.size();

  m_jacobian.resize(m_joint_number);

  // Initialize effort limits
  m_joint_effort_limits.resize(m_joint_number);

  // Parse joint limits
  KDL::JntArray upper_pos_limits(m_joint_number);
  KDL::JntArray lower_pos_limits(m_joint_number);
  for (size_t i = 0; i < m_joint_number; ++i) {
    if (!robot_model.getJoint(m_joint_names[i])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint %s does not appear in robot_description",
                   m_joint_names[i].c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::ERROR;
    }
    if (robot_model.getJoint(m_joint_names[i])->type ==
        urdf::Joint::CONTINUOUS) {
      upper_pos_limits(i) = std::nan("0");
      lower_pos_limits(i) = std::nan("0");
      m_joint_effort_limits(i) = std::nan("0");
    } else {
      // Non-existent urdf limits are zero initialized
      upper_pos_limits(i) =
          robot_model.getJoint(m_joint_names[i])->limits->upper;
      lower_pos_limits(i) =
          robot_model.getJoint(m_joint_names[i])->limits->lower;
      m_joint_effort_limits(i) =
          robot_model.getJoint(m_joint_names[i])->limits->effort;
    }
  }

  // Initialize solvers
  // m_ik_solver->init(get_node(),m_robot_chain,upper_pos_limits,lower_pos_limits);
  KDL::Vector grav(0.0, 0.0, -9.81);
  KDL::Tree tmp("not_relevant");
  tmp.addChain(m_robot_chain, "not_relevant");
  m_forward_kinematics_solver.reset(new KDL::TreeFkSolverPos_recursive(tmp));
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_robot_chain));
  m_ik_solver_vel.reset(new KDL::ChainIkSolverVel_pinv(m_robot_chain));
  m_ik_solver.reset(new KDL::ChainIkSolverPos_NR_JL(
      m_robot_chain, lower_pos_limits, upper_pos_limits, *m_fk_solver,
      *m_ik_solver_vel, 100, 1e-6));
  m_jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(m_robot_chain));
  m_dyn_solver.reset(new KDL::ChainDynParam(m_robot_chain, grav));
  RCLCPP_INFO(get_node()->get_logger(),
              "Finished initializing kinematics solvers");

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Robot Chain: ");
  for (unsigned int i = 0; i < m_robot_chain.getNrOfSegments(); ++i) {
    KDL::Segment segment = m_robot_chain.getSegment(i);
    KDL::Joint joint = segment.getJoint();
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Segment " << i << ": " << segment.getName());
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Joint " << i << ": " << joint.getName());
  }
  // Check command interfaces.
  // We support effort.
  m_cmd_interface_types =
      get_node()->get_parameter("command_interfaces").as_string_array();
  if (m_cmd_interface_types.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  for (const auto &type : m_cmd_interface_types) {
    if (type != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unsupported command interface: %s. Choose effort",
                   type.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::ERROR;
    }
  }
  m_state_interface_types =
      get_node()->get_parameter("state_interfaces").as_string_array();
  if (m_state_interface_types.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No state_interfaces specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  // Check if kuka is been used
  m_kuka_hw = get_node()->get_parameter("kuka_hw").as_bool();
  if (m_kuka_hw == true) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Using Kuka, the position will be overwritten at each control cycle to "
        "make the robot behave as in gravity compensation mode");
    m_cmd_interface_types.push_back(hardware_interface::HW_IF_POSITION);
  }
  m_configured = true;

  // Initialize effords to null
  m_efforts = ctrl::VectorND::Zero(m_joint_number);

  // Initialize joint state
  m_joint_positions.resize(m_joint_number);
  m_joint_velocities.resize(m_joint_number);
  m_simulated_joint_motion.resize(m_joint_number);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Base on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EffortControllerBase::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  if (m_active) {
    m_joint_cmd_eff_handles.clear();
    // m_joint_cmd_pos_handles.clear();
    // m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    m_joint_state_vel_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EffortControllerBase::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  if (m_active) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Getting interfaces");

  // Get command handles.
  // Position
  if (m_kuka_hw == true) {
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, m_joint_names,
            hardware_interface::HW_IF_POSITION, m_joint_cmd_pos_handles)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected %zu '%s' command interfaces, got %zu.",
                   m_joint_number, hardware_interface::HW_IF_POSITION,
                   m_joint_cmd_pos_handles.size());
      return CallbackReturn::ERROR;
    }
  }
  // Effort
  if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, m_joint_names, hardware_interface::HW_IF_EFFORT,
          m_joint_cmd_eff_handles)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' command interfaces, got %zu.",
                 m_joint_number, hardware_interface::HW_IF_EFFORT,
                 m_joint_cmd_eff_handles.size());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Finished getting command interfaces");
  // Get state handles.
  // Position
  m_controller_name = hardware_interface::HW_IF_POSITION;
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, m_joint_names, hardware_interface::HW_IF_POSITION,
          m_joint_state_pos_handles)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' state interfaces, got %zu.", m_joint_number,
                 hardware_interface::HW_IF_POSITION,
                 m_joint_state_pos_handles.size());
    return CallbackReturn::ERROR;
  }

  // Velocity
  m_controller_name = hardware_interface::HW_IF_POSITION;
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, m_joint_names, hardware_interface::HW_IF_VELOCITY,
          m_joint_state_vel_handles)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' state interfaces, got %zu.", m_joint_number,
                 hardware_interface::HW_IF_VELOCITY,
                 m_joint_state_vel_handles.size());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Finished getting state interfaces");
  // Copy joint state to internal simulation
  // if (!m_ik_solver->setStartState(m_joint_state_pos_handles))
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Could not set start state");
  //   return
  //   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  // };
  // m_ik_solver->updateKinematics();

  // Provide safe command buffers with starting where we are
  // computeJointEffortCmds(ctrl::VectorND::Zero(m_joint_number));
  // writeJointEffortCmds();

  m_active = true;
  RCLCPP_INFO(get_node()->get_logger(), "Finished Base on_activate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void EffortControllerBase::writeJointEffortCmds() {
  // Write all available types.
  for (const auto &type : m_cmd_interface_types) {
    if (type == hardware_interface::HW_IF_EFFORT) {
      for (size_t i = 0; i < m_joint_number; ++i) {
        // Effort saturation
        m_efforts[i] = std::clamp(m_efforts[i], -m_joint_effort_limits(i),
                                  m_joint_effort_limits(i));
        m_joint_cmd_eff_handles[i].get().set_value(m_efforts[i]);
      }
    }
  }
  if (m_kuka_hw == true) {
    for (size_t i = 0; i < m_joint_number; ++i) {
      m_joint_cmd_pos_handles[i].get().set_value(m_joint_positions(i));
    }
  }
}

void EffortControllerBase::computeJointEffortCmds(const ctrl::VectorND &tau) {
  // Saturation of torque rate
  for (size_t i = 0; i < m_joint_number; i++) {
    const double difference = tau[i] - m_efforts[i];
    m_efforts[i] +=
        std::min(std::max(difference, -m_delta_tau_max), m_delta_tau_max);
    if (std::abs(difference) > m_delta_tau_max) {
      // RCLCPP_WARN(get_node()->get_logger(),
      //             "Joint %s effort rate saturated, was: %f",
      //             m_joint_names[i].c_str(), tau[i]);
    }
  }
}

void EffortControllerBase::computeIKSolution(
    const KDL::Frame &desired_pose, ctrl::VectorND &simulated_joint_positions) {
  // Invese kinematics
  int ret = m_ik_solver->CartToJnt(m_joint_positions, desired_pose,
                                   m_simulated_joint_motion);

  // Check if solution was found
  if (ret < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find IK solution");
    return;
  }

  simulated_joint_positions = m_simulated_joint_motion.data;
}

ctrl::Vector6D
EffortControllerBase::displayInBaseLink(const ctrl::Vector6D &vector,
                                        const std::string &from) {
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i) {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_joint_positions, transform_kdl,
                                         from);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i) {
    out[i] = wrench_kdl(i);
  }

  return out;
}

ctrl::Matrix6D
EffortControllerBase::displayInBaseLink(const ctrl::Matrix6D &tensor,
                                        const std::string &from) {
  // Get rotation to base
  KDL::Frame R_kdl;
  m_forward_kinematics_solver->JntToCart(m_joint_positions, R_kdl, from);

  // Adjust format
  ctrl::Matrix3D R;
  R << R_kdl.M.data[0], R_kdl.M.data[1], R_kdl.M.data[2], R_kdl.M.data[3],
      R_kdl.M.data[4], R_kdl.M.data[5], R_kdl.M.data[6], R_kdl.M.data[7],
      R_kdl.M.data[8];

  // Treat diagonal blocks as individual 2nd rank tensors.
  // Display in base frame.
  ctrl::Matrix6D tmp = ctrl::Matrix6D::Zero();
  tmp.topLeftCorner<3, 3>() = R * tensor.topLeftCorner<3, 3>() * R.transpose();
  tmp.bottomRightCorner<3, 3>() =
      R * tensor.bottomRightCorner<3, 3>() * R.transpose();

  return tmp;
}

ctrl::Vector6D
EffortControllerBase::displayInTipLink(const ctrl::Vector6D &vector,
                                       const std::string &to) {
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i) {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_joint_positions, transform_kdl, to);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M.Inverse() * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i) {
    out[i] = wrench_kdl(i);
  }

  return out;
}

void EffortControllerBase::updateJointStates() {
  for (size_t i = 0; i < m_joint_number; ++i) {
    const auto &position_interface = m_joint_state_pos_handles[i].get();
    const auto &velocity_interface = m_joint_state_vel_handles[i].get();

    m_joint_positions(i) = position_interface.get_value();
    m_joint_velocities(i) = velocity_interface.get_value();

    // std::transform(m_joint_positions.begin(), m_joint_positions.end(),
    // m_joint_positions.begin(),
    // [](double val) -> double {
    //     return std::round(val * 10000) / 10000;
    // });
    // Rount to 4 decimal places
    // m_joint_positions(i) = std::round(m_joint_positions(i) * 10000) / 10000;
  }
}

} // namespace effort_controller_base