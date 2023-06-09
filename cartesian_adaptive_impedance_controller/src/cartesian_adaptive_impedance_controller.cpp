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
  m_Q_matrix(0) = get_node()->get_parameter("Q").as_double();
  m_Q_matrix(1) = get_node()->get_parameter("Q").as_double();
  m_Q_matrix(2) = get_node()->get_parameter("Q").as_double();

  // Set R matrix
  m_R_matrix(0) = get_node()->get_parameter("R").as_double();
  m_R_matrix(1) = get_node()->get_parameter("R").as_double();
  m_R_matrix(2) = get_node()->get_parameter("R").as_double();

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
  // Update joint states
  Base::updateJointStates();

  // Compute the torque to applay at the joints
  ctrl::VectorND tau_tot = CartesianImpedanceController::computeTorque();

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();
  return controller_interface::return_type::OK;
}

// void CartesianAdaptiveImpedanceController::updateMinimizationVariables()
// {
//   m_external_forces.insert(m_external_forces.begin(),ext_force);
//   m_external_forces.pop_back();

//   ctrl::Vector3D des_force = ImpedanceBase::m_target_wrench.head<3>();
//   m_desired_forces.insert(m_desired_forces.begin(),des_force);
//   m_desired_forces.pop_back();

//   ctrl::Matrix3D des_stiff = ImpedanceBase::m_cartesian_stiffness.topLeftCorner<3,3>();
//   m_desired_stiffness.insert(m_desired_stiffness.begin(),des_stiff);
//   m_desired_stiffness.pop_back();
// }

void CartesianAdaptiveImpedanceController::computeDesiredStiffness()
{
  // updateMinimizationVariables();

  USING_NAMESPACE_QPOASES
  /* Setup data of first QP. */
  real_t H[3*3] = 
  {
    m_R_matrix(0) + m_Q_matrix(0) * Base::m_joint_positions(0), 0.0, 0.0,
    0.0, m_R_matrix(1) + m_Q_matrix(1) * Base::m_joint_positions(1), 0.0,
    0.0, 0.0, m_R_matrix(2) + m_Q_matrix(2) * Base::m_joint_positions(2)
  };
  real_t A[3*3] = 
  {
    Base::m_joint_positions(0), 0.0, 0.0,
    0.0, Base::m_joint_positions(1), 0.0,
    0.0, 0.0, Base::m_joint_positions(2)
  };
  real_t g[3] = {
    - 2 * m_stiffness_min(0) * m_R_matrix(0) - 2 * ImpedanceBase::m_target_wrench(0) * Base::m_joint_positions(0) * m_Q_matrix(0)
      + 2 * m_Q_matrix(0) * ImpedanceBase::m_cartesian_damping(0,0) * Base::m_joint_velocities(0) * Base::m_joint_positions(0),
    - 2 * m_stiffness_min(1) * m_R_matrix(1) - 2 * ImpedanceBase::m_target_wrench(1) * Base::m_joint_positions(1) * m_Q_matrix(1)
      + 2 * m_Q_matrix(1) * ImpedanceBase::m_cartesian_damping(1,1) * Base::m_joint_velocities(1) * Base::m_joint_positions(1),
    - 2 * m_stiffness_min(2) * m_R_matrix(2) - 2 * ImpedanceBase::m_target_wrench(2) * Base::m_joint_positions(2) * m_Q_matrix(2)
      + 2 * m_Q_matrix(2) * ImpedanceBase::m_cartesian_damping(2,2) * Base::m_joint_velocities(2) * Base::m_joint_positions(2)
  };
  real_t lb[3] = { m_stiffness_min(0), m_stiffness_min(1), m_stiffness_min(2)};
  real_t ub[3] = { m_stiffness_max(0), m_stiffness_max(1), m_stiffness_max(2)};
  real_t lbA[3] = { 0.0, 0.0, 0.0 };
  real_t ubA[3] = { 
    m_force_max(0) -  ImpedanceBase::m_cartesian_damping(0,0) * Base::m_joint_velocities(0),
    m_force_max(1) -  ImpedanceBase::m_cartesian_damping(1,1) * Base::m_joint_velocities(1),
    m_force_max(2) -  ImpedanceBase::m_cartesian_damping(2,2) * Base::m_joint_velocities(2)
  };

  /* Setting up QProblem object. */
  QProblem min_problem( 3, 3 );

  /* Solve QP. */
  int nWSR = 10;
  min_problem.init( H,g,A,lb,ub,lbA,ubA, nWSR );

  real_t xOpt[3];
  min_problem.getPrimalSolution( xOpt );

  ImpedanceBase::m_cartesian_stiffness(0,0) = xOpt[0];
  ImpedanceBase::m_cartesian_stiffness(1,1) = xOpt[1];
  ImpedanceBase::m_cartesian_stiffness(2,2) = xOpt[2];
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
